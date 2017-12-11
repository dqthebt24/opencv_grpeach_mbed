#include "mbed.h"
#include "DisplayBace.h"
#include "rtos.h"
#include "define.h"
#include "mStorage.hpp"

// Include for graphic
#include "graphicFramework/myImage.h"
#include "graphicFramework/mGraphic.hpp"

// Define for store facial xml file
#include "haar_cascade.h"
#include "FATFileSystem.h"
#include "HeapBlockDevice.h"

// Include for opencv
#include "cProcess/cProcess.hpp"
#include "opencv_3_1/opencv2/core.hpp"
#include "opencv_3_1/opencv2/imgproc.hpp"
#include "opencv_3_1/opencv2/objdetect.hpp"
#include "opencv_3_1/opencv2/face.hpp"
using namespace cv;
using namespace face;

typedef struct {
    int x;
    int y;
    int radius;
    uint8_t color[2];
}mCircle;

typedef struct {
    int x1;
    int y1;
    int x2;
    int y2;
    uint8_t color[2];
}mLine;

typedef struct {
    vector<Mat> database_image;
    vector<int> database_label;
}face_database_t;

typedef struct {
    Rect obj;
    int label;
}obj_detect_result;

typedef struct {
    vector<mCircle> circles;
    vector<mLine> lines;
    vector<Point> contour;//only have max contour
}gesture_result;

typedef struct {
    vector<Point> Pointstart;
    vector<Point> Pointdepth;
    vector<int> Pointindex;
    int detecthand;
}ConvexPoint;

typedef struct
{
    double hand_max;
    double hand_min;
}Sampling;

/* Hardware */
static DisplayBase Display;
static DigitalOut  lcd_pwon(P7_15);
static DigitalOut  lcd_blon(P8_1);
static PwmOut      lcd_cntrst(P8_15);
static Serial      pc(USBTX, USBRX);
static TouckKey_LCD_shield touch(P4_0, P2_13, I2C_SDA, I2C_SCL);

/* Semaphore and mutex */
rtos::Mutex mMutexProcess;
rtos::Mutex mMutexObjects;
rtos::Mutex mMutexClicked;

static Semaphore semProcessThread(0);
static Semaphore semTouch(0);
static Semaphore semDrawAction(0);

/* Threads */
static Thread *    p_VideoLcdTask = NULL;
static Thread *    p_DrawObjects = NULL;
static Thread *    p_Touch = NULL;

static cStorage *myStorage = NULL;
static vector<obj_detect_result> gObjectDetects;
static gesture_result gGestureResult;
static bool gIsProcessing = false;
static bool gIsDrawing = false;
static int write_buff_num = 0;
static int read_buff_num = 0;
static bool graphics_init_end = false;
static APP_MODE appMode = FACE_DETECTION;
static CLICKED_CODE clickedCode = CLICKED_UNKNOWN;

/****** cache control ******/
static void dcache_clean(void * p_buf, uint32_t size) {
    uint32_t start_addr = (uint32_t)p_buf & 0xFFFFFFE0;
    uint32_t end_addr   = (uint32_t)p_buf + size;
    uint32_t addr;

    /* Data cache clean */
    for (addr = start_addr; addr < end_addr; addr += 0x20) {
        __v7_clean_dcache_mva((void *)addr);
    }
}

/****** LCD ******/
#if(0) /* When needing LCD Vsync interrupt, please make it effective. */
static void IntCallbackFunc_LoVsync(DisplayBase::int_type_t int_type) {
    /* Interrupt callback function for Vsync interruption */
}
#endif

static void Init_LCD_Display(void) {
    DisplayBase::graphics_error_t error;
    DisplayBase::lcd_config_t lcd_config;
    PinName lvds_pin[8] = {
        /* data pin */
        P5_7, P5_6, P5_5, P5_4, P5_3, P5_2, P5_1, P5_0
    };

    lcd_pwon = 0;
    lcd_blon = 0;
    Thread::wait(100);
    lcd_pwon = 1;
    lcd_blon = 1;

    Display.Graphics_Lvds_Port_Init(lvds_pin, 8);

    /* Graphics initialization process */
    lcd_config = LcdCfgTbl_LCD_shield;
    error = Display.Graphics_init(&lcd_config);
    if (error != DisplayBase::GRAPHICS_OK) {
        printf("Line %d, error %d\n", __LINE__, error);
        mbed_die();
    }
    graphics_init_end = true;

#if(0) /* When needing LCD Vsync interrupt, please make it effective. */
    /* Interrupt callback function setting (Vsync signal output from scaler 0)*/
    error = Display.Graphics_Irq_Handler_Set(DisplayBase::INT_TYPE_S0_LO_VSYNC,
                                                    0, IntCallbackFunc_LoVsync);
    if (error != DisplayBase::GRAPHICS_OK) {
        printf("Line %d, error %d\n", __LINE__, error);
        mbed_die();
    }
#endif
}

static void Start_LCD_Display(uint8_t * p_buf) {
    DisplayBase::rect_t rect;

    rect.vs = 0;
    rect.vw = LCD_PIXEL_HEIGHT;
    rect.hs = 0;
    rect.hw = LCD_PIXEL_WIDTH;
    Display.Graphics_Read_Setting(
        DisplayBase::GRAPHICS_LAYER_0,
        (void *)p_buf,
        FRAME_BUFFER_STRIDE,
        GRAPHICS_FORMAT,
        WR_RD_WRSWA,
        &rect
    );
    Display.Graphics_Start(DisplayBase::GRAPHICS_LAYER_0);
}

/****** Video ******/
#if(0) /* When needing video Vsync interrupt, please make it effective. */
static void IntCallbackFunc_ViVsync(DisplayBase::int_type_t int_type) {
    /* Interrupt callback function for Vsync interruption */
}
#endif

static void IntCallbackFunc_Vfield(DisplayBase::int_type_t int_type) {
    /* Interrupt callback function */
#if VIDEO_INPUT_METHOD == VIDEO_CVBS
    if (vfield_count == 0) {
        vfield_count = 1;
    } else {
        vfield_count = 0;
#else
    {
#endif
        if (p_VideoLcdTask != NULL) {
            p_VideoLcdTask->signal_set(1);
        }
    }
}

static void Init_Video(void) {
    DisplayBase::graphics_error_t error;

    /* Graphics initialization process */
    if (graphics_init_end == false) {
        /* When not initializing LCD, this processing is needed. */
        error = Display.Graphics_init(NULL);
        if (error != DisplayBase::GRAPHICS_OK) {
            printf("Line %d, error %d\n", __LINE__, error);
            mbed_die();
        }
        graphics_init_end = true;
    }

#if VIDEO_INPUT_METHOD == VIDEO_CVBS
    error = Display.Graphics_Video_init( DisplayBase::INPUT_SEL_VDEC, NULL);
    if( error != DisplayBase::GRAPHICS_OK ) {
        printf("Line %d, error %d\n", __LINE__, error);
        mbed_die();
    }
#elif VIDEO_INPUT_METHOD == VIDEO_CMOS_CAMERA
    DisplayBase::video_ext_in_config_t ext_in_config;
    PinName cmos_camera_pin[11] = {
        /* data pin */
        P2_7, P2_6, P2_5, P2_4, P2_3, P2_2, P2_1, P2_0,
        /* control pin */
        P10_0,      /* DV0_CLK   */
        P1_0,       /* DV0_Vsync */
        P1_1        /* DV0_Hsync */
    };

    /* MT9V111 camera input config */
    ext_in_config.inp_format     = DisplayBase::VIDEO_EXTIN_FORMAT_BT601; /* BT601 8bit YCbCr format */
    ext_in_config.inp_pxd_edge   = DisplayBase::EDGE_RISING;              /* Clock edge select for capturing data          */
    ext_in_config.inp_vs_edge    = DisplayBase::EDGE_RISING;              /* Clock edge select for capturing Vsync signals */
    ext_in_config.inp_hs_edge    = DisplayBase::EDGE_RISING;              /* Clock edge select for capturing Hsync signals */
    ext_in_config.inp_endian_on  = DisplayBase::OFF;                      /* External input bit endian change on/off       */
    ext_in_config.inp_swap_on    = DisplayBase::OFF;                      /* External input B/R signal swap on/off         */
    ext_in_config.inp_vs_inv     = DisplayBase::SIG_POL_NOT_INVERTED;     /* External input DV_VSYNC inversion control     */
    ext_in_config.inp_hs_inv     = DisplayBase::SIG_POL_INVERTED;         /* External input DV_HSYNC inversion control     */
    ext_in_config.inp_f525_625   = DisplayBase::EXTIN_LINE_525;           /* Number of lines for BT.656 external input */
    ext_in_config.inp_h_pos      = DisplayBase::EXTIN_H_POS_CRYCBY;       /* Y/Cb/Y/Cr data string start timing to Hsync reference */
    ext_in_config.cap_vs_pos     = 6;                                     /* Capture start position from Vsync */
    ext_in_config.cap_hs_pos     = 150;                                   /* Capture start position form Hsync */
#if (LCD_TYPE == 0)
    /* The same screen ratio as the screen ratio of the LCD. */
    ext_in_config.cap_width      = 640;                                   /* Capture width */
    ext_in_config.cap_height     = 363;                                   /* Capture height Max 468[line]
                                                                             Due to CMOS(MT9V111) output signal timing and VDC5 specification */
#else
    ext_in_config.cap_width      = 640;                                   /* Capture width  */
    ext_in_config.cap_height     = 468;                                   /* Capture height Max 468[line]
                                                                             Due to CMOS(MT9V111) output signal timing and VDC5 specification */
#endif
    error = Display.Graphics_Video_init( DisplayBase::INPUT_SEL_EXT, &ext_in_config);
    if( error != DisplayBase::GRAPHICS_OK ) {
        printf("Line %d, error %d\n", __LINE__, error);
        mbed_die();
    }

    /* Camera input port setting */
    error = Display.Graphics_Dvinput_Port_Init(cmos_camera_pin, 11);
    if( error != DisplayBase::GRAPHICS_OK ) {
        printf("Line %d, error %d\n", __LINE__, error);
        mbed_die();
    }
#endif

#if(0) /* When needing video Vsync interrupt, please make it effective. */
    /* Interrupt callback function setting (Vsync signal input to scaler 0) */
    error = Display.Graphics_Irq_Handler_Set(DisplayBase::INT_TYPE_S0_VI_VSYNC,
                                                 0, IntCallbackFunc_ViVsync);
    if (error != DisplayBase::GRAPHICS_OK) {
        printf("Line %d, error %d\n", __LINE__, error);
        mbed_die();
    }
#endif

    /* Interrupt callback function setting (Field end signal for recording function in scaler 0) */
    error = Display.Graphics_Irq_Handler_Set(VIDEO_INT_TYPE, 0, 
                                                        IntCallbackFunc_Vfield);
    if (error != DisplayBase::GRAPHICS_OK) {
        printf("Line %d, error %d\n", __LINE__, error);
        mbed_die();
    }
}

static void initDrawResultLayer(void)
{
    DisplayBase::rect_t rect;
    /* The layer by which the touch panel location is drawn */
    memset(user_frame_buffer_draw, 0, sizeof(user_frame_buffer_draw));
    dcache_clean(user_frame_buffer_draw, sizeof(user_frame_buffer_draw));
    rect.vs = 0;
    rect.vw = LCD_PIXEL_HEIGHT;
    rect.hs = 0;
    rect.hw = LCD_PIXEL_WIDTH;
    Display.Graphics_Read_Setting(
        DisplayBase::GRAPHICS_LAYER_1,
        (void *)user_frame_buffer_draw,
        DRAW_BUFFER_STRIDE,
        DisplayBase::GRAPHICS_FORMAT_ARGB4444,
        DisplayBase::WR_RD_WRSWA_32_16BIT,
        &rect
    );
    Display.Graphics_Start(DisplayBase::GRAPHICS_LAYER_1);
}

void initDrawAction32(void)
{
    // Init draw control
    DisplayBase::rect_t rect;
    rect.vs = 0;
    rect.vw = LCD_PIXEL_HEIGHT;
    rect.hs = 0;
    rect.hw = LCD_PIXEL_WIDTH;
    
    memset(user_buf_draw_action_888, 0, 
                                        sizeof(user_buf_draw_action_888));     
    /* Clean cache */
    dcache_clean(user_buf_draw_action_888, 
                                        sizeof(user_buf_draw_action_888));
    
    Display.Graphics_Read_Setting(
        DisplayBase::GRAPHICS_LAYER_3,
        (void *)user_buf_draw_action_888,
        DRAW_BUFFER_STRIDE_LAYER_3,
        DisplayBase::GRAPHICS_FORMAT_ARGB8888,
        DisplayBase::WR_RD_WRSWA_32_16BIT,
        &rect
    );
    Display.Graphics_Start(DisplayBase::GRAPHICS_LAYER_3);
}
static void initDrawButtonLayer(void)
{
    DisplayBase::rect_t rect;
    /* The layer by which the touch panel location is drawn */
    memset(user_frame_buffer_draw_button, 0, 
                                        sizeof(user_frame_buffer_draw_button));
    dcache_clean(user_frame_buffer_draw_button, 
                                        sizeof(user_frame_buffer_draw_button));
    rect.vs = 0;
    rect.vw = LCD_PIXEL_HEIGHT;
    rect.hs = 0;
    rect.hw = LCD_PIXEL_WIDTH;
    Display.Graphics_Read_Setting(
        DisplayBase::GRAPHICS_LAYER_2,
        (void *)user_frame_buffer_draw_button,
        DRAW_BUFFER_STRIDE,
        DisplayBase::GRAPHICS_FORMAT_ARGB4444,
        DisplayBase::WR_RD_WRSWA_32_16BIT,
        &rect
    );
    Display.Graphics_Start(DisplayBase::GRAPHICS_LAYER_2);
}

static void Start_Video(uint8_t * p_buf) {
    DisplayBase::graphics_error_t error;

    /* Video capture setting (progressive form fixed) */
    error = Display.Video_Write_Setting(
                VIDEO_INPUT_CH,
                COL_SYS,
                p_buf,
                FRAME_BUFFER_STRIDE,
                VIDEO_FORMAT,
                WR_RD_WRSWA,
                VIDEO_PIXEL_VW,
                VIDEO_PIXEL_HW
            );
    if (error != DisplayBase::GRAPHICS_OK) {
        printf("Line %d, error %d\n", __LINE__, error);
        mbed_die();
    }

    /* Video write process start */
    error = Display.Video_Start(VIDEO_INPUT_CH);
    if (error != DisplayBase::GRAPHICS_OK) {
        printf("Line %d, error %d\n", __LINE__, error);
        mbed_die();
    }

    /* Video write process stop */
    error = Display.Video_Stop(VIDEO_INPUT_CH);
    if (error != DisplayBase::GRAPHICS_OK) {
        printf("Line %d, error %d\n", __LINE__, error);
        mbed_die();
    }

    /* Video write process start */
    error = Display.Video_Start(VIDEO_INPUT_CH);
    if (error != DisplayBase::GRAPHICS_OK) {
        printf("Line %d, error %d\n", __LINE__, error);
        mbed_die();
    }
}

/* Select function from button */
APP_MODE getFunctionSelected(int x, int y)
{
    int w = 100; //width of image button
    if ( MODE_BTN_Y > y )
    {
        return MODE_UNKNOWN;
    }
    
    if ( MODE_BTN_X <= x && x <= (MODE_BTN_X + w))
    {
        return FACE_DETECTION;
    }
    else if ( (2*MODE_BTN_X + w) <= x && x <= (2*MODE_BTN_X + 2*w))
    {
        return FACE_RECOGNITION;
    }
    else if ( (3*MODE_BTN_X + 2*w) <= x && x <= (3*MODE_BTN_X + 3*w) )
    {
        return MOTION_DETECTION;
    }
    else if ((4*MODE_BTN_X + 3*w) <= x && x <= (4*MODE_BTN_X + 4*w))
    {
        return GUESTURE_RECOGNITION;
    }
    else
        return MODE_UNKNOWN;
}

/* Click action in face recognition */
void clickActionFaceReg(int x, int y)
{
    mMutexClicked.lock();
    if(REGIS_FACE_BTN_X <= x && x <= (REGIS_FACE_BTN_X + 30) &&
       REGIS_FACE_BTN_Y <= y && y <= (REGIS_FACE_BTN_Y + 30))
    {
        clickedCode = CLICKED_REGIS_FACE;
    }
    else if (FACE_REG_ACT_MENU_X <= x && x <= (FACE_REG_ACT_MENU_X + 60) &&
        FACE_REG_ACT_MENU_Y <= y && y <= (FACE_REG_ACT_MENU_Y + 20))
    {
        clickedCode = CLICKED_CHANGE_ID;
    }
    else if (FACE_REG_ACT_MENU_X <= x && x <= (FACE_REG_ACT_MENU_X + 60) &&
        (FACE_REG_ACT_MENU_Y + 20 + 12)<= y && 
        y <= (FACE_REG_ACT_MENU_Y + 2*20 + 12))//space:12
    {
        clickedCode = CLICKED_ADD;
    }
    else if (FACE_REG_ACT_MENU_X <= x && x <= (FACE_REG_ACT_MENU_X + 60) &&
        (FACE_REG_ACT_MENU_Y + 2*20 + 2*12)<= y && 
        y <= (FACE_REG_ACT_MENU_Y + 3*20 + 2*12))
    {
        clickedCode = CLICKED_IGNORE;
    }
    else
    {
        clickedCode = CLICKED_UNKNOWN;
    }
    mMutexClicked.unlock();
}

/* Click on screen gesture recognition */
void clickActionGestureReg(int x, int y)
{
    mMutexClicked.lock();
    if(GESTURE_SAMPLING_BTN_X <= x && x <= (GESTURE_SAMPLING_BTN_X + 60) &&
       GESTURE_SAMPLING_BTN_Y <= y && y <= (GESTURE_SAMPLING_BTN_Y + 20))
    {
        clickedCode = CLICKED_HAND_SAMPLING;
    }
    else
    {
        clickedCode = CLICKED_UNKNOWN;
    }
    mMutexClicked.unlock();
}

/**** Draw button controls ****/
void drawButtonControls(graphicFrameworkCanvas myCanvasButton)
{
    int space = 16;
    int x = 0;
    int y = MODE_BTN_Y;
    
    x+=space;
    myCanvasButton.drawImage(my_img_button_face_detect,x,y); // 100x40
    x+=100;
    
    x+=space;
    myCanvasButton.drawImage(my_img_button_face_recognition,x,y);
    x+=100;

    x+=space;
    myCanvasButton.drawImage(my_img_button_obj_motion,x,y);
    x+=100;
    
    x+=space;
    myCanvasButton.drawImage(my_img_button_gesture_reg,x,y);
    x+=100;
}

void drawTextScreen(graphicFrameworkCanvas myCanvasButton)
{
    int x = 160;
    int y = 5;
    
    if( appMode == MOTION_DETECTION )
    {
        myCanvasButton.drawImage(my_img_text_obj_motion,x, y);
    }
    else if ( appMode == FACE_DETECTION )
    {
        myCanvasButton.drawImage(my_img_text_face_detection,x, y);
    }
    else if ( appMode == FACE_RECOGNITION )
    {
        myCanvasButton.drawImage(my_img_text_face_recognition,x, y);
    }
    else if ( appMode == GUESTURE_RECOGNITION )
    {
        myCanvasButton.drawImage(my_img_text_gesture_reg,x, y);
    }
}

void clearGestureResult(gesture_result res)
{
    res.circles.clear();
    res.lines.clear();
    res.contour.clear();
}
/****** Thread image process ******/
static void img_draw_objects(void)
{
    graphicFrameworkCanvas myCanvas(user_frame_buffer_draw, 0x01E0,
                    0x0110, (uint8_t)DRAW_BUFFER_BYTE_PER_PIXEL, 
                    (uint8_t)DRAW_POINT,0x06);

    initDrawResultLayer();
    
    uint8_t color[2] = {0x0B,0xFF};// Color Pink
    APP_MODE oldMode = appMode;
    while(1)
    {
        Thread::signal_wait(1);        
        memset(user_frame_buffer_draw, 0, sizeof(user_frame_buffer_draw));

        if ( oldMode != appMode)
        {
            oldMode = appMode;
            
            mMutexObjects.lock();
            gIsDrawing = false;
            mMutexObjects.unlock();     
                   
            // Data cache clean 
            dcache_clean(user_frame_buffer_draw, sizeof(user_frame_buffer_draw));
            continue;
        }

        // Draw faces on screen
        switch(appMode)
        {
            case FACE_DETECTION:
            case MOTION_DETECTION:
            {
                if(gObjectDetects.size() > 0)
                {
                    for(int i = 0;i < gObjectDetects.size();i++)
                    {
                        int _x = gObjectDetects[i].obj.x*IMG_DOWN_SAMPLE;
                        int _y = gObjectDetects[i].obj.y*IMG_DOWN_SAMPLE;
                        int _w = gObjectDetects[i].obj.width*IMG_DOWN_SAMPLE;
                        int _h = gObjectDetects[i].obj.height*IMG_DOWN_SAMPLE;
                        myCanvas.drawRect(_x,_y,_w,_h,color);
                    }
                }
                gObjectDetects.clear();
                break;
            }
            case FACE_RECOGNITION:
            {
                if(gObjectDetects.size() > 0)
                {
                    for(int i = 0;i < gObjectDetects.size();i++)
                    {
                        int _x = gObjectDetects[i].obj.x*IMG_DOWN_SAMPLE;
                        int _y = gObjectDetects[i].obj.y*IMG_DOWN_SAMPLE;
                        int _w = gObjectDetects[i].obj.width*IMG_DOWN_SAMPLE;
                        int _h = gObjectDetects[i].obj.height*IMG_DOWN_SAMPLE;
                        if(gObjectDetects[i].label == 1)
                        {
                            // Yellow
                            color[0] = 0xF0;
                            color[1] = 0xFF;
                        }
                        else if (gObjectDetects[i].label == 2)
                        {
                            // Green
                            color[0] = 0xF0;
                            color[1] = 0xF0;
                        }
                        else
                        {
                            // Pink
                            color[0] = 0x0B;
                            color[1] = 0xFF;
                        }
                        myCanvas.drawRect(_x,_y,_w,_h,color);
                    }
                }
                gObjectDetects.clear();
                break;
            }
            case GUESTURE_RECOGNITION:
            {
                uint8_t contoursColor[2] = {0xF0,0xFF};
                    
                //Draw max contour
                for(int i = 0; i < gGestureResult.contour.size();i++)//i=0
                {
                    myCanvas.draw_pixel(gGestureResult.contour[i].x,
                            gGestureResult.contour[i].y,contoursColor);
                }
                // Draw lines, circles
                for(int i = 0;i < gGestureResult.lines.size();i++)
                {
                    myCanvas.drawLine(gGestureResult.lines[i].x1,
                                      gGestureResult.lines[i].y1,
                                      gGestureResult.lines[i].x2,
                                      gGestureResult.lines[i].y2,
                                      gGestureResult.lines[i].color);
                }
                
                for(int i = 0;i < gGestureResult.circles.size();i++)
                {
                    myCanvas.drawCircle(gGestureResult.circles[i].x,
                                        gGestureResult.circles[i].y,
                                        gGestureResult.circles[i].radius,
                                        gGestureResult.circles[i].color);
                }
                break;
            }
            default:
            break;
        } // switch

        mMutexObjects.lock();
        gIsDrawing = false;
        mMutexObjects.unlock();
        
        // Data cache clean 
        dcache_clean(user_frame_buffer_draw, sizeof(user_frame_buffer_draw));
    }//while
}

/* Touch task*/
static void touch_int_callback(void) 
{
    semTouch.release();
}
static void touch_task(void) 
{
    graphicFrameworkCanvas myCanvasButton(user_frame_buffer_draw_button, 0x01E0,
                    0x0110, (uint8_t)DRAW_BUFFER_BYTE_PER_PIXEL, 
                    (uint8_t)DRAW_POINT,0x06);
    TouchKey::touch_pos_t touch_pos[TOUCH_NUM];
    
    /* Callback setting */
    touch.SetCallback(&touch_int_callback);

    /* Reset touch IC */
    touch.Reset();
    
    initDrawButtonLayer();
    drawTextScreen(myCanvasButton);
    drawButtonControls(myCanvasButton);
    
    int x_touch = -1, y_touch = -1;

    set_time(0);//set_time(1498450609);//11:15,Mon-26-June
    double seconds = (double)time(NULL);
    
    while(1)
    {
        /* Wait touch event */
        semTouch.wait();
        printf("USER touched!\n");

        touch.GetCoordinates(TOUCH_NUM, touch_pos);

        for (int i = 0; i < TOUCH_NUM; i ++) {
            if (touch_pos[i].valid) {
                int new_touch_x = touch_pos[i].x;
                int new_touch_y = touch_pos[i].y;
                APP_MODE mode = getFunctionSelected(new_touch_x, new_touch_y);

                if(mode != appMode && mode != MODE_UNKNOWN)
                {
                    appMode = mode;
                    memset(user_frame_buffer_draw_button, 0, sizeof(uint8_t)*LCD_PIXEL_WIDTH*(MODE_BTN_Y)*FRAME_BUFFER_BYTE_PER_PIXEL);
                    drawTextScreen(myCanvasButton);
                    dcache_clean(user_frame_buffer_draw_button, sizeof(user_frame_buffer_draw_button));
                }
                else if (appMode == FACE_RECOGNITION)
                {
                    double time_1 = (double)time(NULL) - seconds;
                    int posAbs = abs(new_touch_x - x_touch) + 
                                      abs(new_touch_y - y_touch); 
                    
                    if(posAbs > 5 || time_1 > 0.5) //1s
                    {
                        x_touch = new_touch_x;
                        y_touch = new_touch_y;

                        clickActionFaceReg(new_touch_x,new_touch_y);

                    }
                    seconds = (double)time(NULL);
                }
                else if (appMode == GUESTURE_RECOGNITION)
                {
                    double time_1 = (double)time(NULL) - seconds;
                    int posAbs = abs(new_touch_x - x_touch) + 
                                      abs(new_touch_y - y_touch); 

                    if(posAbs > 5 || time_1 > 0.5) //1s
                    {
                        x_touch = new_touch_x;
                        y_touch = new_touch_y;
                        clickActionGestureReg(new_touch_x,new_touch_y);
                    }

                    seconds = (double)time(NULL);
                }
            }
        }
    }
}

/****** Video input is output to LCD ******/
static void video_lcd_task(void) {
    DisplayBase::graphics_error_t error;
    int wk_num;

    /* Initialization memory */
    for (int i = 0; i < FRAME_BUFFER_NUM; i++) {
        memset(FrameBufferTbl[i], 0, (FRAME_BUFFER_STRIDE * LCD_PIXEL_HEIGHT));
        dcache_clean(FrameBufferTbl[i],(FRAME_BUFFER_STRIDE * LCD_PIXEL_HEIGHT));
    }

    /* Start of Video */
    Start_Video(FrameBufferTbl[write_buff_num]);

    /* Wait for first video drawing */
    Thread::signal_wait(1);
    write_buff_num++;
    if (write_buff_num >= FRAME_BUFFER_NUM) {
        write_buff_num = 0;
    }
    error = Display.Video_Write_Change(VIDEO_INPUT_CH, 
                        FrameBufferTbl[write_buff_num], FRAME_BUFFER_STRIDE);
    if (error != DisplayBase::GRAPHICS_OK) {
        printf("Line %d, error %d\n", __LINE__, error);
        mbed_die();
    }

    /* Start of LCD */
    Start_LCD_Display(FrameBufferTbl[read_buff_num]);

    /* Backlight on */
    Thread::wait(200);
    lcd_cntrst.write(1.0);

    while (1) {
        Thread::signal_wait(1);
        wk_num = write_buff_num + 1;
        if (wk_num >= FRAME_BUFFER_NUM) {
            wk_num = 0;
        }
        /* If the next buffer is empty, it's changed. */
        if (wk_num != read_buff_num) {
            read_buff_num  = write_buff_num;
            write_buff_num = wk_num;
            
            /* Change video buffer */
            error = Display.Video_Write_Change(VIDEO_INPUT_CH, 
                    FrameBufferTbl[0/*write_buff_num*/], FRAME_BUFFER_STRIDE);
            if (error != DisplayBase::GRAPHICS_OK) {
                printf("Line %d, error %d\n", __LINE__, error);
                mbed_die();
            }
            
            mMutexProcess.lock();
            if ( gIsProcessing == false)
            {
                gIsProcessing = true;    
                mMutexProcess.unlock();

                memcpy((void *)my_frame,(void*)FrameBufferTbl[0],
                                        FRAME_BUFFER_STRIDE * LCD_PIXEL_HEIGHT);
                semProcessThread.release();
            }
            else
            {
                mMutexProcess.unlock();
            }
            
            /* Change LCD buffer */
            Display.Graphics_Read_Change(DisplayBase::GRAPHICS_LAYER_0, 
                                    (void *)FrameBufferTbl[0/*read_buff_num*/]);
        }
    }
}

bool initSdCard()
{
    if(myStorage == NULL)
    {
        return false;
    }
    if (myStorage->isConnectSdCard() == STORG_PASS) 
    {
        if(myStorage->mountSdCard() == STORG_PASS)
        {
            return true;
        }
        else
        {
            // mount sdcard failed!
            return false;
        }
    }
    else
    {
        // no sdcard!
        togle_led(LED_RED);
        return false;
    }
}
void faceDectectionApp(void)
{
    Size face_size(LCD_PIXEL_WIDTH/IMG_DOWN_SAMPLE,
                                            LCD_PIXEL_HEIGHT/IMG_DOWN_SAMPLE);
    Mat smallImage = Mat::zeros(face_size, CV_8UC1);
    CascadeClassifier haar_cascade;
    vector<Rect> faces;
    printf("face detection load file!\n");
    if (!haar_cascade.load(HAAR_CASCADE_FACE_PATH))
    {
        // load failed
        togle_led(LED_RED);
        while(1)
        {
            wait(0.5);
        }
    }
    else{
        togle_led(LED_GREEN);
    }

    printf("face detection load file done!\n");
    togle_reset(LED_RED,LED_BLUE);
    faces.clear();
    
    gObjectDetects.clear();
    while (1) {
        semProcessThread.wait();
        
        if ( appMode != FACE_DETECTION )
        {
            mMutexProcess.lock();
            gIsProcessing = false;
            mMutexProcess.unlock();
            break;
        }
        
        Mat gray;
        {
            Mat res = _m_cvtRgb5652Rgb(my_frame,(int)LCD_PIXEL_WIDTH,
                                                    (int)LCD_PIXEL_HEIGHT,2);
            cvtColor(res,gray,COLOR_RGB2GRAY);
            res.release();
        }

        equalizeHist(gray,gray);
        resize(gray, smallImage, face_size);

        haar_cascade.detectMultiScale(smallImage,faces,1.1,
                                        2,0|CV_HAAR_SCALE_IMAGE,Size(10,10));        
        mMutexObjects.lock();
        if(gIsDrawing == false)
        {
            mMutexObjects.unlock();
            for(int i = 0;i < faces.size();i++)
            {
                obj_detect_result tmp;
                tmp.obj = faces[i];
                tmp.label = -1;
                gObjectDetects.push_back(tmp);
            }
            //gObjectDetects = faces;
            gIsDrawing = true;
            p_DrawObjects->signal_set(1);
        }
        else
        {
            mMutexObjects.unlock();
        }

        /* Reset flag */
        faces.clear();
        mMutexProcess.lock();
        gIsProcessing = false;
        mMutexProcess.unlock();
    };
}

void motionDetectionApp(void)
{
    bool isNewStart = true;
    int threshValue = 30;
    int blurSize = 5;
    float mArea = 0.0;
    float mMaxArea = 0.0;
    int mMaxIndex = 0;
    bool _isDrawing;
    int mCounter = 0;
    Mat curFrameGray;
    Mat prevFrameGray;
    Mat diffImage;
    Mat binImage;
    vector<vector<Point> > contours;
    vector<Rect> _objects;

    gObjectDetects.clear();
    while(1)
    {
        semProcessThread.wait();
        
        if ( appMode != MOTION_DETECTION )
        {
            mMutexProcess.lock();
            gIsProcessing = false;
            mMutexProcess.unlock();
            
            break;
        }

        _objects.clear();
        mCounter++;
        if( isNewStart == false)
        {
            {
                Mat res = _m_cvtRgb5652Rgb(my_frame,(int)LCD_PIXEL_WIDTH,
                                                    (int)LCD_PIXEL_HEIGHT,2);
                cvtColor(res,curFrameGray,COLOR_RGB2GRAY);
                res.release();
            }
            /* differential between foreground and background */
            absdiff(curFrameGray, prevFrameGray, diffImage);
            blur(diffImage, diffImage, Size(blurSize, blurSize));
            threshold(diffImage, binImage, threshValue, 255, CV_THRESH_BINARY);
            findContours(binImage, contours, CV_RETR_EXTERNAL, 
                                                        CV_CHAIN_APPROX_NONE);

            if(contours.size() > 0)
            {
                for (int i = 0; i < contours.size(); i++)
                {
                    mArea = contourArea(contours[i]);
                    if (mArea > mMaxArea)
                    {
                        mMaxArea = mArea;
                        mMaxIndex = i;
                    }
                }
                
                if (mMaxArea > MAX_COUNTOURS)
                {
                    Rect objectBoundingRectangle = boundingRect(
                                                        contours.at(mMaxIndex));
                    _objects.push_back(objectBoundingRectangle);
                }
            }
            
            /* Set display motion objects */
            mMutexObjects.lock();
            _isDrawing = gIsDrawing;
            mMutexObjects.unlock();
            
            if(_isDrawing == false)
            {
                for(int i = 0;i < _objects.size();i++)
                {
                    obj_detect_result tmp;
                    tmp.obj = _objects[i];
                    tmp.label = -1;
                    gObjectDetects.push_back(tmp);
                }
                //gObjectDetects = _objects;
                gIsDrawing = true;
                p_DrawObjects->signal_set(1);
            }
            
            /* Reset values */
            mMaxArea = 0;
            mArea = 0;
            mMaxIndex = 0;
            
            /* Update background */
            if(mCounter == 50)
            {
                mCounter = 0;
                isNewStart = true;
            }
        }
        else
        {
            isNewStart = false;
            prevFrameGray = _m_cvtRgb5652Rgb(my_frame,(int)LCD_PIXEL_WIDTH,
                                                    (int)LCD_PIXEL_HEIGHT,2);
            cvtColor(prevFrameGray,prevFrameGray,COLOR_RGB2GRAY);
        }
        
        mMutexProcess.lock();
        gIsProcessing = false;
        mMutexProcess.unlock();
    };
}

void faceRecognitionApp(void)
{
    // Init draw
    graphicFrameworkCanvas myCanvas(user_buf_draw_action_888, 0x01E0,
                    0x0110, 0x04, 
                    (uint8_t)DRAW_POINT,0x06);

    uint8_t label_id = 1;
    bool oldStateFace = false;// no faces
    // End init draw
    
    Size face_size(LCD_PIXEL_WIDTH/IMG_DOWN_SAMPLE,
                                            LCD_PIXEL_HEIGHT/IMG_DOWN_SAMPLE);
    Mat smallImage = Mat::zeros(face_size, CV_8UC1);
    CascadeClassifier haar_cascade;
    vector<Rect> faces;
    
    //Init recognizer
    int radius = 2, neighbors = 4, gridx = 8, gridy = 8, thresmax = 50;
    Ptr<LBPHFaceRecognizer> modelReg = createLBPHFaceRecognizer(radius, neighbors,
                                    gridx, gridy, thresmax);
    face_database_t face_database;
    Size regSize(70,70);
    bool flag_class = false;
    bool flag_train = false;
    bool isFirstTrain = true;
    bool changeApp = false;
    
    if (!haar_cascade.load(HAAR_CASCADE_FACE_PATH))
    {
        // load failed
        togle_led(LED_RED);
        while(1)
        {
            wait(0.5);
        }
    }
    else{
        togle_led(LED_GREEN);
    }
    togle_reset(LED_RED,LED_BLUE);
    faces.clear();
    
    gObjectDetects.clear();
    while(1)
    {
        semProcessThread.wait();

        if ( appMode != FACE_RECOGNITION )
        {
            mMutexProcess.lock();
            gIsProcessing = false;
            mMutexProcess.unlock();
            memset(user_buf_draw_action_888, 0, 
                                    sizeof(user_buf_draw_action_888));     
            break;
        }
        
        Mat gray;
        Mat imgRgb = _m_cvtRgb5652Rgb(my_frame,(int)LCD_PIXEL_WIDTH,
                                                    (int)LCD_PIXEL_HEIGHT,2);
        cvtColor(imgRgb,gray,COLOR_RGB2GRAY);

        equalizeHist(gray,gray);
        resize(gray, smallImage, face_size);

        haar_cascade.detectMultiScale(smallImage,faces,1.1,
                                        2,0|CV_HAAR_SCALE_IMAGE,Size(10,10));
        
        mMutexObjects.lock();
        if(gIsDrawing == false)
        {
            mMutexObjects.unlock();

            if( faces.size() > 0)
            {
                // Show label
                if (oldStateFace == false)
                {
                    oldStateFace = true;
                    myCanvas.drawImage(my_img_register_face,    
                                        REGIS_FACE_BTN_X ,REGIS_FACE_BTN_Y);
                }
                
                // Click update database
                mMutexClicked.lock();
                if(clickedCode == CLICKED_REGIS_FACE)
                {
                    mMutexClicked.unlock();
                    
                    // Clean draw faces
                    mMutexObjects.lock();
                    if(gIsDrawing == false)
                    {
                        mMutexObjects.unlock();
                        gObjectDetects.clear();
                        gIsDrawing = true;
                        p_DrawObjects->signal_set(1);
                    }
                    else
                    {
                        mMutexObjects.unlock();
                    }

                    // Update database/ train data
                    face_database.database_image.clear();
                    face_database.database_label.clear();
                    flag_train = false;
                    
                    for (int i = 0; i < faces.size(); i++)
                    {
                        // Check exit app
                        if(changeApp == true)
                        {
                            break;
                        }
                        mMutexClicked.lock();
                        clickedCode = CLICKED_UNKNOWN;
                        mMutexClicked.unlock();
                        
                        int _x = faces[i].x*IMG_DOWN_SAMPLE;
                        int _y = faces[i].y*IMG_DOWN_SAMPLE;
                        int _w = faces[i].width*IMG_DOWN_SAMPLE;
                        int _h = faces[i].height*IMG_DOWN_SAMPLE;
                        
                        Rect roi(_x,_y,_w,_h);
                        
                        Mat imgShow = imgRgb(roi);
                        if(_w > 100)
                        {
                            resize(imgShow,imgShow,Size(100,100));
                            _w = 100;
                            _h = 100;
                        }

                        uint8_t* src = cvtMat2RGBA888(_w,_h,imgShow);
                        
                        label_id = 1;
                        
                        memset(user_buf_draw_action_888, 0, 
                                    sizeof(user_buf_draw_action_888));
                        myCanvas.drawImage(src,100, 90);
                        myCanvas.drawImage(my_img_id_01,    
                                        FACE_REG_ID_MENU_X ,FACE_REG_ID_MENU_Y);
                        myCanvas.drawImage(my_img_change_id,    
                                FACE_REG_ACT_MENU_X ,FACE_REG_ACT_MENU_Y);
                        myCanvas.drawImage(my_img_add_this,    
                            FACE_REG_ACT_MENU_X ,FACE_REG_ACT_MENU_Y + 12 + 20);
                        myCanvas.drawImage(my_img_ignore,    
                            FACE_REG_ACT_MENU_X ,FACE_REG_ACT_MENU_Y+ 24 + 40);
                        
                        /* Clean cache */
                        dcache_clean(user_buf_draw_action_888, 
                                    sizeof(user_buf_draw_action_888));
                        while(1)
                        {             
                            bool isBreak = false;               
                            mMutexClicked.lock();
                            CLICKED_CODE codeTmp = clickedCode;
                            mMutexClicked.unlock();
                            
                            // Check exit app
                            if ( appMode != FACE_RECOGNITION )
                            {
                                changeApp = true;     
                                isBreak = true;
                                break;
                            }
                            
                            switch(codeTmp)
                            {
                                case CLICKED_CHANGE_ID:
                                {
                                    memset(user_buf_draw_action_888 + FACE_REG_ID_MENU_Y*4*LCD_PIXEL_WIDTH, 0, 
                                                    21*4*LCD_PIXEL_WIDTH);// clear 21 lines
                                    // Change ID label
                                    if(label_id == 1)
                                    {
                                        label_id = 2;
                                        myCanvas.drawImage(my_img_id_02,    
                                            FACE_REG_ID_MENU_X ,FACE_REG_ID_MENU_Y);
                                    }
                                    else
                                    {
                                        label_id = 1;
                                        myCanvas.drawImage(my_img_id_01,
                                            FACE_REG_ID_MENU_X ,FACE_REG_ID_MENU_Y);
                                    }
                                    
                                    /* Clean cache */
                                    dcache_clean(user_buf_draw_action_888, 
                                            sizeof(user_buf_draw_action_888));
                                            
                                    clickedCode = CLICKED_UNKNOWN;
                                    break;
                                }
                                case CLICKED_ADD:
                                {
                                    
                                    Mat imgReg = gray(roi);
                                    resize(imgReg,imgReg,regSize);
                                    //train face
                                    face_database.database_image.push_back(imgReg);
                                    face_database.database_label.push_back(label_id);
                                    flag_train = true;
                                    isBreak = true;
                                    break;
                                }
                                case CLICKED_IGNORE:
                                {
                                    //ignore
                                    isBreak = true;
                                    break;
                                }
                                default:
                                break;
                            }
                            
                            if(isBreak)
                                break;
                            wait(0.1);
                        }
                    }// for faces.size()
                    
                    if( flag_train == true)
                    {
                        if(isFirstTrain == true)
                        {
                            isFirstTrain = false;
                            modelReg->train(face_database.database_image, face_database.database_label);
                        }
                        else
                        {
                            modelReg->update(face_database.database_image, face_database.database_label);
                        }
                    }
                    
                    // Clean screen
                    memset(user_buf_draw_action_888, 0, 
                                    sizeof(user_buf_draw_action_888));
                    oldStateFace = false;

                    mMutexClicked.lock();
                    clickedCode = CLICKED_UNKNOWN;
                    mMutexClicked.unlock();
                } //clickedCode == CLICKED_REGIS_FACE
                else
                {   
                    mMutexClicked.unlock();
                    if (isFirstTrain == false)
                    {
                        for(int i = 0; i < faces.size();i++)
                        {
                            int _x = faces[i].x*IMG_DOWN_SAMPLE;
                            int _y = faces[i].y*IMG_DOWN_SAMPLE;
                            int _w = faces[i].width*IMG_DOWN_SAMPLE;
                            int _h = faces[i].height*IMG_DOWN_SAMPLE;
                            
                            Rect roi(_x,_y,_w,_h);
                            Mat target_face = gray(roi);
                            resize(target_face,target_face,regSize);
                            int predicted = -1;
                            double confidence_level = 0;
                            modelReg->predict(target_face, predicted, confidence_level);
                            
                            obj_detect_result tmp;
                            tmp.obj = faces[i];
                 
                            if(predicted != -1)
                            {
                                if (confidence_level < thresmax)
                                {
                                    tmp.label = predicted;
                                }
                                else
                                {
                                    tmp.label = -1;
                                }
                            }
                            else
                            {
                                tmp.label = -1;
                            }
                            gObjectDetects.push_back(tmp);
                        }
                    }
                    else
                    {
                        for(int i = 0;i < faces.size();i++)
                        {
                            obj_detect_result tmp;
                            tmp.obj = faces[i];
                            tmp.label = -1;
                            gObjectDetects.push_back(tmp);
                        }
                    }
                    
                    gIsDrawing = true;
                    p_DrawObjects->signal_set(1);
                }//clickedCode != CLICKED_REGIS_FACE
            } //faces.size() > 0
            else
            {
                oldStateFace = false;
                memset(user_buf_draw_action_888, 0, 
                                    sizeof(user_buf_draw_action_888));
                mMutexClicked.lock();
                clickedCode = CLICKED_UNKNOWN;
                mMutexClicked.unlock();
                
                // Clean draw faces
                mMutexObjects.lock();
                if(gIsDrawing == false)
                {
                    mMutexObjects.unlock();
                    gObjectDetects.clear();
                    gIsDrawing = true;
                    p_DrawObjects->signal_set(1);
                }
                else
                {
                    mMutexObjects.unlock();
                }
            }
        }
        else
        {
            mMutexObjects.unlock();
        }
        
        /* Reset flag */
        faces.clear();
        mMutexProcess.lock();
        gIsProcessing = false;
        mMutexProcess.unlock();
        
        /* Clean cache */
        dcache_clean(user_buf_draw_action_888, 
                                    sizeof(user_buf_draw_action_888));
    }
}

/* 2 functions for gesture recognition */
int distanceP2P(Point2f a, Point2f b)
{
    int d = (int)sqrt(fabs(pow(a.x - b.x, 2) + pow(a.y - b.y, 2)));
    return d;
}

float getAngle(Point s, Point f, Point e){
    float l1 = distanceP2P(f, s);
    float l2 = distanceP2P(f, e);
    float dot = (s.x - f.x)*(e.x - f.x) + (s.y - f.y)*(e.y - f.y);
    float angle = acos(dot / (l1*l2));
    angle = angle * 180 / M_PI;
    return angle;
}

void drawButtonSampling(graphicFrameworkCanvas canvas,bool sampling)
{
    // Clear 21 lines
    memset(user_buf_draw_action_888 + (GESTURE_SAMPLING_BTN_Y - 1)*
        LCD_PIXEL_WIDTH*4, 0, 
        21*LCD_PIXEL_WIDTH*4);
    if(sampling == true) // Sampling, Clicking stop, draw sampling
    { 
        canvas.drawImage(my_img_sampling, GESTURE_SAMPLING_BTN_X, 
                            GESTURE_SAMPLING_BTN_Y);
    }
    else
    {
        canvas.drawImage(my_img_stop, GESTURE_SAMPLING_BTN_X, 
                            GESTURE_SAMPLING_BTN_Y);
    }
}

void gestureRegconition(void)
{
    // Init draw
    graphicFrameworkCanvas myCanvas(user_buf_draw_action_888, 0x01E0,
                    0x0110, 0x04, 
                    (uint8_t)DRAW_POINT,0x06);

    bool sampling = true;
    int kernelSize = 7;
    int thresValue = 100;
    int maxIndex = 0;
    double area = 0.0;
    unsigned long lArea = 0, maxArea = 0;
    int hullSize = 0;
    bool handSamp = true;
    bool isFinger = false;
    int oneFinger = 0;
    int countFinger = 0;
    Sampling handdist;
    Point2f mPa;
    ConvexPoint checkPoint;
    
    Mat imgGray;
    Mat blurImg;
    Mat thresholdImg;
    vector<vector<Point> > contours;
    vector<mLine> lLines;
    vector<mCircle> lCircles;
    vector<Point> lMaxContour;
    
    myCanvas.drawImage(my_img_stop, GESTURE_SAMPLING_BTN_X, 
                                    GESTURE_SAMPLING_BTN_Y);
    mMutexClicked.lock();
    clickedCode = CLICKED_UNKNOWN;
    mMutexClicked.unlock();
    clearGestureResult(gGestureResult);
    while(1)
    {
        semProcessThread.wait();
        
        if ( appMode != GUESTURE_RECOGNITION )
        {
            mMutexProcess.lock();
            gIsProcessing = false;
            mMutexProcess.unlock();
            memset(user_buf_draw_action_888, 0, 
                                    sizeof(user_buf_draw_action_888));   
            clearGestureResult(gGestureResult);  
            break;
        }
        
        //Draw button sampling
        mMutexClicked.lock();
        if (clickedCode == CLICKED_HAND_SAMPLING)
        {
            clickedCode = CLICKED_UNKNOWN;
            mMutexClicked.unlock();
            drawButtonSampling(myCanvas,sampling);
            sampling = !sampling;
        }
        else
        {
            mMutexClicked.unlock();
        }

        // Get frame camera
        {
            Mat res = _m_cvtRgb5652Rgb(my_frame,(int)LCD_PIXEL_WIDTH,
                                                (int)LCD_PIXEL_HEIGHT,2);
            cvtColor(res,imgGray,COLOR_RGB2GRAY);
            res.release();
        }
        
        //Clear old data
        lLines.clear();
        lCircles.clear();
        lMaxContour.clear();
        
        // Start processing
        blur(imgGray,blurImg,Size(kernelSize,kernelSize));
        threshold(blurImg,thresholdImg,thresValue,255,CV_THRESH_BINARY);

        checkPoint.Pointstart.clear();
        checkPoint.Pointdepth.clear();
        checkPoint.Pointindex.clear();
        checkPoint.detecthand = 0;
        
        findContours(thresholdImg,contours,CV_RETR_EXTERNAL,
                                            CV_CHAIN_APPROX_SIMPLE,Point(0,0));

        if(contours.size() > 0)
        {
            //togle_led(LED_GREEN);
            maxIndex = 0;
            vector<vector<int> > hull(contours.size());
            vector<vector<Vec4i> > convDef(contours.size());
            vector<vector<Point> > hull_point(contours.size());
            vector<vector<Point> > detectPoint(contours.size());
            
            for(int i = 0;i < contours.size();i++)
            {
                area = contourArea(contours[i]);
                lArea = long(area);
                if(lArea > maxArea)
                {
                    hullSize++;
                    maxArea = lArea;
                    maxIndex = i;
                }
            }
            
            if(maxArea > 50000)
            {
                //togle_reset(LED_RED,LED_BLUE);
                checkPoint.detecthand = 1;
                convexHull(Mat(contours[maxIndex]),hull[maxIndex],false);
                convexityDefects(contours[maxIndex],hull[maxIndex],convDef[maxIndex]);
            }
            
            for (int i = 0;i < convDef[maxIndex].size();i++)
            {
                if(convDef[maxIndex][i][3] > 20*256)
                {
                    int ind_0 = convDef[maxIndex][i][0];
                    int ind_1 = convDef[maxIndex][i][2];
                    checkPoint.Pointstart.push_back(contours[maxIndex][ind_0]);
                    checkPoint.Pointdepth.push_back(contours[maxIndex][ind_1]);
                    checkPoint.Pointindex.push_back(ind_1);
                }
            }

            vector<Moments> mu(contours.size());
            vector<Point2d> mc(contours.size());    
            mu[maxIndex] = moments(contours[maxIndex],false);
            mc[maxIndex] = Point2f(mu[maxIndex].m10 / mu[maxIndex].m00,
                                   mu[maxIndex].m01 / mu[maxIndex].m00);
            mPa = mc[maxIndex];

            // Reset values
            lArea = 0;
            maxArea = 0;
            hullSize = 0;
            area = 0;
            
            
            // Draw contours
            Mat imgContour(LCD_PIXEL_HEIGHT,LCD_PIXEL_WIDTH,CV_8UC1,Scalar(0));
            drawContours(imgContour,contours,maxIndex,Scalar(255));
            for(int i=0;i<imgContour.rows;i++)
            {
                for(int j=0;j<imgContour.cols;j++)
                {
                    if(imgContour.at<uchar>(i,j) == 255)
                    {
                        lMaxContour.push_back(Point(j,i));
                    }
                }
            }
            
            
        }// contours.size() > 0
        else
        {
            checkPoint.detecthand = 0;
        }
        
        if(sampling == true)
        {
            if (checkPoint.Pointstart.size() > 1)
            {
                for(int i = 0;i < checkPoint.Pointstart.size() -1;i++)
                {
                    float pAngle = getAngle(checkPoint.Pointstart[i],mPa,
                                                    checkPoint.Pointstart[i+1]);
                    float fAngle = getAngle(checkPoint.Pointstart[1],mPa,
                                                    checkPoint.Pointstart[i+1]);
    
                    if(pAngle < 90.00 && pAngle > 5.00 && fAngle < 180)
                    {
                        double pFdist = distanceP2P(mPa,
                                                    checkPoint.Pointstart[i]);
                        double pLdist = distanceP2P(mPa,
                                                    checkPoint.Pointdepth[i]);
                        double _Pmax = handdist.hand_max;
                        double _Pmin = handdist.hand_min;
                        if (pFdist > pLdist)
                        {
                            if (pFdist > _Pmax)
                            {
                                handdist.hand_max = pFdist;
                            }
                            if (pLdist < _Pmin)
                            {
                                handdist.hand_min = pLdist;
                            }
                        }
                        else
                        {
                            if (pLdist > _Pmin)
                            {
                                handdist.hand_max = pLdist;
                            }
                            if (pFdist < _Pmin)
                            {
                                handdist.hand_min = pFdist;
                            }
                        }
                    }
                }
            }
        }//sampling == true
        else
        {
            if(checkPoint.detecthand == 1)
            {
                if (checkPoint.Pointstart.size() > 1)
                {
                    for(int i = 0;i < checkPoint.Pointstart.size() -1;i++)
                    {
                        float pAngle = getAngle(checkPoint.Pointstart[i],mPa,
                                                    checkPoint.Pointstart[i+1]);
                        float fAngle = getAngle(checkPoint.Pointstart[1],mPa,
                                                    checkPoint.Pointstart[i+1]);

                        // Draw line
                        mLine aLine;
                        aLine.x1 = (int)checkPoint.Pointstart[i].x;
                        aLine.y1 = (int)checkPoint.Pointstart[i].y;
                        aLine.x2 = (int)mPa.x;
                        aLine.y2 = (int)mPa.y;
                        aLine.color[0] = 0x00;//Color red
                        aLine.color[1] = 0xFF;
                        lLines.push_back(aLine);
                        //line(showRes,checkPoint.Pointstart[i],mPa,
//                                                            Scalar(255,255,0));
                        //Draw circle
                        mCircle cir;
                        cir.x = (int)checkPoint.Pointstart[i].x;
                        cir.y = (int)checkPoint.Pointstart[i].y;
                        cir.radius = 10;
                        cir.color[0] = 0xF0;
                        cir.color[1] = 0xF0;//Color green
                        lCircles.push_back(cir);
                        
                        
                        //circle(showRes,checkPoint.Pointstart[i],10,
//                                                            Scalar(0,255,0));
    
                        if(pAngle < 90.00 && pAngle > 5.00 && fAngle < 180)
                        {
                            double pFdist = distanceP2P(mPa,
                                                    checkPoint.Pointstart[i]);
                            if(pFdist > (handdist.hand_min + 110) && 
                                pFdist < (handdist.hand_max + 40))
                            {
                                isFinger = true;
                                //line(showRes,checkPoint.Pointstart[i],mPa,Scalar(255,255,255));
                                aLine.x1 = (int)checkPoint.Pointstart[i].x;
                                aLine.y1 = (int)checkPoint.Pointstart[i].y;
                                aLine.x2 = (int)mPa.x;
                                aLine.y2 = (int)mPa.y;// Keep color red
                                lLines.push_back(aLine);
                                
                                oneFinger++;
                            }
                            else
                            {
                                isFinger = false;
                            }
    
                            if(isFinger == true)
                            {
                                //circle(showRes,mPa,handdist.hand_min,Scalar(255,0,0));
                                cir.x = (int)mPa.x;
                                cir.y = (int)mPa.y;
                                cir.radius = (int)handdist.hand_min;
                                lCircles.push_back(cir);//color green
                                
                                //circle(showRes,mPa,handdist.hand_min + 110,Scalar(255,0,0));
                                cir.radius = (int)handdist.hand_min + 110;
                                lCircles.push_back(cir);//color green
                                
                                double pLdist = distanceP2P(mPa,checkPoint.Pointdepth[i]);
                                if(pLdist > handdist.hand_min - 5)
                                {
                                    countFinger++;
                                    //circle(showRes,contours[maxIndex][checkPoint.Pointindex[i]],2,Scalar(0,255,255));
                                    cir.x = (int)contours[maxIndex][checkPoint.Pointindex[i]].x;
                                    cir.y = (int)contours[maxIndex][checkPoint.Pointindex[i]].y;
                                    cir.radius = 2;
                                    lCircles.push_back(cir);//color green
                                }
                            }
                        }
                    }// for
                    if(isFinger == false)
                    {
                        for(int i = 0; i < checkPoint.Pointstart.size() -1;i++)
                        {
                            double pLdist = distanceP2P(mPa,
                                                    checkPoint.Pointdepth[i]);
                            if (pLdist > (handdist.hand_min - 5))
                            {
                                countFinger = 0;
                                isFinger = true;
                            }
                        }
                    }
                }//checkPoint.Pointstart.size() > 1
            }//checkPoint.detecthand == 1
        }
        
        // Draw
        mMutexObjects.lock();
        if(gIsDrawing == false)
        {
            mMutexObjects.unlock();
            
            // Assign data
            gGestureResult.circles = lCircles;
            gGestureResult.lines = lLines;
            gGestureResult.contour = lMaxContour;
            
            gIsDrawing = true;
            p_DrawObjects->signal_set(1);
        }
        else
        {
            mMutexObjects.unlock();
        }
        
        mMutexProcess.lock();
        gIsProcessing = false;
        mMutexProcess.unlock();

        /* Clean cache */
        dcache_clean(user_buf_draw_action_888, 
                                    sizeof(user_buf_draw_action_888));
    }
}

/* Init a block on RAM to store file */
HeapBlockDevice myBd(2*128 * 512, 512);// ~128 Kb

/****** main image process here******/
int main(void) {

    /* Format block on RAM */
    if(FATFileSystem::format(&myBd))
    {
        togle_led(LED_RED);
    }
    else
    {
        togle_led(LED_GREEN);
    }
    FATFileSystem myFs(STORAGE_NAME,&myBd); /* Start the block*/
    
    /* Create file .xml*/
    if(arr2File(haar_cascafe_frontal_face,HAAR_FILE_SIZE,HAAR_CASCADE_FACE_PATH))
    {
        togle_led(LED_BLUE);
    }
    else
    {
        togle_led(LED_RED);
    }
    
    /* Init SD card*/
    /*myStorage = new cStorage(SDCARD_NAME);
    if( initSdCard() == false)
    {
        while(1);
    }*/

    /* Initialization of LCD */
    Init_LCD_Display();    /* When using LCD, please call before than Init_Video(). */

    /* Initialization of Video */
    Init_Video();
    
    /* Start Video and Lcd processing */
    p_VideoLcdTask = new Thread();
    p_VideoLcdTask->start(video_lcd_task);
    
    p_Touch = new Thread();
    p_Touch->start(touch_task);
    
    /* Start image processor*/
    p_DrawObjects = new Thread();
    p_DrawObjects->signal_set(0);
    p_DrawObjects->start(img_draw_objects);
    
    initDrawAction32();

    while(1)
    {
        switch(appMode)
        {
            case FACE_DETECTION:
            {
                faceDectectionApp();
                break;
            }
            case MOTION_DETECTION:
            {
                motionDetectionApp();
                break;
            }
            case FACE_RECOGNITION:
            {
                faceRecognitionApp();
                break;
            }
            case GUESTURE_RECOGNITION:
            {
                gestureRegconition();
                break;
            }
            default:
            {
                wait(0.5);
                break;
            }
        }
    }
    return 1;
}
