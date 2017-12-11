#ifndef DEFINE_H
#define DEFINE_H

#define VIDEO_CVBS             (0)                 /* Analog  Video Signal */
#define VIDEO_CMOS_CAMERA      (1)                 /* Digital Video Signal */
#define VIDEO_YCBCR422         (0)
#define VIDEO_RGB888           (1)
#define VIDEO_RGB565           (2)

/**** User Selection *********/
/** Camera setting **/
#define VIDEO_INPUT_METHOD     (VIDEO_CMOS_CAMERA) /* Select  VIDEO_CVBS or VIDEO_CMOS_CAMERA                       */
#define VIDEO_INPUT_FORMAT     (VIDEO_RGB565 )    /* Select  VIDEO_YCBCR422 or VIDEO_RGB888 or VIDEO_RGB565        */
#define USE_VIDEO_CH           (0)                 /* Select  0 or 1            If selecting VIDEO_CMOS_CAMERA, should be 0.)               */
#define VIDEO_PAL              (0)                 /* Select  0(NTSC) or 1(PAL) If selecting VIDEO_CVBS, this parameter is not referenced.) */
/** LCD setting **/
#define LCD_TYPE               (0)                 /* Select  0(4.3inch) or 1(7.1inch) */
/*****************************/

/** LCD shield config **/
#if (LCD_TYPE == 0)
  #include "LCD_shield_config_4_3inch.h"
#else
  #include "LCD_shield_config_7_1inch.h"
#endif

/** Video and Grapics (GRAPHICS_LAYER_0) parameter **/
/* video input */
#if USE_VIDEO_CH == (0)
  #define VIDEO_INPUT_CH       (DisplayBase::VIDEO_INPUT_CHANNEL_0)
  #define VIDEO_INT_TYPE       (DisplayBase::INT_TYPE_S0_VFIELD)
#else
  #define VIDEO_INPUT_CH       (DisplayBase::VIDEO_INPUT_CHANNEL_1)
  #define VIDEO_INT_TYPE       (DisplayBase::INT_TYPE_S1_VFIELD)
#endif

/* NTSC or PAL */
#if VIDEO_PAL == 0
  #define COL_SYS              (DisplayBase::COL_SYS_NTSC_358)
#else
  #define COL_SYS              (DisplayBase::COL_SYS_PAL_443)
#endif

/* Video input and LCD layer 0 output */
#if VIDEO_INPUT_FORMAT == VIDEO_YCBCR422
  #define VIDEO_FORMAT         (DisplayBase::VIDEO_FORMAT_YCBCR422)
  #define GRAPHICS_FORMAT      (DisplayBase::GRAPHICS_FORMAT_YCBCR422)
  #define WR_RD_WRSWA          (DisplayBase::WR_RD_WRSWA_NON)
#elif VIDEO_INPUT_FORMAT == VIDEO_RGB565
  #define VIDEO_FORMAT         (DisplayBase::VIDEO_FORMAT_RGB565)
  #define GRAPHICS_FORMAT      (DisplayBase::GRAPHICS_FORMAT_RGB565)
  #define WR_RD_WRSWA          (DisplayBase::WR_RD_WRSWA_32_16BIT)
#else
  #define VIDEO_FORMAT         (DisplayBase::VIDEO_FORMAT_RGB888)
  #define GRAPHICS_FORMAT      (DisplayBase::GRAPHICS_FORMAT_RGB888)
  #define WR_RD_WRSWA          (DisplayBase::WR_RD_WRSWA_32BIT)
#endif

/* The size of the video input is adjusted to the LCD size. */
#define VIDEO_PIXEL_HW                LCD_PIXEL_WIDTH
#define VIDEO_PIXEL_VW                LCD_PIXEL_HEIGHT

/*! Frame buffer stride: Frame buffer stride should be set to a multiple of 32 or 128
    in accordance with the frame buffer burst transfer mode. */
/* FRAME BUFFER Parameter GRAPHICS_LAYER_0 */
#define FRAME_BUFFER_NUM              (3u)
#if ( VIDEO_INPUT_FORMAT == VIDEO_YCBCR422 || VIDEO_INPUT_FORMAT == VIDEO_RGB565 )
  #define FRAME_BUFFER_BYTE_PER_PIXEL (2u)
#else
  #define FRAME_BUFFER_BYTE_PER_PIXEL (4u)
#endif
#define FRAME_BUFFER_STRIDE           (((LCD_PIXEL_WIDTH * FRAME_BUFFER_BYTE_PER_PIXEL) + 31u) & ~31u)

/* DRAW BUFFER Parameter GRAPHICS_LAYER_1 */
#define DRAW_BUFFER_BYTE_PER_PIXEL   (2u)
#define DRAW_BUFFER_STRIDE           (((LCD_PIXEL_WIDTH * DRAW_BUFFER_BYTE_PER_PIXEL) + 31u) & ~31u)

//thedo__
#define DRAW_BUFFER_STRIDE_LAYER_3           (((LCD_PIXEL_WIDTH * 4) + 31u) & ~31u)

/* Draw panel parameter */
#define DRAW_POINT                    (1u)

/* IMAGE DOWN SAMPLE*/
#define IMG_DOWN_SAMPLE                (2u)

/* Touch number point */
#define TOUCH_NUM                      (1u)

#define SDCARD_NAME "SD"
#define SDCARD_PATH "/SD"

#define STORAGE_NAME                    "fs"
#define HAAR_CASCADE_FACE_PATH          "/fs/lbpcascade_frontalface.xml"

#define MODE_BTN_X                      16
#define MODE_BTN_Y                      235

#define REGIS_FACE_BTN_X                5
#define REGIS_FACE_BTN_Y                130

#define FACE_REG_ID_MENU_X              100
#define FACE_REG_ID_MENU_Y              50

#define FACE_REG_ACT_MENU_X             410
#define FACE_REG_ACT_MENU_Y             80

#define GESTURE_SAMPLING_BTN_X          410
#define GESTURE_SAMPLING_BTN_Y          110

#define M_PI                            (3.14159)
#define HAAR_FILE_SIZE                  (51856)//(0xCA90)


enum APP_MODE{
    FACE_DETECTION = 0,
    MOTION_DETECTION,
    FACE_RECOGNITION,
    GUESTURE_RECOGNITION,
    MODE_UNKNOWN
};

enum CLICKED_CODE
{
    CLICKED_UNKNOWN = 0,
    CLICKED_REGIS_FACE,
    CLICKED_CHANGE_ID,
    CLICKED_ADD,
    CLICKED_IGNORE,
    CLICKED_HAND_SAMPLING
};

#define MAX_COUNTOURS (500)

#if defined(__ICCARM__)
/* 32 bytes aligned */
#pragma data_alignment=32
static uint8_t user_frame_buffer0[FRAME_BUFFER_STRIDE * LCD_PIXEL_HEIGHT];
#pragma data_alignment=32
static uint8_t user_frame_buffer1[FRAME_BUFFER_STRIDE * LCD_PIXEL_HEIGHT];
#pragma data_alignment=32
static uint8_t user_frame_buffer2[FRAME_BUFFER_STRIDE * LCD_PIXEL_HEIGHT];
#else
/* 32 bytes aligned */
static uint8_t user_frame_buffer0[FRAME_BUFFER_STRIDE * LCD_PIXEL_HEIGHT]__attribute((aligned(32)));
static uint8_t user_frame_buffer1[FRAME_BUFFER_STRIDE * LCD_PIXEL_HEIGHT]__attribute((aligned(32)));
static uint8_t user_frame_buffer2[FRAME_BUFFER_STRIDE * LCD_PIXEL_HEIGHT]__attribute((aligned(32)));
static uint8_t user_frame_buffer_draw[DRAW_BUFFER_STRIDE * LCD_PIXEL_HEIGHT]__attribute((aligned(32)));
static uint8_t user_frame_buffer_draw_button[DRAW_BUFFER_STRIDE * LCD_PIXEL_HEIGHT]__attribute((aligned(32)));
static uint8_t user_buf_draw_action_888[DRAW_BUFFER_STRIDE_LAYER_3 * LCD_PIXEL_HEIGHT]__attribute((aligned(32)));
static uint8_t my_frame[FRAME_BUFFER_STRIDE * LCD_PIXEL_HEIGHT]__attribute((aligned(32)));
#endif
static uint8_t * FrameBufferTbl[FRAME_BUFFER_NUM] = {user_frame_buffer0, user_frame_buffer1, user_frame_buffer2};
#if VIDEO_INPUT_METHOD == VIDEO_CVBS
static volatile int32_t vfield_count = 0;
#endif

#endif //DEFINE_H