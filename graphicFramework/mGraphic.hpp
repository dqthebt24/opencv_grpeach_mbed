#ifndef M_GRAPHIC_HPP
#define M_GRAPHIC_HPP

#include "mbed.h"

#ifdef  __cplusplus
extern "C" {  /* Start of C Symbol */ 
#endif

class graphicFrameworkCanvas
{
private:
    uint16_t mWidth;
    uint16_t mHeight;
    uint8_t mBpp;
    uint8_t mDrawPoint; // Point per pixel.
    uint8_t mSizeHeaderImg; // Bytes
    uint8_t * mBufLcd;
    
    void drawLineNormal(int x1, int y1, int x2, int y2, uint8_t* color);
    void drawBresenham1(int x1, int y1, int x2, int y2, uint8_t* color);
    void drawBresenham2(int x1, int y1, int x2, int y2, uint8_t* color);
    void drawBresenham3(int x1, int y1, int x2, int y2, uint8_t* color);
    void drawBresenham4(int x1, int y1, int x2, int y2, uint8_t* color);
    
public:
    graphicFrameworkCanvas(uint8_t * bufDraw,uint16_t w,uint16_t h, int8_t bpp, 
                                    int8_t drawPoint, int8_t sizeHeader);
    ~graphicFrameworkCanvas();
    void draw_pixel(int x, int y,uint8_t *color);
    void draw_pixel(uint8_t * p_buf, int id, int x, int y,uint8_t *color);
    void drawImage(uint8_t * _img, int _x, int _y);
    void drawRect(int x, int y, int width, int height, uint8_t *color);
    void drawLine(int x1, int y1, int x2, int y2, uint8_t* color);
    void drawCircle(int x0,int y0, int radius, uint8_t* color);
}; // End class graphicFrameworkCanvas

#ifdef  __cplusplus
 }  /* End of C Symbol */ 
#endif

#endif //M_GRAPHIC_HPP