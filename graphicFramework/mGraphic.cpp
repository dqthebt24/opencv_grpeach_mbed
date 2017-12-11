#include "mGraphic.hpp"
int counter =0;
static void togle_led(DigitalOut led)
{
    led = 1;
    wait(0.1);
    led = 0;
    wait(0.1);
}

graphicFrameworkCanvas::graphicFrameworkCanvas(uint8_t * bufDraw, uint16_t w,uint16_t h, 
                        int8_t bpp, int8_t drawPoint, int8_t sizeHeader)
{
    mBufLcd = bufDraw;
    mWidth = w;
    mHeight = h;
    mBpp = bpp;
    mDrawPoint = drawPoint;
    mSizeHeaderImg = sizeHeader;
}

graphicFrameworkCanvas::~graphicFrameworkCanvas()
{
}

void graphicFrameworkCanvas::draw_pixel(uint8_t * p_buf, int id, 
                                                int x, int y,uint8_t *color)
{   
    int idx_base;
    int wk_idx;
    int i;
    int j;
    
    /* A coordinate in the upper left is calculated from a central coordinate. */
    if ((x - (mDrawPoint / 2)) >= 0) {
        x -= (mDrawPoint / 2);
    }
    if (x > (mWidth - mDrawPoint)) {
        x = (mWidth - mDrawPoint);
    }
    if ((y - (mDrawPoint / 2)) >= 0) {
        y -= (mDrawPoint / 2);
    }
    if (y > (mHeight - mDrawPoint)) {
        y = (mHeight - mDrawPoint);
    }
    idx_base = (x + (mWidth * y)) * mBpp;
    
    /* Drawing */
    for (i = 0; i < mDrawPoint; i++) {
        wk_idx = idx_base + (mWidth * mBpp * i);
        for (j = 0; j < mDrawPoint; j++) {
            for(int k = 0; k < mBpp;k++)
            {
                p_buf[wk_idx++] = color[k];
            }
        }
    }
}

void graphicFrameworkCanvas::drawImage(uint8_t * _img, int _x, int _y)
{   
    uint8_t *p_buf = _img;
    uint8_t coller_pix[mBpp];
    uint16_t w = (_img[1] << 8) | _img[0];
    uint16_t h = (_img[3] << 8) | _img[2];
    uint8_t bpp = _img[4];

    p_buf+=mSizeHeaderImg;

    for(int i = 0;i < (int)h*w*bpp; i+=bpp)
    {
        int pos_x = (i/bpp)%w;
        int pos_y = (i/bpp)/w;
        for(int k = 0; k < mBpp;k++)
        {
            coller_pix[k] = p_buf[i+k];
        }
        draw_pixel(mBufLcd, 0, pos_x + _x, pos_y + _y,coller_pix);
    }
}

void graphicFrameworkCanvas::drawRect(int x, int y, int width, int height, 
                                                            uint8_t *color)
{
    for(int k = 0; k < width;k++)
    {
        draw_pixel(mBufLcd, 0, x + k, y, color);
        draw_pixel(mBufLcd, 0, x + k, y + height,color);
    }
    
    for(int k = 0; k < height;k++)
    {
        draw_pixel(mBufLcd, 0, x, y + k, color);
        draw_pixel(mBufLcd, 0, x + width, y + k,color);
    }
}

void graphicFrameworkCanvas::draw_pixel(int x, int y,uint8_t *color)
{
    int idx_base;
    int wk_idx;
    int i;
    int j;
    
    /* A coordinate in the upper left is calculated from a central coordinate. */
    if ((x - (mDrawPoint / 2)) >= 0) {
        x -= (mDrawPoint / 2);
    }
    if (x > (mWidth - mDrawPoint)) {
        x = (mWidth - mDrawPoint);
    }
    if ((y - (mDrawPoint / 2)) >= 0) {
        y -= (mDrawPoint / 2);
    }
    if (y > (mHeight - mDrawPoint)) {
        y = (mHeight - mDrawPoint);
    }
    idx_base = (x + (mWidth * y)) * mBpp;
    
    /* Drawing */
    for (i = 0; i < mDrawPoint; i++) {
        wk_idx = idx_base + (mWidth * mBpp * i);
        for (j = 0; j < mDrawPoint; j++) {
            for(int k = 0; k < mBpp;k++)
            {
                mBufLcd[wk_idx++] = color[k];
            }
        }
    }
    return;
}
void graphicFrameworkCanvas::drawLineNormal(int x1, int y1, int x2, int y2, uint8_t* color)
{
    int tmp_x = 0;
    int tmp_y = 0;
    int i = 0;

    if (x1 == x2) // Case equation x = m.
    {
        if (y1 > y2) // Swap 2 points
        {
            //ptTemp = pt1;
            tmp_x = x1;
            tmp_y = y1;
            
            //pt1 = pt2;
            x1 = x2;
            y1 = y2;
            
            //pt2 = ptTemp;
            x2 = tmp_x;
            y2 = tmp_y;
        }

        for (i = y1; i <= y2; i++) // Draw points
        {
            draw_pixel(x1,i,color);
        }
        return;
    }

    if (y1 == y2) // Case equation y = m.
    {
        if (x1 > x2) // Swap 2 points
        {
            //ptTemp = pt1;
            tmp_x = x1;
            tmp_y = y1;
            
            //pt1 = pt2;
            x1 = x2;
            y1 = y2;
            
            //pt2 = ptTemp;
            x2 = tmp_x;
            y2 = tmp_y;
        }

        for (i = x1; i <= x2; i++) // Draw points
        {
            draw_pixel(i,y1,color);
        }
        return;
    }

    // Case equation y = ax + b
    float fA = (((float)(y2 - y1))) / (x2 - x1); // Calculate a.
    float fB = y1 - fA * x1; // Calculate b.

    if (x1 > x2) // Swap 2 points
    {
        //ptTemp = pt1;
        tmp_x = x1;
        tmp_y = y1;
        
        //pt1 = pt2;
        x1 = x2;
        y1 = y2;
        
        //pt2 = ptTemp;
        x2 = tmp_x;
        y2 = tmp_y;
    }

    // Calculate every points based on the equation
    for (i = x1; i <= x2; i++)
    {
        draw_pixel(i,(int)floor(fA * i + fB),color);
    }
}

void graphicFrameworkCanvas::drawBresenham1(int x1, int y1, int x2, int y2, 
                                                                uint8_t* color)
{
    int iDx = abs(x2 - x1);
    int iDy = abs(y2 - y1);
    int iTwoDy = 2 * iDy;
    int iTwoDyMinusDx = 2 * (iDy - iDx);
    int iP = iTwoDy - iDx;
    int iX, iY, iXEnd;

    // Get smaller x to base
    if (x1 > x2)
    {
        iX = x2;
        iY = y2;
        iXEnd = x1;
    }
    else
    {
        iX = x1;
        iY = y1;
        iXEnd = x2;
    }

    // Draw first point
    draw_pixel(iX,iY,color);

    while (iX < iXEnd)
    {
        iX++;
        if (iP < 0)
            iP += iTwoDy;
        else
        {
            iY++;
            iP += iTwoDyMinusDx;
        }
        
        // Draw points
        draw_pixel(iX,iY,color);
    }
}

void graphicFrameworkCanvas::drawBresenham2(int x1, int y1, int x2, int y2, uint8_t* color)
{
    int iDx = abs(x2 - x1);
    int iDy = abs(y2 - y1);
    int iTwoDy = 2 * iDy;
    int iTwoDyMinusDx = 2 * (iDy - iDx);
    int iP = -iTwoDy - iDx;
    int iX, iY, iXEnd;

    // Get smaller x to base
    if (x1 > x2)
    {
        iX = x2;
        iY = y2;
        iXEnd = x1;
    }
    else
    {
        iX = x1;
        iY = y1;
        iXEnd = x2;
    }

    // Draw first point
    draw_pixel(iX,iY,color);

    while (iX < iXEnd)
    {
        iX++;
        if (iP < 0)
            iP += iTwoDy;
        else
        {
            iY--;
            iP += iTwoDyMinusDx;
        }
        
        // Draw points
        draw_pixel(iX,iY,color);
    }
}

void graphicFrameworkCanvas::drawBresenham3(int x1, int y1, int x2, int y2, 
                                                                uint8_t* color)
{
    int iDx = abs(x2 - x1);
    int iDy = abs(y2 - y1);
    int iTwoDx = 2 * iDx;
    int iTwoDxMinusDy = 2 * (iDx - iDy);
    int iP = iTwoDx - iDy;
    int iX, iY, iYEnd;

    // Get smaller x to base
    if (y1 > y2)
    {
        iX = x2;
        iY = y2;
        iYEnd = y1;
    }
    else
    {
        iX = x1;
        iY = y1;
        iYEnd = y2;
    }

    // Draw first point
    draw_pixel(iX,iY,color);

    while (iY < iYEnd)
    {
        iY++;
        if (iP < 0)
            iP += iTwoDx;
        else
        {
            iX++;
            iP += iTwoDxMinusDy;
        }
        // Draw points
        draw_pixel(iX,iY,color);
    }
}

void graphicFrameworkCanvas::drawBresenham4(int x1, int y1, int x2, int y2, 
                                                                uint8_t* color)
{
    int iDx = abs(x2 - x1);
    int iDy = abs(y2 - y1);
    int iTwoDx = 2 * iDx;
    int iTwoDxMinusDy = 2 * (iDx - iDy);
    int iP = -iTwoDx - iDy;
    int iX, iY, iYEnd;

    // Get smaller x to base
    if (y1 > y2)
    {
        iX = x1;
        iY = y1;
        iYEnd = y2;
    }
    else
    {
        iX = x2;
        iY = y2;
        iYEnd = y1;
    }

    // Draw first point
    draw_pixel(iX,iY,color);

    while (iY > iYEnd)
    {
        iY--;
        if (iP < 0)
            iP += iTwoDx;
        else
        {
            iX++;
            iP += iTwoDxMinusDy;
        }

        // Draw points
        draw_pixel(iX,iY,color);
    }
}

void graphicFrameworkCanvas::drawLine(int x1, int y1, int x2, int y2, 
                                                                uint8_t* color)
{
    if (x1 == x2 || y1 == y2) // Case equation x = m or y = m.
    {
        drawLineNormal(x1,y1,x2,y2,color);
        return;
    }

    // Case equation y = ax + b
    float fA = ((float)(y2 - y1)) / (x2 - x1); // Calculate a.

    // Case (0 < m < 1).
    if (fA > 0 && fA < 1)
    {
        drawBresenham1(x1,y1,x2,y2,color);
        return;
    }

    // Case (-1 < m < 0).
    if (fA > -1 && fA < 0)
    {
        drawBresenham2(x1,y1,x2,y2,color);
        return;
    }

    // Case (m >= 1).
    if (fA >= 1)
    {
        drawBresenham3(x1,y1,x2,y2,color);
        return;
    }

    // Case (m =< -1).
    if (fA <= -1)
    {
        drawBresenham4(x1,y1,x2,y2,color);
        return;
    }
}

void graphicFrameworkCanvas::drawCircle(int x0,int y0, int radius, uint8_t* color)
{
    int x = radius;
    int y = 0;
    int err = 0;

    while (x >= y)
    {
        // Draw points
        draw_pixel(x0 + x,y0 + y,color);
        
        draw_pixel(x0 + y, y0 + x,color);
        draw_pixel(x0 - y, y0 + x,color);
        draw_pixel(x0 - x, y0 + y,color);
        draw_pixel(x0 - x, y0 - y,color);
        draw_pixel(x0 - y, y0 - x,color);
        draw_pixel(x0 + y, y0 - x,color);
        draw_pixel(x0 + x, y0 - y,color);

        y += 1;
        if (err <= 0)
        {
            err += 2*y + 1;
        } else 
        {
            x -= 1;
            err += 2 * (y - x) + 1;
        }
    }
}