#include "cProcess.hpp"
#include "mStorage.hpp"

Mat _m_cvtYcrcb2Rgb(uint8_t *_src, int _weight, int _heigh, int _bytes_per_pixel)
{
    Mat _dst(_heigh, _weight, CV_8UC3);
    int m_stride = _bytes_per_pixel * _weight;
    int Y,U,V;
    int r,g,b;
    for(int ih = 0; ih < _heigh; ih++)
    {
        for(int iw = 0; iw < _weight; iw++)
        {
            if(iw % 2 == 0)
            {
                Y = _src[ih*m_stride + iw*2 + 0];
                U = _src[ih*m_stride + iw*2 + 1];
                V = _src[ih*m_stride + iw*2 + 3];
            }
            else
            {
                Y = _src[ih*m_stride + iw*2 + 0];
                U = _src[ih*m_stride + iw*2 -1];
                V = _src[ih*m_stride + iw*2 + 1];
            }
            
            r = Y + 1.4075*(V-128);
            g = Y - 0.3455*(U-128) - 0.7169*(V-128);
            b = Y + 1.7790*(U-128);
            
            if (r < 0)
                r = 0;
            else if (r > 255)
                r = 255;
            
            if (g < 0)
                g = 0;
            else if (g > 255)
                g = 255;
            
            if (b < 0)
                b = 0;
            else if (b > 255)
                b = 255;
            
            _dst.at<Vec3b>(ih, iw)[0] = r;
            _dst.at<Vec3b>(ih, iw)[1] = g;
            _dst.at<Vec3b>(ih, iw)[2] = b;
        }
    }
    return _dst;
}

Mat _m_cvtYcrcb2Gray(uint8_t *_src, int _weight, int _heigh, int _bytes_per_pixel)
{
    Mat _dst(_heigh, _weight, CV_8UC1);

    int rows = 0;
    int cols = 0;
    for(int i = 0; i < (((_weight*_heigh*_bytes_per_pixel)+ 31u) & ~31u); i+=_bytes_per_pixel)
    {
        if((i-1)/2 > (rows+1)*_weight - 1)
        {
            rows++;
            cols = 0;
        }
        _dst.at<uchar>(rows,cols++) = _src[i];
    }
    return _dst;
}

Mat _m_cvtRgb5652Rgb(uint8_t *_src, int _w, int _h, int _bytes_per_pixel)
{
    Mat _res(_h,_w,CV_8UC3,Scalar(0,0,0));
    const uint16_t lc_RedMask = 0xF800;
    const uint16_t lc_GreenMask = 0x07E0;
    const uint16_t lc_BlueMask =  0x001F;
    int idx = 0;
    for(int i = 0; i < _w*_h*_bytes_per_pixel; i+=2, idx++)
    {
        uint16_t lo_PixelVal = (_src[ i ]) | (_src[ i + 1 ] << 8);
        uint8_t lo_RedVal = (lo_PixelVal & lc_RedMask) >> 11;
        uint8_t lo_GreenVal = (lo_PixelVal & lc_GreenMask) >> 5;
        uint8_t lo_BlueVal = (lo_PixelVal & lc_BlueMask);

        lo_RedVal <<= 3;
        lo_GreenVal <<= 2;
        lo_BlueVal <<= 3;
        _res.at<Vec3b>(idx)[0] = lo_BlueVal;
        _res.at<Vec3b>(idx)[1] = lo_GreenVal;
        _res.at<Vec3b>(idx)[2] = lo_RedVal;
    }
    return _res;
}

void _m_cvtGray2Rgb565(Mat _gray,uint8_t *_src, int _weight, int _heigh, int _bytes_per_pixel)
{
    for(int i = 0;i < _weight*_heigh*_bytes_per_pixel;i += 2)
    {
        uint8_t grayValue = _gray.at<uchar>(((i/2)/_weight)%_heigh,(i/2)%_weight);
        _src[i+1] = ((grayValue>>3)<<3)|(grayValue>>5);
        _src[i] = ((grayValue>>2)<<5)|(grayValue>>3);
    }
}

void writeMatToTxt(Mat _mat, char* file_name)
{
    int channels;
    FILE *myFileW = fopen(file_name,"w");
    if(myFileW != NULL){
        togle_led(LED_BLUE);
        channels = _mat.channels();
        if(channels == 1)
        {
            togle_reset(LED_RED,LED_GREEN);
        }
        for(int i=0; i<_mat.rows; i++)
        {
            for(int j=0; j<_mat.cols; j++)
            {
                int r,g,b;
                char _buf[15];
                if(channels == 1)
                {
                    r = _mat.at<uchar>(i,j);
                    sprintf(_buf,"%d,",r);
                    fprintf(myFileW, "%s", _buf);
                }
                else{
                    r = _mat.at<Vec3b>(i, j)[0];
                    g = _mat.at<Vec3b>(i, j)[1];
                    b = _mat.at<Vec3b>(i, j)[2];
                    sprintf(_buf,"%d,%d,%d,",r,g,b);
                    fprintf(myFileW, "%s", _buf);
                }
            }
        }
        fclose(myFileW);
    }
    else{
        togle_led(LED_RED);
    }
}

void togle_led(DigitalOut led)
{
    led = 1;
    wait(0.1);
    led = 0;
    wait(0.1);
}

void togle_reset(DigitalOut led1, DigitalOut led2)
{
    led1 = 1;
    led2 = 1;
    wait(0.1);
    led1 = 0;
    led2 = 0;
    wait(0.1);
}

uint8_t* cvtMat2RGBA444(int w, int h, Mat src) // w<=40; h <=40
{
    uint8_t res[100*100*2 + 6];
    
    memset(res,0,sizeof(res));
    /*6 bytes header*/
    res[0] = w & 0x00FF;
    res[2] = h & 0x00FF;
    res[4] = 0x02;
    
    int counter = 6;
    for(int i = 0; i < h;i++)
    {
        for(int j = 0; j < w; j++)
        {
            int b = src.at<Vec3b>(i,j)[0];
            int g = src.at<Vec3b>(i,j)[1];
            int r = src.at<Vec3b>(i,j)[2];
            
            res[counter++] = (g & 0xF0) | (b >> 4);//Green, Blue
            res[counter++] = 0xF0 | (r>>4);//Alph, Red
        }
    }
    return res;
}

uint8_t* cvtMat2RGBA888(int w, int h, Mat src)
{
    uint8_t res[100*100*4 + 6];
    
    memset(res,0,sizeof(res));
    /*6 bytes header*/
    res[0] = w & 0x00FF;
    res[2] = h & 0x00FF;
    res[4] = 0x04;
    
    int counter = 6;
    for(int i = 0; i < h;i++)
    {
        for(int j = 0; j < w; j++)
        {
            int b = src.at<Vec3b>(i,j)[0];
            int g = src.at<Vec3b>(i,j)[1];
            int r = src.at<Vec3b>(i,j)[2];
            
            res[counter++] = r; // Red
            res[counter++] = 0xFF; // Alpha
            res[counter++] = b;// Blue
            res[counter++] = g;// Green
        }
    }
    return res;
}

bool arr2File(uint8_t* arr, int size, char* path)
{
    FILE *fp = fopen(path,"w");
    if (fp != NULL)
    {
        for(unsigned int i = 0; i < size; i++)
        {
            fprintf(fp,"%c",arr[i]);
        }
        fclose(fp);
        return true;
    }
    return false;
}

