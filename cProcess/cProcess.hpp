#ifndef C_PROCESS_HPP
#define C_PROCESS_HPP

#include "mbed.h"
#include "cProcess/cProcess.hpp"
#include "opencv_3_1/opencv2/core.hpp"
#include "opencv_3_1/opencv2/imgproc.hpp"
using namespace cv;

Mat _m_cvtYcrcb2Rgb(uint8_t *_src, int _weight, int _heigh, int _bytes_per_pixel);
Mat _m_cvtYcrcb2Gray(uint8_t *_src, int _weight, int _heigh, int _bytes_per_pixel);
Mat _m_cvtRgb5652Rgb(uint8_t *_src, int _weight, int _heigh, int _bytes_per_pixel);
void _m_cvtGray2Rgb565(Mat _gray,uint8_t *_src, int _weight, int _heigh, int _bytes_per_pixel);
uint8_t* cvtMat2RGBA444(int w, int h, Mat src);
uint8_t* cvtMat2RGBA888(int w, int h, Mat src);
bool arr2File(uint8_t* arr, int size, char* path);
void writeMatToTxt(Mat a, char* file_name);
void togle_led(DigitalOut led);
void togle_reset(DigitalOut led1, DigitalOut led2);
#endif //C_PROCESS_HPP