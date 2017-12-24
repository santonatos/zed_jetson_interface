#ifndef OPENCV_INTER_HPP_
#define OPENCV_INTER_HPP_

#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <sl/Core.hpp>
#include <sl/defines.hpp>

using namespace sl;

typedef struct mouseOCVStruct {
    Mat depth;
    cv::Size _resize;
} mouseOCV;


static void onMouseCallback(int32_t event, int32_t x, int32_t y, int32_t flag, void * param);
cv::Mat slMat2cvMat(sl::Mat& input);
int start_opencv();
void show_image(sl::Camera zed);
#endif /*OPENCV_INTER_HPP*/
