#ifndef __CALC_SHIFT_H__
#define __CALC_SHIFT_H__

#define __cplusplus 201103L

#include <opencv2/opencv.hpp>
#include <string>

// 计算img_run相对于img_preset偏移多少个像素，return confidence in [0.0 ~ 1.0]
double calc_shift(const cv::Mat &img_preset, const cv::Mat &img_run, const cv::Rect &roi_preset, cv::Point2f &shift);

#endif /* __CALC_SHIFT_H__ */
