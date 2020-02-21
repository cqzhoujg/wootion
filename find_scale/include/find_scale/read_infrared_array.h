#ifndef READ_THERMAL_ARRAY_H
#define READ_THERMAL_ARRAY_H

#include <opencv2/opencv.hpp>

namespace read_infrared_array
{

cv::Mat read_as_raw(std::string filename);


cv::Mat read_as_gray(std::string filename, bool equalize_hist=true);


cv::Mat read_as_pcolor(std::string filename);
}

#endif // READ_THERMAL_ARRAY_H
