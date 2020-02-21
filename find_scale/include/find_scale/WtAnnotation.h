#ifndef WTANNOTATION_H
#define WTANNOTATION_H


#include <opencv2/opencv.hpp>

namespace WtAnnotation
{
bool LoadRoi(const std::string &sRoiFile, cv::Rect &Roi);
void Load(const std::string &path, cv::Mat &image);
void Load(const std::string &path, std::vector<std::string> &names, std::vector<cv::Rect> &locations);
void Load(const std::string &path, cv::Mat &image, std::vector<std::string> &names, std::vector<cv::Rect> &locations);
}

#endif // WTANNOTATION_H
