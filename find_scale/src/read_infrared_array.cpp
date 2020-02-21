#include "read_infrared_array.h"

#define DEBUG(format, ...) printf("<debug> %s line=%04d func=%s: ", basename(__FILE__), __LINE__,__FUNCTION__),printf(format"\n", ##__VA_ARGS__)


static cv::Mat float2ubyte(const cv::Mat &x)
{
    cv::Mat y;
    double minVal = 0, maxVal = 0;
    cv::minMaxIdx(x, &minVal, &maxVal);
    double deltaVal = std::max(maxVal-minVal,0.0001);
    x.convertTo(y, CV_8UC1, 255.0/deltaVal, -255*minVal/deltaVal);
    return y;
}


static cv::Mat equalize_float(const cv::Mat &x)
{
    cv::Mat x_sort = x.clone();
    const int num_pixels = x.rows * x.cols;
    float *x_arr = (float *)(x_sort.data);
    std::sort(x_arr, x_arr + num_pixels);

    // calc equalize -> [0,1)
    cv::Mat y = x.clone();
    for (int r = 0; r < y.rows; r++)
    {
        float *y_arr = (float *)(y.row(r).data);
        for (int c = 0; c < y.cols; c++)
        {
            const auto lower = std::lower_bound(x_arr, x_arr + num_pixels, y_arr[c]) - x_arr;
            const auto upper = std::upper_bound(x_arr, x_arr + num_pixels, y_arr[c]) - x_arr;
            const float mid = 0.5f * (float)(lower + upper);
            y_arr[c] = mid / (float)num_pixels;
        }
    }

    return float2ubyte(y);
}



cv::Mat read_infrared_array::read_as_raw(std::string filename)
{
    std::ifstream fin(filename, std::ios::binary);

    float nrow = 0.0f;
    float ncol = 0.0f;

    fin.read((char*)&nrow, sizeof(float));
    fin.read((char*)&ncol, sizeof(float));

    cv::Mat img((int)nrow, (int)ncol, CV_32FC1);

    fin.read((char*)img.data, sizeof(float)*nrow*ncol);
    fin.close();

    return img;
}


cv::Mat read_infrared_array::read_as_gray(std::string filename, bool equalize_hist)
{
    cv::Mat img_gray_float, img_gray;

    img_gray_float = read_infrared_array::read_as_raw(filename);

    if (equalize_hist)
    {
        img_gray = equalize_float(img_gray_float);
    }
    else
    {
        img_gray = float2ubyte(img_gray_float);
    }

    cv::cvtColor(img_gray, img_gray, cv::COLOR_GRAY2BGR);

    return img_gray;
}


cv::Mat read_infrared_array::read_as_pcolor(std::string filename)
{
    cv::Mat img_pcolor;

    cv::Mat img_gray = read_infrared_array::read_as_gray(filename);

    cv::applyColorMap(img_gray, img_pcolor, cv::COLORMAP_RAINBOW);

    return img_pcolor;
}
