
#include "calc_shift.h"
#include "opencv2/xfeatures2d.hpp"
#include <vector>
#include <stdio.h>
#include <dirent.h>

#include "kcftracker.hpp"

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;

#define DEBUG(format, ...) printf("<debug> %s line=%04d func=%s: ", basename(__FILE__), __LINE__,__FUNCTION__),printf(format"\n", ##__VA_ARGS__)
#define DEBUG_SHOW_IMAGE false

#define ASSERT_RETURN(expr,val_ret) \
{ \
    if ((expr) == false) \
    { \
        DEBUG("%s is false", #expr); \
        return val_ret; \
    } \
}

//-- Show detected matches
static void show_match_result(const cv::Mat &img_object, const std::vector<cv::KeyPoint> &keypoints_object,
                              const cv::Mat &img_scene,  const std::vector<cv::KeyPoint> &keypoints_scene,
                              const std::vector< cv::DMatch > &good_matches,
                              const std::string winname="match_result")
{
    if (DEBUG_SHOW_IMAGE != true)
        return;
    // show matches map
    cv::Mat img_matches;
    cv::drawMatches(img_object, keypoints_object,
                    img_scene,  keypoints_scene,
                    good_matches,
                    img_matches,
                    cv::Scalar::all(-1), cv::Scalar::all(-1),
                    std::vector<char>(),
                    cv::DrawMatchesFlags::DEFAULT);

    cv::namedWindow(winname + "-matches", cv::WINDOW_NORMAL);
    cv::imshow(winname + "-matches", img_matches );

    // show matches offset
    const int rows = img_object.rows + img_scene.rows;
    const int cols = img_object.cols + img_scene.cols;
    const cv::Point2f center_point((float)(cols / 2.0), (float)(rows / 2.0));
    const int thickness = std::max(1, (int)std::round((rows+cols)/1000.0));

    cv::Mat img_offset = cv::Mat::zeros(rows, cols, CV_32FC3);

    const int num_matches = (int)good_matches.size();
    for (int k = 0; k < num_matches; k++)
    {
        const cv::Point2f pt_obj = keypoints_object[ good_matches[k].queryIdx ].pt;
        const cv::Point2f pt_sce = keypoints_scene[ good_matches[k].trainIdx ].pt;
        const cv::Point2f pt_offset = center_point + pt_obj - pt_sce;
        cv::Mat img_temp = cv::Mat::zeros(rows, cols, CV_32FC3);
        cv::circle(img_temp, pt_offset, 0, cv::Scalar(0,0,1.0), thickness, cv::LINE_AA);
        img_offset += img_temp;
    }

    double minVal = 0, maxVal = 0;
    cv::minMaxLoc(img_offset, &minVal, &maxVal);
    img_offset.convertTo(img_offset, CV_8UC1, 255.0/(maxVal-minVal+0.0001), -255*minVal/(maxVal-minVal+0.0001));
    cv::circle(img_offset, center_point, thickness, cv::Scalar(255,0,0), thickness, cv::LINE_AA);
    cv::namedWindow(winname + "-offset", cv::WINDOW_NORMAL);
    cv::imshow(winname + "-offset", img_offset );
}



static void show_image_with_roi(const cv::Mat img, const cv::Rect roi,
                                std::string winname = "roi", const cv::Scalar scalar = cv::Scalar(255,0,0))
{
    if (DEBUG_SHOW_IMAGE != true)
        return;
    cv::Mat img_show = img.clone();
    const int thickness = std::max(1, (int)std::round((img.rows+img.cols)/1000.0));
    cv::rectangle(img_show, roi, cv::Scalar(255, 0, 0 ), thickness);
    cv::namedWindow(winname, cv::WINDOW_NORMAL);
    cv::imshow(winname, img_show);
}




static cv::Point2f calc_shift_using_kpts_mean(const vector<cv::Point2f> &_vec_shifts)
{
    vector<cv::Point2f> vec_shifts(_vec_shifts);
    vector<double> vec_costs(vec_shifts.size());
    cv::Point2f shift = cv::Point2f(0,0);
    ASSERT_RETURN(vec_shifts.size() == vec_costs.size(), cv::Point2f(0,0));
    for (size_t i = 0; i < vec_shifts.size(); i++)
    {
        vec_costs[i] = 0.0;
        for (size_t j = 0; j < vec_shifts.size(); j++)
        {
            if (i == j) continue;
            vec_costs[i] += cv::norm(vec_shifts[i] - vec_shifts[j]);
        }
    }

    // sort vec shifts and costs
    for (size_t i = 0; i < vec_shifts.size(); i++)
    {
        for (size_t j = i+1; j < vec_shifts.size(); j++)
        {
            if (vec_costs[i] <= vec_costs[j]) continue;
            std::swap(vec_costs[i], vec_costs[j]);
            std::swap(vec_shifts[i], vec_shifts[j]);
        }
    }

    const int num_to_calc = (int)(vec_shifts.size()/2);
    ASSERT_RETURN(num_to_calc > 0, cv::Point2f(0,0));

    cv::Point2f shift_sum = cv::Point2f(0,0);
    for (int i = 0; i < num_to_calc; i++)
    {
        shift_sum = shift_sum + vec_shifts[i];
    }
    shift = shift_sum / num_to_calc;

    return shift;
}




static cv::Point2f calc_shift_using_kpts_perspective(const cv::Mat &M, const cv::Rect &roi)
{
    ASSERT_RETURN(!M.empty(), cv::Point2f(0,0));

    const cv::Point2f center_point = cv::Point2f((float)(roi.x + roi.width /2),
                                                 (float)(roi.y + roi.height/2));
    
    const std::vector<cv::Point2f> src_points {center_point};
    std::vector<cv::Point2f> dst_points;
    
	cv::perspectiveTransform(src_points, dst_points, M);

    ASSERT_RETURN(!src_points.empty(), cv::Point2f(0,0));
    ASSERT_RETURN(!dst_points.empty(), cv::Point2f(0,0));
    
    return dst_points[0]-src_points[0];
}



// return good match num
static int calc_kpts_good_matches(const cv::Mat &img_preset, const cv::Mat &img_run,
                                  std::vector<KeyPoint> &keypoints_preset, std::vector<KeyPoint> &keypoints_run,
                                  std::vector<std::vector<DMatch>> &knn_matches, std::vector<DMatch> &good_matches)
{
    keypoints_preset.clear();
    keypoints_run.clear();
    knn_matches.clear();
    good_matches.clear();
    // Step-1: Detect the keypoints, and compute the descriptors
    int minHessian = 1000;
    Ptr<SURF> detector = SURF::create();
    detector->setHessianThreshold(minHessian);
    Mat descriptors_preset, descriptors_run;

    // detect key points
    detector->detect(img_preset, keypoints_preset);
    detector->detect(img_run,    keypoints_run);
    
    cv::KeyPointsFilter::retainBest(keypoints_preset, 1000);
    cv::KeyPointsFilter::retainBest(keypoints_run, 1000);
    DEBUG("keypoints_preset.size() = %lu, keypoints_run.size() = %lu", keypoints_preset.size(), keypoints_run.size());

    detector->compute(img_preset, keypoints_preset, descriptors_preset);
    detector->compute(img_run,    keypoints_run, descriptors_run);

    ASSERT_RETURN(keypoints_preset.size() >= 2, 0);
    ASSERT_RETURN(keypoints_run.size() >= 2   , 0);

    ASSERT_RETURN(descriptors_preset.data != NULL, 0);
    ASSERT_RETURN(descriptors_run.data != NULL   , 0);

    // Step-2: Matching descriptor vectors using FLANN matcher
    FlannBasedMatcher matcher;
    const float minRatio = 1.f / 1.5f;
    matcher.knnMatch(descriptors_run, descriptors_preset, knn_matches, 2);
    ASSERT_RETURN(!knn_matches.empty(), 0);

    for (auto &knn_matche : knn_matches)
    {
        const DMatch &bestMatch = knn_matche[0];
        const DMatch &betterMatch = knn_matche[1];
        if (bestMatch.distance < betterMatch.distance * minRatio)
        {
            good_matches.push_back(bestMatch);
        }
    }

    show_match_result(img_run, keypoints_run, img_preset, keypoints_preset, good_matches, "good matches");
    return (int)good_matches.size();
}



// return shift and keypoints number(retained finally to calculate shift
static double calc_shift_using_keypoints(const cv::Mat &_img_preset, const cv::Mat &_img_run, const cv::Rect &_roi_preset, cv::Point2f &shift)
{
    const int max_len = 1000;
    const double fx = std::min(max_len / (double)_img_preset.cols, 1.0);
    const double fy = std::min(max_len / (double)_img_preset.rows, 1.0);
    const double fxy = std::min(fx, fy);

    cv::Mat img_preset, img_run;
    cv::resize(_img_preset, img_preset, cv::Size(), fxy, fxy, cv::INTER_AREA);
    cv::resize(_img_run,    img_run,    cv::Size(), fxy, fxy, cv::INTER_AREA);

    const cv::Rect roi_preset = cv::Rect((int)round(_roi_preset.x     * fxy), (int)round(_roi_preset.y      * fxy),
                                         (int)round(_roi_preset.width * fxy), (int)round(_roi_preset.height * fxy));

    shift = cv::Point2f(0,0);
    std::vector<KeyPoint> keypoints_preset, keypoints_run;
    std::vector<DMatch> good_matches;
    std::vector<std::vector<DMatch>> knn_matches;
    calc_kpts_good_matches(img_preset, img_run, keypoints_preset, keypoints_run, knn_matches, good_matches);
    ASSERT_RETURN(!good_matches.empty(), 0.0);

    vector<Point2f> srcPoints(good_matches.size());
    vector<Point2f> dstPoints(good_matches.size());
    for (size_t i = 0, size = good_matches.size(); i < size; i++)
    {
        srcPoints[i] = keypoints_preset[good_matches[i].trainIdx].pt;
        dstPoints[i] = keypoints_run[good_matches[i].queryIdx].pt;
    }

    vector<unsigned char> inliers_mask(good_matches.size());
    const Mat T = findHomography(srcPoints, dstPoints, RANSAC, 3, inliers_mask);
    // std::cout<< "\nperspective matrix:\n"<< T <<std::endl <<std::endl;

    // 统计每个点与其他点距离之和
    vector<DMatch> homography_matches;
    vector<cv::Point2f> vec_shifts;
    for (size_t i = 0, size = inliers_mask.size(); i < size; i++)
    {
        if (inliers_mask[i])
        {
            homography_matches.push_back(good_matches[i]);
            const cv::Point keypoint_query = keypoints_run[good_matches[i].queryIdx].pt;
            const cv::Point keypoint_train = keypoints_preset[good_matches[i].trainIdx].pt;
            vec_shifts.push_back(cv::Point2f(keypoint_query - keypoint_train));
        }
    }
    DEBUG("knn_matches, good_matches, inliers_mask, homography_matches = %lu, %lu, %lu, %lu",
            knn_matches.size(), good_matches.size(), inliers_mask.size(), homography_matches.size());
    show_match_result(img_run, keypoints_run, img_preset, keypoints_preset, homography_matches, "Homography inliers");

    ASSERT_RETURN(!T.empty(), 0.0);
    ASSERT_RETURN(homography_matches.size()>=4, 0.0);

    const cv::Point2f shift_mean = calc_shift_using_kpts_mean(vec_shifts);
    const cv::Point2f shift_perspective = calc_shift_using_kpts_perspective(T, roi_preset);
    
    const int matches_num_perspective = 10;
    const double fxy_max = 1.1;
    const double fxy_min = 0.9;
    double weight_perspective = 0.0;
    
    if (homography_matches.size() >= matches_num_perspective
        && T.at<double>(0,0) > fxy_min && T.at<double>(0,0) < fxy_max
        && T.at<double>(1,1) > fxy_min && T.at<double>(1,1) < fxy_max)
        weight_perspective = 1.0;
    else
        weight_perspective = 0.0;
    
    DEBUG("weight_perspective = %f", weight_perspective);
    shift = shift_mean * (1.0 - weight_perspective) + shift_perspective * weight_perspective;

    show_image_with_roi(img_preset, roi_preset, "keypoints img_preset");
    show_image_with_roi(img_run, cv::Rect(roi_preset.x + shift.x, roi_preset.y + shift.y, roi_preset.width, roi_preset.height), "keypoints img_run");

    shift /= fxy;
    const double confidence = std::min(1.0, (double)homography_matches.size()/20.0);
    return confidence;
}





static double calc_shift_using_kcf(const cv::Mat &img_preset, const cv::Mat &img_run, const cv::Rect &_roi_preset, cv::Point2f &shift)
{
    cv::Rect roi_preset = _roi_preset;
    const int roi_w = img_preset.cols / 5;
    const int roi_h = img_preset.rows / 5;

    roi_preset.x = (int)round((roi_preset.x + 0.5*roi_preset.width) - 0.5*roi_w);
    roi_preset.width  = roi_w;

    roi_preset.y = (int)round((roi_preset.y + 0.5*roi_preset.height) - 0.5*roi_h);
    roi_preset.height = roi_h;
    
    // Create KCFTracker object
    KCFTracker tracker(true, false, false, false);
    tracker.padding = std::max((float)img_preset.cols/(float)roi_preset.width,
                               (float)img_preset.rows/(float)roi_preset.height);

    tracker.init(roi_preset, img_preset);
    float peak_value = 0.f; // 该位置与预置位的相关系数
    const cv::Rect roi_run = tracker.update(img_run, peak_value);
    DEBUG("kcf peak_value = %f", peak_value);

    shift.x = (float)roi_run.x - roi_preset.x;
    shift.y = (float)roi_run.y - roi_preset.y;

    show_image_with_roi(img_preset, _roi_preset, "kcf img_preset", cv::Scalar(255,0,0));
    show_image_with_roi(img_run, _roi_preset+cv::Point(shift), "kcf img_run", cv::Scalar(255,0,0));

    const double confidence = std::min(1.0, peak_value/0.6);
    return confidence;
}




double calc_shift(const cv::Mat &img_preset, const cv::Mat &img_run, const cv::Rect &_roi_preset, cv::Point2f &shift)
{
    cv::Rect roi_preset = _roi_preset;
    ASSERT_RETURN(img_preset.cols == img_run.cols && img_preset.rows == img_run.rows, 0.0);
    ASSERT_RETURN(img_preset.cols > 0 && img_preset.rows > 0, 0.0);

    const int w_min = img_preset.cols / 5;
    const int h_min = img_preset.rows / 5;
    if (roi_preset.width == 0 || roi_preset.height == 0) // no roi exist
    {
        roi_preset = Rect((img_preset.cols - w_min) / 2, (img_preset.rows - h_min) / 2, w_min, h_min);
        DEBUG("roi_preset's width and height is 0, use default rect: (x,y,w,h) = (%d, %d, %d,%d)",
                  roi_preset.x, roi_preset.y, roi_preset.width, roi_preset.height);
    }

    cv::Point2f shift_keypoints;
    cv::Point2f shift_kcf;
    
    const double confidence_trust = 0.90;
    const double confidence_keypoints = calc_shift_using_keypoints(img_preset, img_run, roi_preset, shift_keypoints);
    DEBUG("keypoints confidence is %0.4f, confidence_trust is %f", confidence_keypoints, confidence_trust);
    
    if (confidence_keypoints > confidence_trust)
    {
        shift = shift_keypoints;
        DEBUG("using shift from keypoints directly.");
        return confidence_keypoints;
    }

    DEBUG("trying KCF tracker.");
    
    const int max_width  = 512;
    const int max_height = 512;
    const double fx = std::min(max_width  / (double)img_preset.cols, 1.0);
    const double fy = std::min(max_height / (double)img_preset.rows, 1.0);
    const double fxy = std::min(fx, fy);

    cv::Mat tmp_preset, tmp_run;
    cv::resize(img_preset, tmp_preset, cv::Size(), fxy, fxy, cv::INTER_AREA);
    cv::resize(img_run,    tmp_run,    cv::Size(), fxy, fxy, cv::INTER_AREA);

    const cv::Rect tmp_roi = cv::Rect((int)round(roi_preset.x * fxy), (int)round(roi_preset.y * fxy),
                                      (int)round(roi_preset.width * fxy), (int)round(roi_preset.height * fxy));

    const double confidence_kcf = calc_shift_using_kcf(tmp_preset, tmp_run, tmp_roi, shift_kcf);
    shift_kcf = shift_kcf / fxy;

    DEBUG("keypoints confidence is %0.4f, kcf confidence is %0.4f", confidence_keypoints, confidence_kcf);
    if (confidence_keypoints < confidence_kcf)
    {
        shift = shift_kcf;
        DEBUG("using shift from kcf.");
        return confidence_kcf;
    }
    else
    {
        shift = shift_keypoints;
        DEBUG("using shift from keypoints.");
        return confidence_keypoints;
    }
    
    return 0.0;
}
