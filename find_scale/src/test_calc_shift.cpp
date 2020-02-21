
#include "calc_shift.h"
#include "WtAnnotation.h"
#include <iostream>
#include <vector>


#define DEBUG(format, ...) printf(format"\n", ##__VA_ARGS__)

int main(int argc, char **argv)
{
    DEBUG("build time: %s %s", __DATE__, __TIME__);
    if (argc < 5)
    {
        DEBUG("argv[1] must be preset directory.");
        DEBUG("argv[2] must be a ratio threshold in [0,1].");
        DEBUG("argv[3] must be min zoom level");
        DEBUG("argv[4] must be max zoom level");
        return 0;
    }

    const double threshold = atof(argv[2]);
    const int zool_level_min = atof(argv[3]);
    const int zool_level_max = atof(argv[4]);

    const int64 tick_start = cv::getTickCount();

    const std::string preset_dir = argv[1];
    DEBUG("preset_dir = %s", preset_dir.c_str());
    
    std::vector<cv::String> preset_zoomx_vector;
    for (int zoom_level = zool_level_min; zoom_level <= zool_level_max; zoom_level++)
    {
        std::vector<cv::String> temp_vector;
        cv::glob(cv::format("%s/*zoom%d.xml", preset_dir.c_str(), zoom_level), temp_vector, true);
        DEBUG("zoom_level = %d, preset zoom len = %d", zoom_level, (int)temp_vector.size());
        for (auto it = temp_vector.begin(); it != temp_vector.end(); ++it)
        {
            preset_zoomx_vector.push_back(*it);
        }
    }
    DEBUG("preset list num = %d", (int)preset_zoomx_vector.size());

    std::vector<cv::String> error_images;
    std::vector<double> error_ratios;
    for (auto it_path = preset_zoomx_vector.begin(); it_path != preset_zoomx_vector.end(); ++it_path)
    {
        DEBUG("processing %s", it_path->c_str());
        std::vector<cv::String> xmls_run;
        
        const std::string curr_pattern = it_path->substr(0, it_path->find_last_of(".")) + "_*.xml";
        DEBUG("curr_pattern = %s", curr_pattern.c_str());
        cv::glob(curr_pattern, xmls_run, true);
        
        for (auto it_xml = xmls_run.begin(); it_xml != xmls_run.end(); ++it_xml)
        {
            const std::string img_ext = std::string(".jpg");
            
            const std::string ann_path_preset = *it_path;
            const std::string img_path_preset = ann_path_preset.substr(0, ann_path_preset.find_last_of(".")) + img_ext;
            
            const std::string ann_path_run = *it_xml;
            const std::string img_path_run = it_xml->substr(0, it_xml->find_last_of(".")) + img_ext;

            DEBUG("-------------------------------------------------------------");
            DEBUG("ann_path_preset: %s", ann_path_preset.c_str());
            DEBUG("img_path_run: %s", img_path_run.c_str());

            const cv::Mat img_preset = cv::imread(img_path_preset, cv::IMREAD_COLOR);
            const cv::Mat img_run    = cv::imread(img_path_run,    cv::IMREAD_COLOR);
            
            cv::Rect ann_roi_preset, ann_roi_run;
            WtAnnotation::LoadRoi(ann_path_preset, ann_roi_preset);
            WtAnnotation::LoadRoi(ann_path_run,    ann_roi_run);
            
            cv::Point2f shift;
            const double confidence = calc_shift(img_preset, img_run, ann_roi_preset, shift);
            const double time_span = (cv::getTickCount() - tick_start)/cv::getTickFrequency();
            DEBUG("calc_shift time consumed: %6.4f sec", time_span);
            DEBUG("calc shift confidence = %f", confidence);
            
            std::cout<< "ann_roi_preset = " << ann_roi_preset << std::endl;
            std::cout<< "shift = " << shift << std::endl;

            cv::Rect roi_run = cv::Rect(ann_roi_preset.x + shift.x, ann_roi_preset.y + shift.y,
                                        ann_roi_preset.width, ann_roi_preset.height);
            std::cout<< "roi_run = " << roi_run << std::endl;

            // collect error images
            const double err_x = abs(ann_roi_run.x - roi_run.x) / (double)ann_roi_run.width;
            const double err_y = abs(ann_roi_run.y - roi_run.y) / (double)ann_roi_run.height;
            const double err_ratio = std::max(err_x, err_y);

            if (err_ratio > threshold)
            {
                error_images.push_back(img_path_run);
                error_ratios.push_back(err_ratio);
            }

            cv::Mat img_show = img_preset.clone();
            cv::rectangle(img_show, ann_roi_preset, cv::Scalar(255, 0, 0 ), 2, 8 );
            cv::putText(img_show, "roi", cv::Point(ann_roi_preset.x, ann_roi_preset.y-4),
                        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255,0,0), 2);
            cv::namedWindow("preset", cv::WINDOW_NORMAL);
            cv::imshow("preset", img_show);

            img_show = img_run.clone();
            cv::rectangle(img_show, ann_roi_run, cv::Scalar(255, 0, 0 ), 1, 8 );
            cv::putText(img_show, "ann", cv::Point(ann_roi_run.x, ann_roi_run.y-4),
                        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255,0,0), 1);
            cv::rectangle(img_show, roi_run, cv::Scalar( 0, 0, 255), 2, 8 );
            cv::putText(img_show, "calc", cv::Point(roi_run.x, roi_run.y-4),
                        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0,0,255), 2);

            cv::namedWindow("run", cv::WINDOW_NORMAL);
            cv::imshow("run", img_show);
            if (cv::waitKey(100) == 32)
                cv::waitKey();
        }
    }

    DEBUG("\n\n------------------------------------------------------------------------");
    DEBUG("error images num = %d", (int)error_images.size());
    DEBUG("error ratios num = %d", (int)error_ratios.size());
    for (int k = 0; k < (int)error_images.size(); ++k)
    {
        for (int kk = 0; kk < (int)error_images.size() - k - 1; ++kk)
        {
            if (error_ratios[k] > error_ratios[k+1])
            {
                std::swap(error_ratios[k], error_ratios[k+1]);
                std::swap(error_images[k], error_images[k+1]);
            }
        }
    }
    for (int k = 0; k < (int)error_images.size(); ++k)
    {
        DEBUG("error = %05.3f, image path: %s", error_ratios[k], error_images[k].c_str());
    }

    return 0;
}