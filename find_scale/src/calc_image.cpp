
#include "find_scale.h"
#include "read_infrared_array.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "calc_image");
    ros::Time::init();
    std::string ann_path_preset;
    std::string img_path_run;
    double dZoomLevel; // 放大倍数
    FindScale::FindScale calc_image;

    ROS_INFO("build time: %s %s", __DATE__, __TIME__);
    if (argc >= 4 && argc % 3 != 1)
    {
        ROS_ERROR("usage: calc_image [annotation_preset] [picture_run] [zoom_level]");
        exit(-1);
    }

    const int num_test = (argc - 1) / 3;
    for (int k = 0; ; k++)
    {
        const int k_test = k % num_test;
        printf("\n\n-----------------------------------------------\n");
        printf("k = %d of %d\n", k_test, num_test);
        ann_path_preset = std::string(argv[k_test*3+1]);
        img_path_run    = std::string(argv[k_test*3+2]);
        dZoomLevel      = atof       (argv[k_test*3+3]);
        
        ROS_INFO("ann_path_preset: %s", ann_path_preset.c_str());
        ROS_INFO("img_path_run: %s", img_path_run.c_str());
        ROS_INFO("zoom_level:%.1f", dZoomLevel);
        double dPanAngleDelta;
        double dTiltAngleDelta;

        // Read images and annotation
        cv::Mat img_run, img_preset;
        if ((int)img_path_run.find_last_of(".") > (int)ann_path_preset.find_last_of("/"))
        {
            ROS_INFO("reading jpg");
            std::string img_path_preset = ann_path_preset.substr(0, ann_path_preset.find_last_of(".")) + ".jpg";
            img_run = cv::imread(img_path_run);
            img_preset = cv::imread(img_path_preset, cv::IMREAD_COLOR);
        }
        else
        {
            ROS_INFO("reading thermal");
            std::string img_path_preset = ann_path_preset.substr(0, ann_path_preset.find_last_of("."));
            img_run = read_infrared_array::read_as_gray(img_path_run);
            img_preset = read_infrared_array::read_as_gray(img_path_preset);
        }
        
        cv::Rect roi_preset;
        if (!WtAnnotation::LoadRoi(ann_path_preset, roi_preset))
        {
            ROS_INFO("load roi from file:%s failed, return rect:[%d, %d, %d, %d]",
                     ann_path_preset.c_str(), roi_preset.x, roi_preset.y, roi_preset.width, roi_preset.height);
        }
        
        std::vector<int> vnRoiVertex;

        if (!calc_image.CalcImageAngle(img_preset, img_run, roi_preset, dZoomLevel, dPanAngleDelta, dTiltAngleDelta, vnRoiVertex))
        {
            ROS_INFO("img_run.size = (%d,%d)", img_run.rows, img_run.cols);
            ROS_INFO("img_preset.size = (%d,%d)", img_preset.rows, img_preset.cols);
            ROS_ERROR("CalcImageAngle failed");
            continue; //exit(0);
        }

        ROS_INFO("calculate angle delta result:[pan:%.2f, tilt:%.2f] DEG", dPanAngleDelta * 180.0/CV_PI, dTiltAngleDelta * 180.0/CV_PI);

        cv::Mat img_show = img_preset.clone();
        cv::rectangle( img_show, roi_preset, cv::Scalar(255, 0, 0 ), 2, 8 );
        cv::putText(img_show, "preset", cv::Point(roi_preset.x, roi_preset.y-4),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255,0,0), 2);
        cv::namedWindow("preset", cv::WINDOW_NORMAL);
        cv::imshow("preset", img_show);

        img_show = img_run.clone();
        const double shift_x = tan(dPanAngleDelta)  * dZoomLevel * 4.5 * img_run.cols / FindScale::camera_sensor_w;
        const double shift_y = tan(dTiltAngleDelta) * dZoomLevel * 4.5 * img_run.rows / FindScale::camera_sensor_h;
        const cv::Rect roi_run = cv::Rect(roi_preset.x - shift_x, roi_preset.y - shift_y, roi_preset.width, roi_preset.height);
        ROS_INFO("shift_x = %f, shift_y = %f", shift_x, shift_y);
        cv::rectangle( img_show, roi_preset, cv::Scalar(0, 255, 0), 2, 8 );
        cv::rectangle( img_show, roi_run, cv::Scalar(255, 0, 0), 2, 8 );
        cv::putText(img_show, "run", cv::Point(roi_run.x, roi_run.y-4),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255,0,0), 2);
        cv::namedWindow("run", cv::WINDOW_NORMAL);
        cv::imshow("run", img_show);
        if (cv::waitKey(1000) == 32)
            cv::waitKey();
    }

    exit(0);
}