// Standard libs
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/Image.h"

// OpenCV deps
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>

// Msg type
#include "farm_msgs/Lane.h"

class LaneDetector
{
    ros::NodeHandle nh;
    farm_msgs::Lane lanes;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub_;

public:
    LaneDetector() : it(nh)
    {
        image_sub_ = it.subscribe("/front_cam/image_raw", 1, &LaneDetector::imageCb, this);
        ros::Publisher pub = nh.advertise<farm_msgs::Lane>("/lanes", 1);
    }

    ~LaneDetector(){}

    void imageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        namespace enc = sensor_msgs::image_encodings;

        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        detect_lanes(cv_ptr->image);
        // image_pub_.publish(cv_ptr->toImageMsg());

    }
    void detect_lanes(cv::Mat img)
    {
        cv::Mat src, src_gray;
        cv::Mat dst, detected_edges;

        int edgeThresh = 1;
        int lowThreshold = 200;
        int highThreshold = 300;
        int kernel_size = 5;

        img.copyTo(src);

        cv::cvtColor(img, src_gray, CV_BGR2GRAY);
        cv::blur(src_gray, detected_edges, cv::Size(5, 5));
        cv::Canny(detected_edges, detected_edges, lowThreshold, highThreshold, kernel_size);

        dst = cv::Scalar::all(0);
        img.copyTo(dst, detected_edges);
        dst.copyTo(img);

        // Convert to HSV colorspace
        cv::Mat hsv;
        cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

        // Threshold values for green color, brown color
        cv::Scalar lower_green(40, 40, 40);
        cv::Scalar upper_green(80, 255, 255);
        cv::Scalar lower_brown(0, 0, 0);
        cv::Scalar upper_brown(180, 255, 46);

        // Create mask of threshold range on given img
        cv::Mat mask_green, mask_brown;
        cv::inRange(hsv, lower_green, upper_green, mask_green);
        cv::inRange(hsv, lower_brown, upper_brown, mask_brown);

        // Assuming vanishing point and create lines from it
        cv::Point vanishing_point = cv::Point(img.cols / 2, img.rows / 2);
        cv::line(img, vanishing_point, cv::Point(0, img.rows), cv::Scalar(0, 0, 255), 2);
        cv::line(img, vanishing_point, cv::Point(img.cols, img.rows), cv::Scalar(0, 0, 255), 2);

        // Set ROI
        cv::Mat roi;
        cv::Rect roi_rect = cv::Rect(0, img.rows / 2, img.cols, img.rows / 2);
        roi = img(roi_rect);

        // Apply Perspective transform
        cv::Mat perspective;
        cv::Point2f src_vertices[4];
        cv::Point2f dst_vertices[4];

        src_vertices[0] = cv::Point2f(0, img.rows);
        src_vertices[1] = cv::Point2f(img.cols, img.rows);
        src_vertices[2] = cv::Point2f(img.cols / 2 + 60, img.rows / 2 + 60);
        src_vertices[3] = cv::Point2f(img.cols / 2 - 60, img.rows / 2 + 60);

        dst_vertices[0] = cv::Point2f(0, img.rows);
        dst_vertices[1] = cv::Point2f(img.cols, img.rows);
        dst_vertices[2] = cv::Point2f(img.cols, 0);
        dst_vertices[3] = cv::Point2f(0, 0);

        // Find lines and append it in lanes
        cv::Mat perspective_transform = cv::getPerspectiveTransform(src_vertices, dst_vertices);
        cv::warpPerspective(roi, perspective, perspective_transform, roi.size());

        // Find lines
        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(perspective, lines, 1, CV_PI / 180, 50, 50, 10);

        // Draw lines
        for (size_t i = 0; i < lines.size(); i++)
        {
            cv::Vec4i l = lines[i];
            cv::line(img, cv::Point(l[0], l[1] + img.rows / 2), cv::Point(l[2], l[3] + img.rows / 2), cv::Scalar(0, 0, 255), 3);
        }

        // Show images
        cv::imshow("Original", img);
        cv::imshow("Edges", detected_edges);
        cv::imshow("ROI", roi);
        cv::imshow("Perspective", perspective);
        cv::waitKey(0);
        

        // pub.publish(lanes);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lane_detector");
    LaneDetector ic;
    ros::spin();
    return 0;
}
