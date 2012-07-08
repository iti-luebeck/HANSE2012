
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <dynamic_reconfigure/server.h>
#include <hanse_object_detection/ObjectDetectionConfig.h>
#include "blobs/blob.h"
#include "blobs/BlobResult.h"
#include <vector>
#include <hanse_msgs/Object.h>
#include <eigen3/Eigen/Dense>

typedef enum {R, G, B} Channel;

class ObjectDetection {
public:
    ObjectDetection(ros::NodeHandle nh) {
        image_pub = nh.advertise<sensor_msgs::Image>("/image_object_detection", 1);
        object_pub = nh.advertise<hanse_msgs::Object>("/object", 1);

        channel = R;
        inverted = false;
        use_blobs = false;
        morph_iterations = 5;
        automatic = true;
        threshold = 127;

        cv::Mat meanImage = cv::imread("/home/hanse/frame0000.jpg");
        std::vector<cv::Mat> meanChannels;
        cv::split(meanImage, meanChannels);
        imgMat[0] = Eigen::MatrixXf(meanImage.rows, meanImage.cols);
        imgMat[1] = Eigen::MatrixXf(meanImage.rows, meanImage.cols);
        imgMat[2] = Eigen::MatrixXf(meanImage.rows, meanImage.cols);
        for (int i = 0; i < meanImage.rows; i++) {
            for (int j = 0; j < meanImage.cols; j++) {
                imgMat[0](i, j) = meanChannels[0].at<unsigned char>(i, j);
                imgMat[1](i, j) = meanChannels[1].at<unsigned char>(i, j);
                imgMat[2](i, j) = meanChannels[2].at<unsigned char>(i, j);
            }
        }
    }

    void configCallback(hanse_object_detection::ObjectDetectionConfig &config, uint32_t level)
    {
        switch (config.channel) {
        case 0:
            channel = R;
            break;
        case 1:
            channel = G;
            break;
        case 2:
            channel = B;
            break;
        default:
            channel = B;
            break;
        }

        inverted = config.inverted;
        use_blobs = config.use_blobs;
        morph_iterations = config.morph_iterations;
        threshold = config.threshold;
        automatic = config.automatic;
        subtract_background = config.subtract_background;
        subtract_mean = config.subtract_mean;
    }

    void imageCB(const sensor_msgs::ImageConstPtr& visual_img_msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(visual_img_msg, sensor_msgs::image_encodings::RGB8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge excepion: %s",e.what());
            return;
        }

        cv::Mat image_rgb = cv_ptr->image;
        cv::Mat imageFloat(image_rgb.rows, image_rgb.cols, CV_32FC3);
        std::vector<cv::Mat> channelsChar;
        std::vector<cv::Mat> channelsFloat;
        cv::split(image_rgb, channelsChar);
        cv::split(imageFloat, channelsFloat);

        for (int c = 0; c < channelsFloat.size(); c++) {
            for (int i = 0; i < channelsFloat[c].rows; i++) {
                for (int j = 0; j < channelsFloat[c].cols; j++) {
                    channelsFloat[c].at<float>(i, j) =(float)channelsChar[c].at<unsigned char>(i, j);
                }
            }
        }

        if (subtract_background) {
            float max = 0.f;
            float min = 255.f;
            for (int c = 0; c < channelsFloat.size(); c++) {
                for (int i = 0; i < channelsFloat[c].rows; i++) {
                    for (int j = 0; j < channelsFloat[c].cols; j++) {
                        channelsFloat[c].at<float>(i, j) = channelsFloat[c].at<float>(i, j) / imgMat[c](i, j);
                        if (channelsFloat[c].at<float>(i, j) > max) {
                            max = channelsFloat[c].at<float>(i, j);
                        }
                        if (channelsFloat[c].at<float>(i, j) < min) {
                            min = channelsFloat[c].at<float>(i, j);
                        }
                    }
                }
            }

            for (int c = 0; c < channelsFloat.size(); c++) {
                for (int i = 0; i < channelsFloat[c].rows; i++) {
                    for (int j = 0; j < channelsFloat[c].cols; j++) {
                        channelsFloat[c].at<float>(i, j) = ((channelsFloat[c].at<float>(i, j) - min) * (255.f / max));
                    }
                }
            }
        }

        cv::Mat grayFloat;

        switch (channel) {
        case R:
            grayFloat = channelsFloat[0];
            break;
        case G:
            grayFloat = channelsFloat[1];
            break;
        case B:
            grayFloat = channelsFloat[2];
            break;
        }

        if (subtract_mean) {
            float max = 0.f;
            float min = 255.f;
            for (int i = 0; i < grayFloat.rows; i++) {
                for (int j = 0; j < grayFloat.cols; j++) {
                    grayFloat.at<float>(i, j) =
                            127.5f + 0.5f * (grayFloat.at<float>(i, j) /
                                             ((1 / (255.f * 3.f)) * (channelsFloat[0].at<float>(i, j) +
                                                           channelsFloat[1].at<float>(i, j) +
                                                           channelsFloat[2].at<float>(i, j))));
                    if (grayFloat.at<float>(i, j) > max) {
                        max = grayFloat.at<float>(i, j);
                    }
                    if (grayFloat.at<float>(i, j) < min) {
                        min = grayFloat.at<float>(i, j);
                    }
                }
            }

            for (int i = 0; i < grayFloat.rows; i++) {
                for (int j = 0; j < grayFloat.cols; j++) {
                    grayFloat.at<float>(i, j) = ((grayFloat.at<float>(i, j) - min) * (255.f / max));
                }
            }
        }

        cv::Mat grayChar(grayFloat.rows, grayFloat.cols, CV_8U);
        for (int i = 0; i < grayFloat.rows; i++) {
            for (int j = 0; j < grayFloat.cols; j++) {
                grayChar.at<unsigned char>(i, j) = (unsigned char)grayFloat.at<float>(i, j);
            }
        }

        cv::Mat binary;

        int thresholdType = 0;
        if (inverted) {
            thresholdType |= cv::THRESH_BINARY_INV;
        } else {
            thresholdType |= cv::THRESH_BINARY;
        }
        if (automatic) {
            thresholdType |= cv::THRESH_OTSU;
        }

        double thresh = cv::threshold(grayChar, binary, threshold, 255, thresholdType);

        cv::erode(binary, binary, cv::Mat(), cv::Point(-1,-1), morph_iterations);
        cv::dilate(binary, binary, cv::Mat(), cv::Point(-1,-1), morph_iterations);

        float size = 0;
        float orientation = 0;
        float x = 0;
        float y = 0;
        float blobRatio = 1.0f;

        if (use_blobs) {            
            IplImage *thresh = new IplImage(binary);
            CBlobResult blobs(thresh, NULL, 0);
            blobs.Filter(blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, 100 );

            // Get largest blob.
            int maxArea = 0;
            int maxBlob = -1;
            for (int j = 0; j < blobs.GetNumBlobs(); j++) {
                CBlob *blob = blobs.GetBlob(j);
                if (blob->Area() > maxArea) {
                    maxArea = blob->Area();
                    maxBlob = j;
                }
            }

            if (maxBlob >= 0) {
                size = blobs.GetBlob(maxBlob)->Moment(0, 0);

                // First order moments -> mean position
                double m10 = blobs.GetBlob(maxBlob)->Moment(1, 0) / size;
                double m01 = blobs.GetBlob(maxBlob)->Moment(0, 1) / size;
                x = m10;
                y = m01;

                // Second order moments -> orientation
                double mu11 = blobs.GetBlob(maxBlob)->Moment(1, 1);
                double mu20 = blobs.GetBlob(maxBlob)->Moment(2, 0);
                double mu02 = blobs.GetBlob(maxBlob)->Moment(0, 2);
                orientation = 0.5 * atan2( 2 * mu11 , ( mu20 - mu02 ) );

                // Draw blob to gray image.
                binary = binary.setTo(cv::Scalar(0));
                blobs.GetBlob(maxBlob)->FillBlob(thresh, cvScalar(255), 0, 0);
            }
        } else {
            cv::Moments moment = cv::moments(binary, true);

            size = moment.m00;

            // First order moments -> mean position
            x = moment.m10 / size;
            y = moment.m01 / size;

            // Second order moments -> orientation
            double mu11 = moment.mu11;
            double mu20 = moment.mu20;
            double mu02 = moment.mu02;
            orientation = 0.5 * atan2( 2 * mu11 , ( mu20 - mu02 ) );
            blobRatio = mu20 / mu02;
        }

        size /= (binary.rows * binary.cols);

        bool isGood = true;
        isGood &= (size < 0.5);                                 // Criterium 1: size
        isGood &= (blobRatio < 0.8f || (blobRatio > 1.2f));     // Criterium 2: ratio
        isGood &= (thresh > 6);                                 // Criterium 3: threshold

        if (isGood) {
            cv::line(binary, cv::Point(x, y),
                    cv::Point(x + cos(orientation)*200, y + sin(orientation)*200),
                    cv::Scalar(120), 4, CV_FILLED);
        }

        // Theta is now the rotation angle reletive to the x axis.
        // We want it relative to the y axis -> 90° ccw
        orientation -= CV_PI / 2;
        if (orientation < -CV_PI) {
            orientation += CV_PI;
        }

        if (orientation < -CV_PI/2) {
            orientation += CV_PI;
        } else if (orientation > CV_PI/2) {
            orientation -= CV_PI;
        }

        hanse_msgs::Object objMsg;
        objMsg.header = visual_img_msg->header;
        objMsg.size = size;
        objMsg.x = x;
        objMsg.y = y;
        objMsg.orientation = orientation;
        object_pub.publish(objMsg);

        cv_bridge::CvImage out_msg;
        out_msg.header   = visual_img_msg->header;
        out_msg.encoding = sensor_msgs::image_encodings::MONO8;
        out_msg.image    = binary;
        image_pub.publish(out_msg.toImageMsg());
    }

private:
    ros::Publisher image_pub;
    ros::Publisher object_pub;
    Channel channel;
    bool inverted;
    bool use_blobs;
    int morph_iterations;
    bool automatic;
    int threshold;
    bool subtract_background;
    bool subtract_mean;

    Eigen::MatrixXf imgMat[3];
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_detection_node");
  ros::NodeHandle nh("~");

  std::string image_topic = "/hanse/camera/bottom";
  nh.getParam("image_topic", image_topic);

  ObjectDetection od(nh);

  dynamic_reconfigure::Server<hanse_object_detection::ObjectDetectionConfig> server;
  dynamic_reconfigure::Server<hanse_object_detection::ObjectDetectionConfig>::CallbackType f;

  f = boost::bind(&ObjectDetection::configCallback, &od, _1, _2);
  server.setCallback(f);

  ros::Subscriber camSub =
          nh.subscribe<sensor_msgs::Image>(image_topic, 1, boost::bind(&ObjectDetection::imageCB, &od, _1));

  ros::spin();
}
