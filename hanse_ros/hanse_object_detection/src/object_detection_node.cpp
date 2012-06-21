
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

typedef enum {R, G, B} Channel;

class ObjectDetection {
public:
    ObjectDetection(ros::NodeHandle nh) {
        image_pub = nh.advertise<sensor_msgs::Image>("/image_object_detection", 1);
        object_pub = nh.advertise<hanse_msgs::Object>("/object", 1);

        channel = R;
        inverted = false;
    }

    void configCallback(hanse_object_detection::ObjectDetectionConfig &config, uint32_t level)
    {
        std::string channelStr = config.channel;
        if (channelStr.compare("R")) {
            channel = R;
        } else if (channelStr.compare("G")) {
            channel = G;
        } else if (channelStr.compare("B")) {
            channel = B;
        }
        inverted = config.inverted;
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

        std::vector<cv::Mat> channels;
        cv::split(image_rgb, channels);
        cv::Mat gray;

        switch (channel) {
        case R:
            gray = channels[0];
            break;
        case G:
            gray = channels[1];
            break;
        case B:
            gray = channels[2];
            break;
        }

        cv::Mat binary;

        if (inverted) {
            cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
        } else {
            cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        }

        IplImage *thresh = new IplImage(binary);
        cvDilate(thresh, thresh, NULL, 5);
        cvErode(thresh, thresh, NULL, 5);

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

        float size = 0;
        float orientation = 0;
        float x = 0;
        float y = 0;

        if (maxBlob >= 0) {
            size = blobs.GetBlob(maxBlob)->Moment(0, 0);

            // First order moments -> mean position
            double m10 = blobs.GetBlob(maxBlob)->Moment(1, 0) / size;
            double m01 = blobs.GetBlob(maxBlob)->Moment(0, 1) / size;
            x = m10;
            y = m01;

            // Second order moments -> orientation
            double mu11 = blobs.GetBlob(maxBlob)->Moment(1, 1) / size;
            double mu20 = blobs.GetBlob(maxBlob)->Moment(2, 0) / size;
            double mu02 = blobs.GetBlob(maxBlob)->Moment(0, 2) / size;
            orientation = 0.5 * atan2( 2 * mu11 , ( mu20 - mu02 ) );

            // Draw blob to gray image.
            binary = binary.setTo(cv::Scalar(0));
            blobs.GetBlob(maxBlob)->FillBlob(thresh, cvScalar(255), 0, 0);
        }

        CvMoments M;
        cvMoments(thresh, &M, 1);

        // Second order moments -> orientation
        double mu11 = cvGetCentralMoment( &M, 1, 1 ) / size;
        double mu20 = cvGetCentralMoment( &M, 2, 0 ) / size;
        double mu02 = cvGetCentralMoment( &M, 0, 2 ) / size;
        orientation = 0.5 * atan2( 2 * mu11 , ( mu20 - mu02 ) );

        cv::line(binary, cv::Point(x, y),
                cv::Point(x + cos(orientation)*200, y + sin(orientation)*200),
                cv::Scalar(120), 4, CV_FILLED);

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

        size /= (binary.rows * binary.cols);

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
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_detection_node");
  ros::NodeHandle nh("~");

  std::string image_topic = "/camera/rgb/image_color";
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
