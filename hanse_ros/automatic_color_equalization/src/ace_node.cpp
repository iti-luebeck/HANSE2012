
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include "ace.h"
#include <dynamic_reconfigure/server.h>
#include <automatic_color_equalization/aceConfig.h>

class ACENode {
public:
    ACENode(ros::NodeHandle nh) {
        pub = nh.advertise<sensor_msgs::Image>("/image_ace", 1);

        samples = 200;
        step = 32;
        sigma = 0.4;

        init();
    }

    void init() {
        int colorSpace = ACE_COLORSPACE_YCrCb;
        int sampling = ACE_SAMPLE_GAUSS;
        int saturation = ACE_SATURATE;
        int channels = ACE_CHANNEL_1 | ACE_CHANNEL_2 | ACE_CHANNEL_3;

        ace.init(320, 240, samples, sampling, sigma, step,
                 colorSpace, channels, saturation);
    }

    void configCallback(automatic_color_equalization::aceConfig &config, uint32_t level)
    {
        step = config.step;
        sigma = config.sigma;
        samples = config.samples;

        init();
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

        cv::Mat largeImage = cv_ptr->image;
        cv::Mat image;
        cv::resize(largeImage, image, cv::Size(320,240));
        IplImage *aceImage = new IplImage(image);
        IplImage *doAceImage = cvCreateImage(cvSize(aceImage->width, aceImage->height), aceImage->depth, aceImage->nChannels);
        ace.automaticColorEqualization(aceImage, doAceImage);

        sensor_msgs::Image aceMsg;
        aceMsg.header = visual_img_msg->header;
        cv_ptr->image = cv::Mat(doAceImage);

    //    cv_bridge::CvImagePtr p =  boost::make_shared<cv_bridge::CvImage>();
    //    p->image = gray;
        cv_ptr->toImageMsg(aceMsg);
        pub.publish(aceMsg);

        cvReleaseImage(&doAceImage);
    }

private:
    ros::Publisher pub;
    ACE ace;
    int step;
    double sigma;
    int samples;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ace_node");
  ros::NodeHandle nh("~");

  std::string image_topic = "/hanse/camera/front";
  nh.getParam("image_topic", image_topic);
  std::cout << image_topic << std::endl;

  ACENode aceNode(nh);

  dynamic_reconfigure::Server<automatic_color_equalization::aceConfig> server;
  dynamic_reconfigure::Server<automatic_color_equalization::aceConfig>::CallbackType f;

  f = boost::bind(&ACENode::configCallback, &aceNode, _1, _2);
  server.setCallback(f);

  ros::Subscriber camSub =
          nh.subscribe<sensor_msgs::Image>(image_topic, 1, boost::bind(&ACENode::imageCB, &aceNode, _1));

  ros::spin();
}
