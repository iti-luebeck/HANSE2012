#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <unicap.h>
#include <vector>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#define NUM_SERVICE_LOOPS 8

class TisDriver {
    static const int bufferCount = 10;
    ros::NodeHandle nh;
    unicap_device_t camDevice;
    unicap_handle_t cam;
    unicap_device_t controlDevice;
    unicap_handle_t control;
    unicap_data_buffer_t buffers[bufferCount];
public:
    TisDriver();

    void openDevices();

    void checkSuccess(int status) {
        if (!SUCCESS(status)) {
            ROS_ERROR("unicap error");
            exit(1);
        }
    }

    void selectVideoFormat();

    void findProperties(bool useControl);

    void setupCapture();

    void tick();

    //ros::Publisher pub_image;

   /* void new_frame_cb (unicap_event_t event, unicap_handle_t handle,
                  unicap_data_buffer_t * buffer, void *usr_data);*/
private:
    // Node Handle.
    ros::NodeHandle node;
    //ros::Publisher pub_image;
};
