#include <ros/ros.h>
#include <unicap.h>
#include <vector>

#define FOURCC(A, B, C, D) ((A) | (B << 8) | (C << 16) | (D << 24))

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
};

TisDriver::TisDriver()
{
    openDevices();
    selectVideoFormat();
    findProperties(false);
    findProperties(true);
    setupCapture();
}

void TisDriver::openDevices()
{
    int devCount = 0;
    bool foundCam = false, foundControl = false;
    while (true) {
        unicap_device_t device;
        int status = unicap_enumerate_devices(NULL, &device, devCount);
        if (SUCCESS(status)) {
            if (strstr(device.identifier, "DFK 22AUC03")) {
                if (strstr(device.identifier, "The Imaging Source")) {
                    controlDevice = device;
                    foundControl = true;
                } else {
                    camDevice = device;
                    foundCam = true;
                }
            }
        } else {
            break;
        }
        devCount++;
    }
    if (!(foundControl && foundCam)) {
        ROS_ERROR("Failed to enumerate the camera devices");
        exit(1);
    }
    checkSuccess(unicap_open(&cam, &camDevice));
    checkSuccess(unicap_open(&control, &controlDevice));
}

void TisDriver::selectVideoFormat()
{
    ROS_INFO("selectVideoFormat()");
    int formatCount = 0;
    bool found = false;
    int width, height;
    width = 744;
    height = 480;
    while (true) {
        unicap_format_t format;
        int status = unicap_enumerate_formats(cam, NULL, &format, formatCount);
        if (SUCCESS(status)) {
            if (format.fourcc == FOURCC('G', 'R', 'E', 'Y')) {
                if (format.size_count == 0 &&
                    format.size.width == width &&
                    format.size.height == height) {
                    found = true;
                    checkSuccess(unicap_set_format(cam, &format));
                } else {
                    for (int i = 0; i < format.size_count; i++) {
                        ROS_INFO("%i %i", format.sizes[i].width, format.sizes[i].height);
                        if (format.sizes[i].width != width ||
                            format.sizes[i].height != height) {
                            continue;
                        }
                        found = true;
                        format.size.width = width;
                        format.size.height = height;
                        checkSuccess(unicap_set_format(cam, &format));
                    }
                }
            } 
        } else {
            break;
        }

        formatCount++;
    }
    if (!found) {
        ROS_ERROR("Failed to find format");
        exit(1);
    }
}

void TisDriver::findProperties(bool useControl)
{
    ROS_INFO("findProperties()");
    unicap_handle_t &h = useControl ? control : cam;
    int propCount = 0;
    while (true) {
        unicap_property_t prop;
        int status = unicap_enumerate_properties(h, NULL, &prop, propCount);
        if (SUCCESS(status)) {
            ROS_INFO("prop: %s",  prop.identifier);
        } else {
            break;
        }
        propCount++;
    }
}

static void
new_frame_cb (unicap_event_t event, unicap_handle_t handle,
              unicap_data_buffer_t * buffer, void *usr_data)
{
    for (int i = 0; i < 1024; i++) {
        printf("%02x ", buffer->data[i]);
    }
    printf("\n\n");
    ROS_INFO("frame!");
}

void TisDriver::setupCapture()
{
    ROS_INFO("setupCapture()");
    unicap_format_t format;
    checkSuccess(unicap_get_format(cam, &format));
    format.buffer_type = UNICAP_BUFFER_TYPE_USER;
    checkSuccess(unicap_set_format(cam, &format));

    for (int i = 0; i < bufferCount; i++) {
        buffers[i].data = new uint8_t[format.buffer_size * 4];
        buffers[i].buffer_size = format.buffer_size * 4;
    };

    unicap_register_callback(cam, UNICAP_EVENT_NEW_FRAME, (unicap_callback_t)new_frame_cb, 0);


    checkSuccess(unicap_start_capture(cam));
    /*
    for (int i = 0; i < bufferCount; i++) {
        checkSuccess(unicap_queue_buffer(cam, &buffers[i]));
        }*/
}

void TisDriver::tick()
{/*
    unicap_data_buffer_t *buffer;
    int count;
    while (true) {
        unicap_poll_buffer(cam, &count);
        ROS_INFO("count %i", count);
    }
    //checkSuccess(unicap_wait_buffer(cam, &buffer));
    ROS_INFO("got frame!");
    checkSuccess(unicap_queue_buffer(cam, buffer));*/
    usleep(100);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tis_driver");

    TisDriver d;

    while (true) {
        d.tick();
    }
    return 0;
}
