#include "sonarviz.h"
#include "angles/angles.h"


SonarViz::SonarViz(ros::NodeHandle handle) :
    nh(handle),
    publisher(handle.advertise<sensor_msgs::Image>("sonar/scan/viz", 1)),
    subscriber(handle.subscribe("sonar/scan", 1, &SonarViz::callback, this)),
    lastHeadPosition(0)
{
}

void SonarViz::callback(const hanse_msgs::ScanningSonar &msg)
{
    double posDiff = angles::normalize_angle(lastHeadPosition - msg.headPosition);
    double posLow, posHigh;
    if(posDiff < 0) {
        posLow = lastHeadPosition;
        posHigh = msg.headPosition;
    } else {
        posLow = msg.headPosition;
        posHigh = lastHeadPosition;
    }

    if(posHigh > posLow) {
        sonarDataMap.erase(sonarDataMap.upper_bound(posLow), sonarDataMap.lower_bound(posHigh));
    } else {
        sonarDataMap.erase(sonarDataMap.upper_bound(posLow), sonarDataMap.end());
        sonarDataMap.erase(sonarDataMap.begin(), sonarDataMap.lower_bound(posHigh));
    }

    sonarDataMap.erase(msg.headPosition);
    sonarDataMap.insert(std::make_pair(msg.headPosition, msg));
    lastHeadPosition = msg.headPosition;


    lastMsgTime = msg.header.stamp;
    tick();
}

sensor_msgs::Image SonarViz::cairoToRosImage(Cairo::RefPtr<Cairo::ImageSurface> surface)
{
    sensor_msgs::Image img;
    img.width = surface->get_width();
    img.height = surface->get_height();
    img.step = img.width*3;
    img.encoding = "rgb8";
    img.is_bigendian = 0;

    unsigned char* data = surface->get_data();
    int stride = surface->get_stride();
    for (unsigned y=0; y<img.height; y++) {
        for (unsigned x=0; x<img.width; x++) {
            unsigned char* p = data + 4*x + stride*y;
            img.data.push_back(p[2]);
            img.data.push_back(p[1]);
            img.data.push_back(p[0]);
        }
    }
    return img;
}

void SonarViz::tick()
{
    Cairo::RefPtr<Cairo::ImageSurface> surface =
        Cairo::ImageSurface::create(Cairo::FORMAT_RGB24, 400, 400);
    Cairo::RefPtr<Cairo::Context> cr = Cairo::Context::create(surface);


    cr->save(); // save the state of the context
    cr->set_source_rgb(1.0, 1.0, 1.0);
    cr->paint(); // fill image with the color
    cr->restore(); // color is back to black now

    cr->save();

    cr->scale(surface->get_width() / 2.0, surface->get_height() / 2.0);
    cr->translate(1, 1);


    cr->save();
    cr->set_source_rgb(0, 0, 0);
    cr->begin_new_path();
    cr->arc(0, 0, 1, 0, 2 * M_PI);
    cr->fill();
    cr->restore();
    
    auto endIterator = sonarDataMap.end();
    endIterator--;
    double lastPos = endIterator->first - 2 * M_PI;

    cr->set_antialias(Cairo::ANTIALIAS_NONE);

    for (auto msg : sonarDataMap) {
        cr->save();

        int dataPoints = msg.second.echoData.size();

        double x1 = cos(msg.first);
        double x2 = cos(lastPos);
        double y1 = sin(msg.first);
        double y2 = sin(lastPos);

        if (msg.first - lastPos < M_PI / 2) {
            for (int i = 0; i < dataPoints; i++) {
		ros::Time msgTime = msg.second.header.stamp;
		double d = (lastMsgTime - msgTime).toSec();
		double f = 0.5 * exp(-2 * d);
                cr->set_source_rgb(0, msg.second.echoData[i] / 255.0, f);
                cr->begin_new_path();
                double r1 = (double)i / dataPoints;
                double r2 = (double)(i + 1) / dataPoints;

                cr->move_to(r1 * x1, r1 * y1);
                cr->line_to(r1 * x2, r1 * y2);
                cr->line_to(r2 * x2, r2 * y2);
                cr->line_to(r2 * x1, r2 * y1);
                cr->fill();
            }
        }

        lastPos = msg.first;
        cr->restore();
    }

    cr->set_antialias(Cairo::ANTIALIAS_DEFAULT);

    cr->set_source_rgba(1, 1, 1, 0.5);
    cr->set_line_width(1);
    cr->begin_new_path();
    cr->move_to(-1, 0);
    cr->line_to(1, 0);
    cr->move_to(0, -1);
    cr->line_to(0, 1);

    cr->scale(2.0 / surface->get_width(), 2.0 / surface->get_height());

    cr->stroke();

    cr->restore();

    sensor_msgs::Image img = cairoToRosImage(surface);
    publisher.publish(img);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sonar_viz");
    ros::NodeHandle n;
    //ros::Rate r(1);

    SonarViz sonarViz(n);

    while (ros::ok()) {
        //sonarViz.tick();
        //r.sleep();
        ros::spinOnce();
    }
}
