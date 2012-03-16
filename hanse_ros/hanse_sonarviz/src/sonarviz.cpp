#include <ros/callback_queue.h>
#include "angles/angles.h"
#include "sonarviz.h"


SonarViz::SonarViz(ros::NodeHandle handle) :
    nh(handle),
    publisher(handle.advertise<sensor_msgs::Image>("sonar/scan/viz", 1)),
    subscriber(handle.subscribe("sonar/scan", 10, &SonarViz::callback, this)),
    lastHeadPosition(0),
    newData(false)
{
    dataImage = Cairo::ImageSurface::create(Cairo::FORMAT_RGB24, imageSize, imageSize);

    Cairo::RefPtr<Cairo::Context> c = Cairo::Context::create(dataImage);

    c->save(); // save the state of the context
    c->set_source_rgb(1.0, 1.0, 1.0);
    c->paint(); // fill image with the color
    c->restore(); // color is back to black now
    lastImageTime = ros::Time::now();

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
    newData = true;
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
    if (!newData) {
	return;
    }

    // update data image

    Cairo::RefPtr<Cairo::ImageSurface> surface =
        Cairo::ImageSurface::create(Cairo::FORMAT_RGB24, imageSize, imageSize);

    Cairo::RefPtr<Cairo::Context> c = Cairo::Context::create(dataImage);
    Cairo::RefPtr<Cairo::Context> cs = Cairo::Context::create(surface);

    cs->save();
    cs->set_source_rgb(0,0,0);
    cs->paint();
    cs->restore();

    c->save();
    c->scale(imageSize / 2.0, imageSize / 2.0);
    c->translate(1, 1);

    cs->save();
    cs->scale(imageSize / 2.0, imageSize / 2.0);
    cs->translate(1, 1);


    auto endIterator = sonarDataMap.end();
    endIterator--;
    ros::Time lastTime = endIterator->second.header.stamp;// - 2 * M_PI;
    float lastPos = endIterator->first - 2 * M_PI;

    cs->set_antialias(Cairo::ANTIALIAS_NONE);
    c->set_antialias(Cairo::ANTIALIAS_NONE);

    const int decimate = 1;

    ros::Time newImageTime;

    for (auto msg : sonarDataMap) {

        int dataPoints = msg.second.echoData.size();

        float x1 = cos(msg.first);
        float x2 = cos(lastPos);
        float y1 = sin(msg.first);
        float y2 = sin(lastPos);

	ros::Time msgTime = msg.second.header.stamp;

	bool draw = msg.first - lastPos < M_PI / 4;
	lastPos = msg.first;
	lastTime = msgTime;

        if (draw) {
	    float d = (lastMsgTime - msgTime).toSec();
	    float f = 0.5 * exp(-2 * d);
	    cs->save();
	    cs->set_source_rgb(0, 0, f);
	    cs->begin_new_path();
	    cs->move_to(0, 0);
	    cs->line_to(x1, y1);
	    cs->line_to(x2, y2);
	    cs->fill();
	    cs->restore();

	    if (msgTime < lastImageTime)
		continue;
	    if (msgTime > newImageTime)
		newImageTime = msgTime;
	    c->save();
	    for (int i = 0; i < dataPoints; i += decimate) {
		int k;
		unsigned char data = 0;
		for (k = 0; i + k < dataPoints && k < decimate; k++) {
		    data = std::max(data, msg.second.echoData[i+k]);
		}
                c->set_source_rgb(0, data / 255.0, 0);
                c->begin_new_path();
                float r1 = (float)i / dataPoints;
                float r2 = (float)(i+k) / dataPoints;

                c->move_to(r1 * x1, r1 * y1);
                c->line_to(r1 * x2, r1 * y2);
                c->line_to(r2 * x2, r2 * y2);
                c->line_to(r2 * x1, r2 * y1);
                c->fill();
            }
	    c->restore();
        }
    }

    lastImageTime = newImageTime;


    cs->set_antialias(Cairo::ANTIALIAS_DEFAULT);

    cs->set_source_rgba(1, 1, 1, 0.5);
    cs->set_line_width(2);
    cs->begin_new_path();
    cs->move_to(-1, 0);
    cs->line_to(1, 0);
    cs->move_to(0, -1);
    cs->line_to(0, 1);

    cs->scale(2.0 / imageSize, 2.0 / imageSize);

    cs->stroke();

    cs->restore();

    cs->save();
    cs->set_source(dataImage, 0, 0);
    cs->set_operator(Cairo::OPERATOR_ADD);
    cs->paint();


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
	ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(1));
	sonarViz.tick();
    }
}
