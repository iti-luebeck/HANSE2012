#include "sonarviz.h"
#include "angles/angles.h"


SonarViz::SonarViz(ros::NodeHandle handle) :
  nh(handle),
  publisher(handle.advertise<sensor_msgs::Image>("sonarviz", 1)),
  subscriber(handle.subscribe("scanning_sonar", 1, &SonarViz::callback, this)),
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
    sonarDataMap.erase(sonarDataMap.upper_bound(posHigh), sonarDataMap.end());
    sonarDataMap.erase(sonarDataMap.begin(), sonarDataMap.lower_bound(posLow));
  }

  sonarDataMap.insert(std::make_pair(msg.headPosition, msg));
  lastHeadPosition = msg.headPosition;
  tick();

  ROS_INFO("%d", sonarDataMap.size());
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
  for(int y=0; y<img.height; y++)
  {
    for(int x=0; x<img.width; x++)
    {
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
        Cairo::ImageSurface::create(Cairo::FORMAT_RGB24, 600, 400);
  Cairo::RefPtr<Cairo::Context> cr = Cairo::Context::create(surface);

    cr->save(); // save the state of the context
    cr->set_source_rgb(0.86, 0.85, 0.47);
    cr->paint(); // fill image with the color
    cr->restore(); // color is back to black now

    cr->save();
    // draw a border around the image
    cr->set_source_rgb(0, 1, 0);
    cr->set_line_width(20.0); // make the line wider
    cr->rectangle(0.0, 0.0, surface->get_width(), surface->get_height());
    cr->stroke();

    sensor_msgs::Image img = cairoToRosImage(surface);
    publisher.publish(img);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "sonarviz");
  ros::NodeHandle n;
  //ros::Rate r(1);

  SonarViz sonarViz(n);

  while (ros::ok()) {
    //sonarViz.tick();
    //r.sleep();
    ros::spinOnce();
  }
}
