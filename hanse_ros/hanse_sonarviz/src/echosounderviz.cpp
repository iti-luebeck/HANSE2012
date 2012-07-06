#include "echosounderviz.h"
#include "angles/angles.h"


EchoSounderViz::EchoSounderViz(ros::NodeHandle handle) :
    nh(handle),
    publisher(handle.advertise<sensor_msgs::Image>("sonar/echo/viz", 1)),
    subscriber(handle.subscribe("sonar/echo", 1, &EchoSounderViz::callback, this))
{
    for(int i=0; i<252; i++)
        echoSounderMsgs.push_back(hanse_msgs::EchoSounder());
}

void EchoSounderViz::callback(const hanse_msgs::EchoSounder &msg)
{
    echoSounderMsgs.push_back(msg);
    while(echoSounderMsgs.size() > 252)
        echoSounderMsgs.erase(echoSounderMsgs.begin());

    tick();
}

sensor_msgs::Image EchoSounderViz::cairoToRosImage(Cairo::RefPtr<Cairo::ImageSurface> surface)
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

void EchoSounderViz::tick()
{
    Cairo::RefPtr<Cairo::ImageSurface> surface =
        Cairo::ImageSurface::create(Cairo::FORMAT_RGB24, 252, 252);
    Cairo::RefPtr<Cairo::Context> cr = Cairo::Context::create(surface);


    cr->save(); // save the state of the context
    cr->set_source_rgb(1.0, 1.0, 1.0);
    cr->paint(); // fill image with the color
    cr->restore(); // color is back to black now

    cr->save();
    cr->set_source_rgb(1, 1, 1);
    cr->begin_new_path();
    cr->arc(0, 0, 1, 0, 2 * M_PI);
    cr->fill();
    cr->restore();    

    cr->set_antialias(Cairo::ANTIALIAS_NONE);

    for(int x=0; x<252; x++)
    {        
        for(int y=0; y<echoSounderMsgs[x].echoData.size(); y++)
        {
            double val = 1.0 - echoSounderMsgs[x].echoData[y] / 255.0;
            cr->set_source_rgb(val, val, val);
            cr->rectangle(x, y, 1, 1);
            cr->stroke();
        }
    }

    sensor_msgs::Image img = cairoToRosImage(surface);
    publisher.publish(img);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "echosounderviz");
    ros::NodeHandle n;
    //ros::Rate r(1);

    EchoSounderViz echoSounderViz(n);

    while (ros::ok()) {
        //sonarViz.tick();
        //r.sleep();
        ros::spinOnce();
    }
}
