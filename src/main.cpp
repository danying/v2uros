#include <iostream>
#include <cstdio>
#include "v2uros/framegrabber.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vga2usb");
    ros::NodeHandle nh;

    FrameGrabber fg(nh);

    if(!fg.open())
    {
        std::cout<<"Exiting ......\n";
        fg.close();
        return 1;
    }

    int w,h;
    double f;
    if(!fg.detectVideoMode(h,w,f))
    {
        std::cout<<"Exiting ......\n";
        fg.close();
        return 1;
    }

    ros::Rate grabrate(f);
    fg.start();

    while(ros::ok())
    {

        fg.grabFrame(V2U_GRABFRAME_FORMAT_BGR24,NULL);
        fg.publishFrame();
        ros::spinOnce();
        grabrate.sleep();
        fg.release();
    }
    fg.stop();
    fg.close();
    return 0;
}

