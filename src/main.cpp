#include <ros/ros.h>
#include <iostream>
#include <cstdio>


#include "v2uros/framegrabber.h"


int main(int argc, char** argv) {

ROS_INFO("Hello world!");
ROS_INFO("Started template C++ node: test_node.");

ros::init(argc, argv, "test_node");
ros::NodeHandle nh;


FrameGrabber fg(nh);

//cv::Mat image;

fg.open();

int w,h;
double f;
fg.detectVideoMode(h,w,f);
ros::Rate grabrate(f);

fg.start();

static int count = 0;

while(ros::ok())
{
  count ++;
  ROS_INFO("test_node: main loop %d", count);

//  std::stringstream fname;
//  fname<<"frame"<<count<<"BGR24.txt";

  fg.grabFrame(V2U_GRABFRAME_FORMAT_BGR24,NULL);
//  fg.writeFrame(fname.str());
//  fg.convertFrame(image);
  fg.publishFrame();

//  cv::waitKey(0);

  ros::spinOnce();
//  ros::Duration(f).sleep();
  grabrate.sleep();

//  if(count==100)
//  {
//      fg.release();
//      break;
//  }
}

fg.release();
fg.stop();


return 0;
}

