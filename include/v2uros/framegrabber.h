/*
 * framegrabber.h
 *
 * Created on: 6/18/2015
 * Author: danying
 */


#ifndef FRAMEGRABBER_H
#define FRAMEGRABBER_H


#include "frmgrab.h"
#include "v2u_lib.h"

#include <cstdio>
#include <iostream>
#include <QMutex>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


class FrameGrabber{

private:
    FrmGrabber* iGrabber;
    V2U_GrabFrame2* frame;
    QMutex iMutex;

    ros::NodeHandle pnh;
    image_transport::ImageTransport * it;
    image_transport::Publisher pub;

public:
    FrameGrabber(ros::NodeHandle & nh);
    ~FrameGrabber();

    bool open();
    void close();
    void start();
    void stop();
    void release();

    bool detectVideoMode(int & height, int & width, double & freq);
    void grabFrame(V2U_UINT32 aFormat, const V2URect* aCrop);
    void writeFrame(std::string fname);

    void convertFrame(cv::Mat & image);
    void publishFrame();

};


#endif // FRAMEGRABBER_H
