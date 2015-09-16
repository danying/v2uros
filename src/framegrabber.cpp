/*
 * framegrabber.cpp
 *
 * Created on: 6/18/2015
 * Author: danying
 */



#include "v2uros/framegrabber.h"


FrameGrabber::FrameGrabber(ros::NodeHandle & nh):pnh(nh)
{
    FrmGrab_Init();
    iGrabber = NULL;
    frame = NULL;

    it = new image_transport::ImageTransport(pnh);
    pub = it->advertise("camera/image", 30);
}

FrameGrabber::~FrameGrabber()
{
    close();
    FrmGrab_Deinit();
    delete it;
}

bool FrameGrabber::open()
{
    bool isOpen = false;
    iGrabber = FrmGrabLocal_Open();
    if(!iGrabber)
    {
        std::cout<<"grabber open failed"<<std::endl;
        isOpen = false;
    }
    else
        isOpen = true;

    return isOpen;
}

void FrameGrabber::close()
{
    FrmGrab_Close(iGrabber);
}

void FrameGrabber::start()
{
    if(iGrabber)
    {
        QMutexLocker lock(&iMutex);
        FrmGrab_Start(iGrabber);
    }
}

void FrameGrabber::stop()
{
    QMutexLocker lock(&iMutex);
    FrmGrab_Stop(iGrabber);
}

void FrameGrabber::release()
{
    if(frame)
        FrmGrab_Release(iGrabber, frame);
}


bool FrameGrabber::detectVideoMode(int &height, int &width, double &freq)
{
    V2U_VideoMode vm;
    bool result = FrmGrab_DetectVideoMode(iGrabber, &vm);
    if(result)
    {
        std::cout<<"video mode detected: "<<vm.height<<"X"<<vm.width<<" @ "<<vm.vfreq/1000<<std::endl;
        height = vm.height;
        width = vm.width;
        freq = vm.vfreq/1000;
    }
    else
    {
        std::cout<<"video mode cannot be detected! @ FrameGrabber" <<std::endl;
        height = width = 0;
        freq = 0;
    }
    return result;
}


void FrameGrabber::grabFrame(V2U_UINT32 aFormat, const V2URect *aCrop)
{
    if(iGrabber)
    {
        QMutexLocker lock(&iMutex);
        frame = FrmGrab_Frame(iGrabber, aFormat, aCrop);
    }
}

void FrameGrabber::writeFrame(std::string fname)
{
    if(frame)
    {
        FILE * out = fopen(fname.c_str(), "wb");
        bool result = false;

        if(fname[fname.length()-3] == 'b')
        {
            result = v2u_write_bmp(out, frame->crop.width, frame->crop.height,frame->palette, frame->pixbuf);
        }
        else if(fname[fname.length()-3] == 'j' || fname[fname.length()-4] == 'j')
        {
//            result = v2u_write_jpeg(out, frame->crop.width, frame->crop.height,frame->palette, frame->pixbuf);
        }
        else if(fname[fname.length()-3] == 'p')
        {
//            result = v2u_write_png(out, frame->crop.width, frame->crop.height,frame->palette, frame->pixbuf);
        }
        else
        {
            result = (fwrite(frame->pixbuf, 1, frame->imagelen, out) == frame->imagelen);
        }

        if(!result)
        {
            std::cout<<"error writing frame"<<std::endl;
        }

        fclose(out);
    }
    else
    {
        std::cout<<"no frame to write"<<std::endl;
    }
}

void FrameGrabber::convertFrame(cv::Mat & image)
{
    if(frame)
    {
        int h = frame->mode.height;
        int w = frame->mode.width;
        image = cv::Mat(h,w,CV_8UC3,(uchar*)frame->pixbuf,3*w);
    }
}


void FrameGrabber::publishFrame()
{
    cv::Mat image;
    convertFrame(image);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    msg->header.stamp = ros::Time::now();
    pub.publish(msg);
}
