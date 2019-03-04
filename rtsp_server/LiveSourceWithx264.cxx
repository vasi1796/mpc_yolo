/*===============================================================================

  Project: H264LiveStreamer
  Module: LiveSourceWithx264.cxx

  Copyright (c) 2014-2015, Rafael Palomar <rafaelpalomaravalos@gmail.com>

  Permission to use, copy, modify, and/or distribute this software for any
  purpose with or without fee is hereby granted, provided that the above
  copyright notice and this permission notice appear in all copies.

  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
  WHATSOEVER RESULTING FROM LOSS OF U..SE, DATA OR PROFITS, WHETHER IN AN
  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

  =============================================================================*/

#include "LiveSourceWithx264.h"

LiveSourceWithx264* LiveSourceWithx264::createNew(UsageEnvironment& env)
{
    return new LiveSourceWithx264(env);
}

EventTriggerId LiveSourceWithx264::eventTriggerId = 0;

unsigned LiveSourceWithx264::referenceCount = 0;

LiveSourceWithx264::LiveSourceWithx264(UsageEnvironment& env):FramedSource(env),
                                                              m_acceptor(boost::asio::ip::tcp::acceptor(m_io_service, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), 3200)))
{
    ++referenceCount;
    //videoCaptureDevice.open(CV_CAP_OPENNI2);
    //videoCaptureDevice.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
    //videoCaptureDevice.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
	//videoCaptureDevice.set(CV_CAP_PROP_AUTO_EXPOSURE, 0.0);
    //videoCaptureDevice.set(CV_CAP_PROP_AUTOFOCUS,1);
    encoder = new x264Encoder();
    encoder->initilize();
	
	double fps = videoCaptureDevice.get(CV_CAP_PROP_FPS);
	std::cout << "Frames per second: " << fps << std::endl;
	
    if(encoder == NULL)
    {
        std::cout << "FAILED TO INITIALIZE ENCODER!" << std::endl;
        exit(1);
    }

    if(eventTriggerId == 0)
    {
        eventTriggerId = envir().taskScheduler().createEventTrigger(deliverFrame0);
    }
}

LiveSourceWithx264::~LiveSourceWithx264(void)
{
    --referenceCount;
    videoCaptureDevice.release();
    encoder->unInitilize();
    envir().taskScheduler().deleteEventTrigger(eventTriggerId);
    eventTriggerId = 0;
}

void LiveSourceWithx264::getUSBCameraFrame()
{
    videoCaptureDevice >> rawImage;
    //cv::resize(rawImage,rawImage,cv::Size(1280,1024));
}

void LiveSourceWithx264::getKinectCameraFrame()
{
    videoCaptureDevice.grab();
    videoCaptureDevice.retrieve(rawImage, CV_CAP_OPENNI_BGR_IMAGE);
    //double frame scaling for kinect
    cv::resize(rawImage, rawImage, cv::Size(), 1.5, 1.5);
}
void LiveSourceWithx264::getTCPFrame()
{
    boost::asio::ip::tcp::socket socket(m_io_service);
    m_acceptor.accept(socket);
    boost::system::error_code error;
    size_t len = boost::asio::read(socket, boost::asio::buffer(m_buf), error);

    //std::cout << "get data length :" << len << std::endl; /* disp the data size recieved */
    cv::Mat mat(310,640,CV_8UC3,m_buf.data());
    rawImage=std::move(mat);
    cv::resize(rawImage, rawImage, cv::Size(), 2, 2);
    //std::cout << "reshape over" << std::endl;
}
void LiveSourceWithx264::encodeNewFrame()
{
    rawImage.data = NULL;
    while(rawImage.data == NULL)
    {
        //getUSBCameraFrame();
        //getKinectCameraFrame();
        getTCPFrame();
    }

    // Got new image to stream
    assert(rawImage.data != NULL);
    encoder->encodeFrame(rawImage);

    // Take all nals from encoder output queue to our input queue
    while(encoder->isNalsAvailableInOutputQueue())
    {
        x264_nal_t nal = encoder->getNalUnit();
        nalQueue.push(nal);
    }
}

void LiveSourceWithx264::deliverFrame0(void* clientData)
{
    ((LiveSourceWithx264*)clientData)->deliverFrame();
}

void LiveSourceWithx264::doGetNextFrame()
{
    if(nalQueue.empty())
    {
        encodeNewFrame();
        gettimeofday(&currentTime, NULL);
        deliverFrame();
    }
    else
    {
        deliverFrame();
    }
}

void LiveSourceWithx264::deliverFrame()
{
    if(!isCurrentlyAwaitingData()) return;

    x264_nal_t nal = nalQueue.front();
    nalQueue.pop();
    assert(nal.p_payload != NULL);

    int trancate = 0;
    if (nal.i_payload >= 4 && nal.p_payload[0] == 0 && nal.p_payload[1] == 0 && nal.p_payload[2] == 0 && nal.p_payload[3] == 1)
    {
        trancate = 4;
    }
    else
    {
        if(nal.i_payload >= 3 && nal.p_payload[0] == 0 && nal.p_payload[1] == 0 && nal.p_payload[2] == 1)
        {
            trancate = 3;
        }
    }

    if(nal.i_payload-trancate > fMaxSize)
    {
        fFrameSize = fMaxSize;
        fNumTruncatedBytes = nal.i_payload-trancate - fMaxSize;
    }
    else
    {
        fFrameSize = (unsigned int) (nal.i_payload - trancate);
    }

    fPresentationTime = currentTime;
    memcpy(fTo, nal.p_payload+trancate, fFrameSize);
    FramedSource::afterGetting(this);
}