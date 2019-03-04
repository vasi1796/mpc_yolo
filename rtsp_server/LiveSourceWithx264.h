/*===============================================================================

  Project: H264LiveStreamer
  Module: LiveSourceWithx264.h

  Copyright (c) 2014-2015, Rafael Palomar <rafaelpalomaravalos@gmail.com>

  Permission to use, copy, modify, and/or distribute this software for any
  purpose with or without fee is hereby granted, provided that the above
  copyright notice and this permission notice appear in all copies.

  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

  =============================================================================*/

#ifndef __LIVE_SOURCE_WITH_X264_H
#define __LIVE_XOURCE_WITH_X264_H

#include <queue>
#include <opencv2/opencv.hpp>
#include <FramedSource.hh>
#include "x264encoder.h"

#include <boost/asio.hpp>
#include <boost/array.hpp>

class LiveSourceWithx264:public FramedSource
{
public:
    static LiveSourceWithx264* createNew(UsageEnvironment& env);
    static EventTriggerId eventTriggerId;
protected:
    LiveSourceWithx264(UsageEnvironment& env);
    virtual ~LiveSourceWithx264(void);
private:
    boost::array<char, 3932160> m_buf;         /* the size of received mat frame is calculated by width*height*no_channels */
    boost::asio::io_service m_io_service;
    boost::asio::ip::tcp::acceptor m_acceptor;

    virtual void doGetNextFrame();
    static void deliverFrame0(void* clientData);
    void deliverFrame();
    void encodeNewFrame();
    void getUSBCameraFrame();
    void getKinectCameraFrame();
    void getTCPFrame();
    static unsigned referenceCount;
    std::queue<x264_nal_t> nalQueue;
    timeval currentTime;
    cv::VideoCapture videoCaptureDevice;
    cv::Mat rawImage;
    x264Encoder *encoder;
};

#endif
