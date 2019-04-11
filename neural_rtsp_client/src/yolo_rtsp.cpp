#include <iostream>
#include <iomanip> 
#include <string>
#include <vector>
#include <fstream>

#define OPENCV
#define GPU
#define PARK_KEY 112

#include "yolo_v2_class.hpp"    // imported functions from DLL
#include "Client.h"
#include <opencv2/opencv.hpp>            // C++
#include "opencv2/core/version.hpp"
#include "opencv2/videoio/videoio.hpp"

#include "vlc/vlc.h"

int DONE = 0;
libvlc_media_player_t *mp;
unsigned int videoBufferSize = 0;
uint8_t *videoBuffer = 0;

void draw_boxes(cv::Mat mat_img, std::vector<bbox_t> result_vec, std::vector<std::string> obj_names, Client &client)
{
    int const colors[6][3] = { { 1,0,1 },{ 0,0,1 },{ 0,1,1 },{ 0,1,0 },{ 1,1,0 },{ 1,0,0 } };
    bool detected_stop = false;
    for (auto &i : result_vec) {
        std::string obj_name = obj_names[i.obj_id];
        if (obj_name == "car" || obj_name == "stop sign")
        {
            cv::Scalar color = obj_id_to_color(i.obj_id);
            cv::rectangle(mat_img, cv::Rect(i.x, i.y, i.w, i.h), color, 2);
            if (i.track_id > 0) 
            {
                obj_name += " - " + std::to_string(i.track_id);
            }
            cv::Size const text_size = getTextSize(obj_name, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, 2, 0);
            int const max_width = (static_cast<unsigned int>(text_size.width) > i.w + 2) ? text_size.width : (i.w + 2);
            cv::rectangle(mat_img, cv::Point2f(std::max((int)i.x - 1, 0), std::max((int)i.y - 30, 0)),
                cv::Point2f(std::min((int)i.x + max_width, mat_img.cols - 1), std::min((int)i.y, mat_img.rows - 1)),
                color, CV_FILLED, 8, 0);
            putText(mat_img, obj_name, cv::Point2f(i.x, i.y - 10), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, cv::Scalar(0, 0, 0), 2);
            if (obj_name == "stop sign") 
            {
                detected_stop = true;
            }
        }
    }
    if (detected_stop) 
    {
        client.send("stop");
    }
    else 
    {
        client.send("go");
    }
}

std::vector<std::string> objects_names_from_file(std::string const filename) {
    std::ifstream file(filename);
    std::vector<std::string> file_lines;
    if (!file.is_open()) 
    {
        return file_lines;
    }
    for (std::string line; getline(file, line);) 
    {
        file_lines.push_back(line); 
    }
    std::cout << "object names loaded \n";
    return file_lines;
}

struct VideoDataStruct
{
    int param;
};

void cbVideoPrerender(void *p_video_data, uint8_t **pp_pixel_buffer, int size)
{
    if (static_cast<unsigned int>(size) > videoBufferSize || !videoBuffer)
    {
        printf("Reallocate raw video buffer %d bytes\n", size);
        free(videoBuffer);
        videoBuffer = (uint8_t *)malloc(size);
        videoBufferSize = size;
    }

    // videoBuffer = (uint8_t *)malloc(size);
    *pp_pixel_buffer = videoBuffer;
}
void cbVideoPostrender(void *p_video_data, uint8_t *p_pixel_buffer, int width, int height, int pixel_pitch, int size, int64_t pts)
{
    // Unlocking
    //CloseHandle(hMutex);
}

static void handleEvent(const libvlc_event_t* pEvt, void* pUserData)
{
    libvlc_time_t time;
    switch (pEvt->type)
    {
    case libvlc_MediaPlayerTimeChanged:
        time = libvlc_media_player_get_time(mp);
        printf("MediaPlayerTimeChanged %lld ms\n", (long long)time);
        break;
    case libvlc_MediaPlayerEndReached:
        printf("MediaPlayerEndReached\n");
        DONE = 1;
        break;
    default:
        printf("%s\n", libvlc_event_type_name(pEvt->type));
    }
}

int main(int argc, char *argv[])
{
    std::string  names_file = "../res/yolov3.txt";
    std::string  cfg_file = "../res/yolov3.cfg";
    std::string  weights_file = "../res/yolov3.weights";

    // RTSP address
    const char* rtspAddress = "rtsp://192.168.43.242:8558/usb1";

    // Video resolution WxH
    cv::Size rtspRes(640, 480);

    if (argc > 4) {    //voc.names yolo-voc.cfg yolo-voc.weights    
        names_file = argv[1];
        cfg_file = argv[2];
        weights_file = argv[3];
    }

    float const thresh = 0.5;

    Detector detector(cfg_file, weights_file);

    auto obj_names = objects_names_from_file(names_file);

    // VLC pointers 
    libvlc_instance_t *inst;
    libvlc_media_t *m;
    void *pUserData = 0;

    VideoDataStruct dataStruct;

    // VLC options
    char smem_options[1000];

    // RV24
    sprintf_s(smem_options
        , "#transcode{vcodec=RV24}:smem{"
        "video-prerender-callback=%lld,"
        "video-postrender-callback=%lld,"
        "video-data=%lld,"
        "no-time-sync},"
        , (long long int)(intptr_t)(void*)&cbVideoPrerender
        , (long long int)(intptr_t)(void*)&cbVideoPostrender
        , (long long int)(intptr_t)(void*)&dataStruct
    );

    const char * const vlc_args[] = {
        "-I", "dummy",            // Don't use any interface
        "--ignore-config",        // Don't use VLC's config
        "--extraintf=logger",     // Log anything
        "--verbose=1",            // Be verbose
        "--sout", smem_options    // Stream to memory
    };

    // Launch VLC
    inst = libvlc_new(sizeof(vlc_args) / sizeof(vlc_args[0]), vlc_args);

    // Create a new item
    m = libvlc_media_new_location(inst, rtspAddress);

    // Create a media player playing environement
    mp = libvlc_media_player_new_from_media(m);

    libvlc_event_manager_t* eventManager = libvlc_media_player_event_manager(mp);
    libvlc_event_attach(eventManager, libvlc_MediaPlayerTimeChanged, handleEvent, pUserData);
    libvlc_event_attach(eventManager, libvlc_MediaPlayerEndReached, handleEvent, pUserData);
    libvlc_event_attach(eventManager, libvlc_MediaPlayerPositionChanged, handleEvent, pUserData);

    //libvlc_video_set_format(mp, "RV24", 240, 320, 240 * 3 );

    // play the media_player
    libvlc_media_player_play(mp);

    // Create a window for displaying the video
    std::string winName("RTSP Stream");
    cv::namedWindow(winName, cv::WINDOW_AUTOSIZE);
    cv::Mat frame;
    int key = 0;
    //cv::VideoCapture cap(0);
    // Endless loop, press Esc to quit
    Client client("192.168.43.242");
    while (key != 27)
    {
        // Check for invalid input
        if (videoBuffer)
        {
            // CV_8UC3 = 8 bits, 3 chanels
            frame = cv::Mat(rtspRes, CV_8UC3, videoBuffer);
        }

        if (frame.rows == 0 || frame.cols == 0)
        {
            continue;
        }
        std::vector<bbox_t> result_vec = detector.detect(frame);
        draw_boxes(frame, result_vec, obj_names,client);
        cv::imshow(winName, frame);
        key = cv::waitKey(33);
        if (key == PARK_KEY) 
        {
            client.send("park");
        }
    }
    libvlc_release(inst);

    return 0;
}