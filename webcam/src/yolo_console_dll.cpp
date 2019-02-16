#include <iostream>
#include <iomanip> 
#include <string>
#include <vector>
#include <queue>
#include <fstream>
#include <thread>
#include <atomic>
#include <mutex>              // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable

#ifdef _WIN32
#define OPENCV
#define GPU
#endif


#include "yolo_v2_class.hpp"    // imported functions from DLL

#include <opencv2/opencv.hpp>            // C++
#include "opencv2/core/version.hpp"
#ifndef CV_VERSION_EPOCH
#include "opencv2/videoio/videoio.hpp"

#include "vlc/vlc.h"

class extrapolate_coords_t {
public:
    std::vector<bbox_t> old_result_vec;
    std::vector<float> dx_vec, dy_vec, time_vec;
    std::vector<float> old_dx_vec, old_dy_vec;

    void new_result(std::vector<bbox_t> new_result_vec, float new_time) {
        old_dx_vec = dx_vec;
        old_dy_vec = dy_vec;
        if (old_dx_vec.size() != old_result_vec.size()) std::cout << "old_dx != old_res \n";
        dx_vec = std::vector<float>(new_result_vec.size(), 0);
        dy_vec = std::vector<float>(new_result_vec.size(), 0);
        update_result(new_result_vec, new_time, false);
        old_result_vec = new_result_vec;
        time_vec = std::vector<float>(new_result_vec.size(), new_time);
    }

    void update_result(std::vector<bbox_t> new_result_vec, float new_time, bool update = true) {
        for (size_t i = 0; i < new_result_vec.size(); ++i) {
            for (size_t k = 0; k < old_result_vec.size(); ++k) {
                if (old_result_vec[k].track_id == new_result_vec[i].track_id && old_result_vec[k].obj_id == new_result_vec[i].obj_id) {
                    float const delta_time = new_time - time_vec[k];
                    if (abs(delta_time) < 1) break;
                    size_t index = (update) ? k : i;
                    float dx = ((float)new_result_vec[i].x - (float)old_result_vec[k].x) / delta_time;
                    float dy = ((float)new_result_vec[i].y - (float)old_result_vec[k].y) / delta_time;
                    float old_dx = dx, old_dy = dy;

                    // if it's shaking
                    if (update) {
                        if (dx * dx_vec[i] < 0) dx = dx / 2;
                        if (dy * dy_vec[i] < 0) dy = dy / 2;
                    } else {
                        if (dx * old_dx_vec[k] < 0) dx = dx / 2;
                        if (dy * old_dy_vec[k] < 0) dy = dy / 2;
                    }
                    dx_vec[index] = dx;
                    dy_vec[index] = dy;

                    //if (old_dx == dx && old_dy == dy) std::cout << "not shakin \n";
                    //else std::cout << "shakin \n";

                    if (dx_vec[index] > 1000 || dy_vec[index] > 1000) {
                        //std::cout << "!!! bad dx or dy, dx = " << dx_vec[index] << ", dy = " << dy_vec[index] << 
                        //    ", delta_time = " << delta_time << ", update = " << update << std::endl;
                        dx_vec[index] = 0;
                        dy_vec[index] = 0;                        
                    }
                    old_result_vec[k].x = new_result_vec[i].x;
                    old_result_vec[k].y = new_result_vec[i].y;
                    time_vec[k] = new_time;
                    break;
                }
            }
        }
    }

    std::vector<bbox_t> predict(float cur_time) {
        std::vector<bbox_t> result_vec = old_result_vec;
        for (size_t i = 0; i < old_result_vec.size(); ++i) {
            float const delta_time = cur_time - time_vec[i];
            auto &bbox = result_vec[i];
            float new_x = (float) bbox.x + dx_vec[i] * delta_time;
            float new_y = (float) bbox.y + dy_vec[i] * delta_time;
            if (new_x > 0) bbox.x = new_x;
            else bbox.x = 0;
            if (new_y > 0) bbox.y = new_y;
            else bbox.y = 0;
        }
        return result_vec;
    }

};


void draw_boxes(cv::Mat mat_img, std::vector<bbox_t> result_vec, std::vector<std::string> obj_names, 
    int current_det_fps = -1, int current_cap_fps = -1)
{
    int const colors[6][3] = { { 1,0,1 },{ 0,0,1 },{ 0,1,1 },{ 0,1,0 },{ 1,1,0 },{ 1,0,0 } };

    for (auto &i : result_vec) {
        cv::Scalar color = obj_id_to_color(i.obj_id);
        cv::rectangle(mat_img, cv::Rect(i.x, i.y, i.w, i.h), color, 2);
        if (obj_names.size() > i.obj_id) {
            std::string obj_name = obj_names[i.obj_id];
            if (i.track_id > 0) obj_name += " - " + std::to_string(i.track_id);
            cv::Size const text_size = getTextSize(obj_name, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, 2, 0);
            int const max_width = (text_size.width > i.w + 2) ? text_size.width : (i.w + 2);
            cv::rectangle(mat_img, cv::Point2f(std::max((int)i.x - 1, 0), std::max((int)i.y - 30, 0)), 
                cv::Point2f(std::min((int)i.x + max_width, mat_img.cols-1), std::min((int)i.y, mat_img.rows-1)), 
                color, CV_FILLED, 8, 0);
            putText(mat_img, obj_name, cv::Point2f(i.x, i.y - 10), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, cv::Scalar(0, 0, 0), 2);
        }
    }
    if (current_det_fps >= 0 && current_cap_fps >= 0) {
        std::string fps_str = "FPS detection: " + std::to_string(current_det_fps) + "   FPS capture: " + std::to_string(current_cap_fps);
        putText(mat_img, fps_str, cv::Point2f(10, 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, cv::Scalar(50, 255, 0), 2);
    }
}
#endif    // OPENCV


void show_console_result(std::vector<bbox_t> const result_vec, std::vector<std::string> const obj_names) {
    for (auto &i : result_vec) {
        if (obj_names.size() > i.obj_id) std::cout << obj_names[i.obj_id] << " - ";
        std::cout << "obj_id = " << i.obj_id << ",  x = " << i.x << ", y = " << i.y 
            << ", w = " << i.w << ", h = " << i.h
            << std::setprecision(3) << ", prob = " << i.prob << std::endl;
    }
}

std::vector<std::string> objects_names_from_file(std::string const filename) {
    std::ifstream file(filename);
    std::vector<std::string> file_lines;
    if (!file.is_open()) return file_lines;
    for(std::string line; getline(file, line);) file_lines.push_back(line);
    std::cout << "object names loaded \n";
    return file_lines;
}

// RTSP address
const char* rtspAddress = "rtsp://172.20.10.5:8558/usb1";
// Video resolution WxH
cv::Size rtspRes(960, 720);

struct VideoDataStruct
{
    int param;
};

int done = 0;
libvlc_media_player_t *mp;
unsigned int videoBufferSize = 0;
uint8_t *videoBuffer = 0;

void cbVideoPrerender(void *p_video_data, uint8_t **pp_pixel_buffer, int size)
{
    if (size > videoBufferSize || !videoBuffer)
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
        done = 1;
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
    std::string filename;

    if (argc > 4) {    //voc.names yolo-voc.cfg yolo-voc.weights test.mp4        
        names_file = argv[1];
        cfg_file = argv[2];
        weights_file = argv[3];
        filename = argv[4];
    }

    float const thresh = (argc > 5) ? std::stof(argv[5]) : 0.20;
    
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
    sprintf(smem_options
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
    std::string winName("Demo Video");
    cv::namedWindow(winName, cv::WINDOW_AUTOSIZE);

    cv::Mat frame;
    int key = 0;

    // Endless loop, press Esc to quit
    while (key != 27)
    {
        // Check for invalid input
        if (videoBuffer)
        {
            // CV_8UC3 = 8 bits, 3 chanels
            frame = cv::Mat(rtspRes, CV_8UC3, videoBuffer);
        }

        if (frame.rows == 0 || frame.cols == 0)
            continue;
        std::vector<bbox_t> result_vec = detector.detect(frame);
        draw_boxes(frame, result_vec, obj_names);
        cv::imshow(winName, frame);
        key = cv::waitKey(33);
    }
    libvlc_release(inst);

    return 0;
}