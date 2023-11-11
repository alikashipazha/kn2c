#include <iostream>
#include <opencv2/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <thread>
#include </home/kn2c/mavlink/c_library_v2-master/minimal/mavlink.h>
#include </home/kn2c/mavlink/c_library_v2-master/common/common.h>
#include <fstream>
#include <fcntl.h> 
#include <string>
#include <unistd.h>

using namespace cv;
using namespace std;
mavlink_optical_flow_t optical_flow_msg;
mavlink_message_t msg;
int main(int argc, char **argv)
{

    //make objects(variables)
    Mat old_frame,old_resize, old_gray;
    vector<Point2f> p0, p1; // Point2f d i
    Mat frame,frame_resize, frame_gray;
    vector<uchar> status;
    vector<float> err;
    float totalMovementx, dx;
    float totalMovementy, dy;
    int MSG_opf_delay = 0;
    int num;
    int flow_quality;
    
    
    uint8_t len;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    //calc opt
    
    //fps
    int i=0;
    auto start_time = std::chrono::steady_clock::now();
    auto end_time = start_time + std::chrono::seconds(5);
    float fpslive;
    int fd;
    fd = open("/dev/ttyACM0", O_RDWR);
    if (fd == -1)  {
        printf("\nserialport_init: Unable to open port =%d",fd);
        return -1;}
    else{printf("\nserialport is open");}
    
    //webcam
    VideoCapture cap0(1);
    //VideoCapture cap0("video2.mp4"); //video

    if (!cap0.isOpened()) {
        cerr << "Failed to open bottom camera." << endl;
        return 0;
    }

    int width = static_cast<int>(cap0.get(3));
    int height = static_cast<int>(cap0.get(4));
    cout << width <<"\t\t\t\t\t\t\t"<< height << endl;
    int cam_fps = cap0.get(CAP_PROP_FPS);
    cout <<"bottom camera frame per second: "<<cam_fps<<endl;
    //cap0.set(CAP_PROP_FPS, 80);
    cam_fps = cap0.get(CAP_PROP_FPS);
    cout <<"bottom camera frame per second: "<<cam_fps<<endl;
    cap0 >> old_frame; // Capture a frame
    resize(old_frame, old_resize ,Size() , 0.1, 0.1);
    //pi camera color map NOT
    //cvtColor(old_resize, old_resize, COLOR_BGR2RGB);
    cvtColor(old_resize, old_gray, COLOR_BGR2GRAY);
    // cvtColor(old_frame, old_gray, COLOR_BGR2GRAY);
    TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);

    while(true){
      
        totalMovementx = 0.0;
        totalMovementy = 0.0;
        num = 0;

        cap0 >> frame;
        resize(frame, frame_resize ,Size() , 0.1, 0.1);
        cvtColor(frame_resize, frame_gray, COLOR_BGR2GRAY);
        // cvtColor(frame, frame_gray, COLOR_BGR2GRAY);

        //find feature points in old
        goodFeaturesToTrack(old_gray, p0, 10, 0.3, 7, Mat(), 7, false, 0.04);
        // calculate optical flow
        if(!p0.empty()) {
            calcOpticalFlowPyrLK(old_gray, frame_gray, p0, p1, status, err, Size(10,10), 2, criteria);}
        else{
            continue;
        }
        
        //calculate flow quality. calculate mean of vector and print dx and dy
        for(uint i = 0; i < p0.size(); i++){
            if(status[i] == 1){
                totalMovementx += (p1[i].x - p0[i].x);
                totalMovementy += (p1[i].y - p0[i].y);
                num++;
            }
        }
        flow_quality = round(255.0 * num / status.size());
        
        //mavlink massage
        //optical_flow_msg.time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
        //optical_flow_msg.flow_comp_m_x = 0; /*< [m/s] Flow in x-sensor direction, angular-speed compensated*/
        //optical_flow_msg.flow_comp_m_y = 0; /*< [m/s] Flow in y-sensor direction, angular-speed compensated*/
        optical_flow_msg.ground_distance = -1; /*< [m] Ground distance. Positive value: distance known. Negative value: Unknown distance*/
        optical_flow_msg.flow_x = totalMovementx; /*< [dpix] Flow in x-sensor direction*/
        optical_flow_msg.flow_y = totalMovementy; /*< [dpix] Flow in y-sensor direction*/
        optical_flow_msg.sensor_id = 100; /*<  Sensor ID*/
        optical_flow_msg.quality = flow_quality; /*<  Optical flow quality / confidence. 0: bad, 255: maximum quality*/
        //optical_flow_msg.flow_rate_x; /*< [rad/s] Flow rate about X axis*/
        //optical_flow_msg.flow_rate_y; /*< [rad/s] Flow rate about Y axis*/

        mavlink_msg_optical_flow_encode(1,200,&msg,&optical_flow_msg);
        len=mavlink_msg_to_send_buffer(buf, &msg);
        write( fd, &buf ,MAVLINK_MAX_PACKET_LEN);

        //save last frame as old
        old_gray = frame_gray.clone();

        //calculate live FPS
        i++;
        cout << i << endl;
        if(chrono::steady_clock::now() >= end_time){
            fpslive = i/5;
            start_time = chrono::steady_clock::now();
            end_time = start_time + chrono::seconds(5);
            cout<<"fps = "<<fpslive<<endl;
            i=0;
            }

    }
    
    cap0.release();

    return 0;
}

