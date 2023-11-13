#include <iostream>
#include <opencv2/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <thread>
#include </home/kn2c/MAVLINK/c_library_v2-master/minimal/mavlink.h>
#include </home/kn2c/MAVLINK/c_library_v2-master/common/common.h>
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

    int fd = open("/dev/ttyACM0", O_RDWR);
    if (fd == -1)  {
        printf("\nserialport_init: Unable to open port =%d",fd);
        return -1;}
    else{printf("\nserialport is open");}

    //make objects(variables)
    Mat old_frame,old_resize, old_gray;
    vector<Point2f> p0, p1; // Point2f d i
    vector<Scalar> colors;
    RNG rng;
    Mat frame,frame_resize, frame_gray;
    vector<uchar> status;
    vector<float> err;
    clock_t start, end;
    float fpslive;
    int num;
    float totalMovementx, dx;
    float totalMovementy, dy;
    int MSG_opf_delay = 0;


    //webcam
    VideoCapture cap0(1);
    //VideoCapture cap0("video2.mp4"); //video

    if (!cap0.isOpened()) {
        cerr << "Failed to open bottom camera." << endl;
        return 0;
    }

    int width = static_cast<int>(cap0.get(3));
    int height = static_cast<int>(cap0.get(4));
    //cout << width <<"\t\t\t\t\t\t\t"<< height << endl;
    int cam_fps = cap0.get(CAP_PROP_FPS);
    //cap0.set(CAP_PROP_FPS, 5);
    cap0 >> old_frame; // Capture a frame
    resize(old_frame, old_resize ,Size() , 0.1, 0.1);
    //pi camera color map NOT
    cvtColor(old_resize, old_resize, COLOR_BGR2RGB);
    cvtColor(old_resize, old_gray, COLOR_BGR2GRAY);
    // cvtColor(old_frame, old_gray, COLOR_BGR2GRAY);
    
    cout <<"bottom camera frame per second: "<<cam_fps<<endl;
    TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);

    while(true){
      
        start = clock();
        cap0 >> frame;
        resize(frame, frame_resize ,Size() , 0.1, 0.1);
        cvtColor(frame_resize, frame_gray, COLOR_BGR2GRAY);
        // cvtColor(frame, frame_gray, COLOR_BGR2GRAY);

        //find feature points in old
        goodFeaturesToTrack(old_gray, p0, 20, 0.3, 7, Mat(), 7, false, 0.04);
        // calculate optical flow
        if(!p0.empty()) calcOpticalFlowPyrLK(old_gray, frame_gray, p0, p1, status, err, Size(30,30), 2, criteria);
        
        //calculate flow quality. calculate mean of vector and print dx and dy
        totalMovementx = 0.0;
        totalMovementy = 0.0;
        num = 0;
        for(uint i = 0; i < p0.size(); i++){
            if(status[i] == 1){
                totalMovementx += (p1[i].x - p0[i].x);
                totalMovementy += (p1[i].y - p0[i].y);
                num++;
            }
        }

        old_gray = frame_gray.clone();
        int flow_quality = round(255.0 * num / status.size());
        //optical_flow_msg.time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
        optical_flow_msg.flow_comp_m_x=0; /*< [m/s] Flow in x-sensor direction, angular-speed compensated*/
        optical_flow_msg.flow_comp_m_y=0; /*< [m/s] Flow in y-sensor direction, angular-speed compensated*/
        optical_flow_msg.ground_distance=-1; /*< [m] Ground distance. Positive value: distance known. Negative value: Unknown distance*/
        optical_flow_msg.flow_x=totalMovementx; /*< [dpix] Flow in x-sensor direction*/
        optical_flow_msg.flow_y=totalMovementy; /*< [dpix] Flow in y-sensor direction*/
        optical_flow_msg.sensor_id=100; /*<  Sensor ID*/
        optical_flow_msg.quality=flow_quality; /*<  Optical flow quality / confidence. 0: bad, 255: maximum quality*/
        //optical_flow_msg.flow_rate_x; /*< [rad/s] Flow rate about X axis*/
        //optical_flow_msg.flow_rate_y; /*< [rad/s] Flow rate about Y axis*/
        
        //calculate live FPS
        end = clock();
        fpslive = float(CLOCKS_PER_SEC)/(float(end)-float(start));
        
        //mavlink massage

        //flag here
        
        cout<<fpslive<<endl;
        imshow("Frame", frame_resize);
        waitKey(100); //only for video !!!!!

        uint8_t len;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];

        mavlink_msg_optical_flow_encode(1,200,&msg,&optical_flow_msg);
        len=mavlink_msg_to_send_buffer(buf, &msg);
        write( fd, &buf ,MAVLINK_MAX_PACKET_LEN);

    } cap0.release();

    return 0;
}