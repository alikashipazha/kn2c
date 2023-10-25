#include <stdio.h>
#include <stdlib.h>
#include </home/kn2c/MAVLINK/c_library_v2-master/minimal/mavlink.h>
#include </home/kn2c/MAVLINK/c_library_v2-master/common/common.h>
#include <iostream>
#include <opencv2/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <thread>
#include <mutex>
#include <algorithm>
#include <vector>
#include <zbar.h>
#include <fstream>
#include <fcntl.h>  
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <bcm2835.h>

using namespace cv;
using namespace std;
using namespace zbar; 

int main(){
    serialPort = open("\\\\.\\COM5",   O_RDWR); //flag
    if(serialPort == -1)  {
        cerr << "serialPort_init: Unable to open port" << endl;
        return -1;
    } else if(DEBUG_TXT) cout << "serialPort is open" << endl;

    while(1){
        unint8_t bufRead[MAVLINK_MAX_PACKET_LEN];
        mavlink_message_t msgRead;
        mavlink_status_t statusRead;
        int QUAD_alt, QUAD_yaw;
        for(int i = 0; i < MAVLINK_MAX_PACKET_LEN; i++) {
            read(serialPort, &bufRead, 1);
            if(mavlink_parse_char(MAVLINK_COMM_0, bufRead, &msgRead, &statusRead)){
                switch (msgRead.msgid) {
                    case MAVLINK_MSG_ID_ATTITUDE:
                        mavlink_msg_attitude_decode(&msgRead, &attitude);
                        QUAD_yaw = attitude.yaw;
                        break;
                    case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
                        mavlink_msg_local_position_ned_decode(&msgRead, &local_pos_ned);
                        QUAD_alt = local_pos_ned.z;
                        break;
                }
            }
        }
    }
}