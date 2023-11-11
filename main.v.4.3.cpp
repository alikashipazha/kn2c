// TODO list:
    // line blur -
    // landing +
    // red color calibre +
    // line +
    // medianBlur() -
    // scale landline or land line -
    // croping image or slamming center of qr box -
    // remove "black in crop right" -
    // fix black with white -
    //
    // add dy < 0 qr +
    // obs states into forward follow line +
    // if line no continue +
    // decode center -> dir + center +
    // last state +
    // 3 main line codes -
    // // finding center of qr code +
    // follow line and decrease increase alt fix <- thresh_haugh*zarib -----
    // states for geting back to default alt MASMALI
    // // state thread conflicts ++++
    // 
    // scale line vs land +
    // px4 data : compass(MAVLINK_yaw) + alt(z) \ 4th thread? -
    // chnage landing state to mission 4 -
    // land(while ln1000) - 
    // and mission3 +
    // 
    // ifs -> whiles
    // multiple resizes in threads
    // timer in each threa
    // px4 data : compass(MAVLINK_yaw) + alt(z) +
    // camera kaj +
    // no line -> increase alt
    // no alt data above red
    // mission timer
    // underabove del
    // cam jelo -> crop
    //
    // IPS (iter per sec) or timer(sec)

// GUIDANCE:
    //FLAG : to edit(setup day)
    //HERE : to complete/add
    //DEL : to delete
    //NOTE : to avoid bug
    //TEST : to test

// ########## KN2C Libraries #####
    // ##                       ##
    // ##        comment        ##
    // ##                       ##
    // ###########################

#include <stdio.h>
#include <stdlib.h>
#include </home/kn2c/mavlink/c_library_v2-master/minimal/mavlink.h>
#include </home/kn2c/mavlink/c_library_v2-master/common/common.h>
// #include </home/kn2c/vision/HG-PWM-master/pwm.hpp>
// #include </home/kn2c/vision/HG-PWM-master/pwm.cpp>
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
#include <wiringPi.h>
// #include <bcm2835.h>

using namespace cv;
using namespace std;
using namespace zbar; 

// ####### KN2C Consts & Flags ###
    // ##                       ##
    // ##        comment        ##
    // ##                       ##
    // ###########################

//SCALE FACTORS
#define SCALE_OPF 0.1
#define SCALE_OBS 0.2
#define SCALE_line_land_qr 0.3 //HERE

//USLEEP : micro second
#define USLEEP_arm 5000000
#define USLEEP_takeoff 5000000
#define USLEEP_servo 100000

//SERVO
#define SERVO_frequency 50
#define SERVO_pin 18 //12
#define SERVO_rotationRate 10
#define SERVO_PWM_clock_divisor 192
#define SERVO_PWM_range 2000 //NOTE for SERVO_frequency 50?

//MAVLINK

//alt
#define MAVLINK_ALT_land 0.0
#define MAVLINK_ALT_lower 0.1
#define MAVLINK_ALT_low 0.25
#define MAVLINK_ALT_normal 0.5
#define MAVLINK_ALT_high 1.5
#define MAVLINK_ALT_higher 2.0
#define MAVLINK_ALT_highest 2.3
//velocity
#define MAVLINK_VELOCITY_X 0.01
#define MAVLINK_VELOCITY_Y 0.03
#define MAVLINK_VELOCITY_Z 0.02
#define MAVLINK_VELOCITY_zigzagRate 1/4
#define MAVLINK_VELOCITY_Y_forCenterRate 2.0
#define MAVLINK_VELOCITY_SLOW 0.5
#define MAVLINK_VELOCITY_FAST 1.5
//yaw
#define MAVLINK_yawRate 20 //per second
#define MAVLINK_YAW_0 0
#define MAVLINK_YAW_90 CV_PI/2
#define MAVLINK_YAW_180 CV_PI
#define MAVLINK_YAW_270 -CV_PI/2
//msg
#define MAVLINK_MSG_systemID 0xFF
#define MAVLINK_MSG_componentID 0xBE
#define MAVLINK_MSG_opf_iteration_delay 3 //DEL
#define MAVLINK_MSG_arm_typeMASK 400
#define MAVLINK_MSG_takeoff_typeMASK 22
#define MAVLINK_MSG_velocity_typeMASK 3527 
#define MAVLINK_MSG_position_typeMASK 3567
#define MAVLINK_MSG_yaw_typeMASK 2559

//COLOR CALIBRES

//red1
#define COLOR_RED1_lower_h 0
#define COLOR_RED1_upper_h 10
#define COLOR_RED1_lower_s 140
#define COLOR_RED1_upper_s 255
#define COLOR_RED1_lower_v 140
#define COLOR_RED1_upper_v 255
//red2
#define COLOR_RED2_lower_h 170
#define COLOR_RED2_upper_h 180
#define COLOR_RED2_lower_s 140
#define COLOR_RED2_upper_s 255
#define COLOR_RED2_lower_v 140
#define COLOR_RED2_upper_v 255
//yellow
#define COLOR_YELLOW_lower_h 20
#define COLOR_YELLOW_upper_h 35
#define COLOR_YELLOW_lower_s 110
#define COLOR_YELLOW_upper_s 255
#define COLOR_YELLOW_lower_v 120
#define COLOR_YELLOW_upper_v 255
//black
#define COLOR_BLACK_lower_h 0
#define COLOR_BLACK_lower_s 0
#define COLOR_BLACK_lower_v 0
#define COLOR_BLACK_upper_h 180
#define COLOR_BLACK_upper_s 255
#define COLOR_BLACK_upper_v 60
//white
#define COLOR_WHITE_lower_h 0
#define COLOR_WHITE_lower_s 0
#define COLOR_WHITE_lower_v 90
#define COLOR_WHITE_upper_h 255
#define COLOR_WHITE_upper_s 10
#define COLOR_WHITE_upper_v 255
//blue
#define COLOR_BLUE_lower_h 100
#define COLOR_BLUE_upper_h 130
#define COLOR_BLUE_lower_s 120
#define COLOR_BLUE_upper_s 255
#define COLOR_BLUE_lower_v 100
#define COLOR_BLUE_upper_v 255

//THRESHOLDS //HERE *SCALE_ *MAVLINK_VELOCITY_

//besmellah
#define THRESH_BESMELLAH 10000
//alt
#define THRESH_ALT_offset 0.05 //meter
//yaw
#define THRESH_YAW_offset CV_PI/16
//state machine
#define THRESH_STATE_TIMER 200
//houghline
#define THRESH_HOUGHLINES_VERTICAL_RATE 4
//mission
#define THRESH_MISSION_1_timer_approachTower 1000
#define THRESH_MISSION_1_timer_travelLength 500
#define THRESH_MISSION_1_timer_travelWidth THRESH_MISSION_1_timer_travelLength/3
#define THRESH_MISSION_2_timer_approachTower 1000
#define THRESH_MISSION_2_timer_travelHalfEdge 500
#define THRESH_MISSION_2_timer_travelWidth THRESH_MISSION_1_timer_travelLength/3
#define THRESH_MISSION_3_timer_nudge 1000
#define THRESH_MISSION_3_dis_x 50
#define THRESH_MISSION_lidarDis_min 5 //cm
#define THRESH_MISSION_lidarDis_max 10 //cm
#define THRESH_MISSION_DIS_altMax 100 //cm
//obstacle
#define THRESH_OBS_color 1000
#define THRESH_OBS_alt_yellow 20
#define THRESH_OBS_alt_red 20
//line
#define THRESH_LINE_d_pixles_black 2000
#define THRESH_LINE_d_pixles_white 1000
#define THRESH_LINE_c_pixles_black 2000
#define THRESH_LINE_c_pixles_white_inBlack 1500
#define THRESH_LINE_c_area_white_offroad 2000
#define THRESH_LINE_c_area_whiteLine_offset 50
#define THRESH_LINE_c_area_whiteLine 2000
#define THRESH_LINE_justLand_pixles_black 100
#define THRESH_LINE_justLand_pixles_blue 100
#define THRESH_LINE_hough_dx 100
//land
#define THRESH_LAND_area 10000
#define THRESH_LAND_dis_x 150
#define THRESH_LAND_dis_y 150
#define THRESH_LAND_lines_dis 500
#define THRESH_LAND_dis_linesX 500
//qr
#define THRESH_QR_timer_getOutOfBox 2000
#define THRESH_QR_dx 50
#define THRESH_QR_dy 50
//opf
#define THRESH_OPF_winSize 30

//DEBUG FLAGS

//text debug flags
#define DEBUG_TXT 1 //DEL
#define DEBUG_txt_opf 0
#define DEBUG_txt_line 0
#define DEBUG_txt_land 1
#define DEBUG_txt_qr 1
#define DEBUG_txt_obs 0
#define DEBUG_txt_mission 1
#define DEBUG_txt_mavlink 1
//camera debug flags | activate one camera debugger only
#define DEBUG_CAM 1 //DEL
#define DEBUG_cam_opf 0
#define DEBUG_cam_line 1
#define DEBUG_cam_land 0
#define DEBUG_cam_qr 0
#define DEBUG_cam_obs 0
#define DEBUG_cam_mission 0

//CAMERAS | 0 : sabz , 1 : abi, qermez
#define CAM_front 1
#define CAM_bottom !CAM_front

//MISSION
#define ON -1
#define OFF 0
#define MISSION_MODE ON
#define MISSION_DO_1 1
#define MISSION_DO_2 1
#define MISSION_DO_3 1
#define MISSION_MEASURE_timer 1
#define MISSION_MEASURE_lidar 0

// ######### KN2C Variables #####
    // ##                      ##
    // ##        comment       ##
    // ##                      ##
    // ##########################

typedef struct { // qr
  string type;
  string data;
  vector <Point> location;
} decodedObject;
typedef enum { // line
    unknown,
    white,
    black,
    black_with_white,
    black_with_white_line,
} crop_type;
typedef enum { // line
    MOVE_YAW_ALT,
    MOVE_FORWARD,
    MOVE_RIGHT,
    MOVE_LEFT,
} movement_direction;
typedef enum {
    STATE_DEFAULT, //0 : not other states
    STATE_OBS_decrease_alt, //1 : "red detection" -> bottomEye out, if altInRange -> STATE_LINE_underAboveObs
    STATE_OBS_increase_alt, //2 : "yellow detection" -> bottomEye out, if altInRange -> STATE_LINE_underAboveObs
    STATE_QR_isWithinCenter, //3 : "qr detection" -> 1st step of qr handling, bottomEye out, if within center -> STATE_QR_turnToDir
    STATE_QR_turnToDir, //4 : STATE_QR_isWithinCenter -> 2nd step of qr handling, bottomEye out, if yawinRange -> STATE_QR_getOutofBox || STATE_MISSION_?
    STATE_QR_getOutofBox, //5 : STATE_QR_turnToDir -> 3rd step of qr handling, bottomEye out, ignore qr box, stateMachineTimer -> STATE_DEFAULT
    STATE_LINE_getBackIntoRout, //6 : "centerX out of lines" || "obstacle detection" || STATE_LINE_getBackIntoRout -> frontEye out, if back into rout -> last state 
    STATE_LINE_landing, //7 : "mission 4" && isLandingState -> bottomEye && frontEye out, if within center -> lands THE END
    STATE_LINE_underAboveObs, //8 : "obstacle detection" -> do not update QUAD_alt, stateMachineTimer -> STATE_DEFAULT
    STATE_LINE_goTo_rightAlt, //9 : "stateMachineTimer" -> if altInRange -> STATE_DEFAULT
    STATE_MISSION_1, //10 : "qr detection" -> bottomEye && frontEye out -> STATE_QR_getOutofBox
    STATE_MISSION_2, //11 : "qr detection" -> bottomEye && frontEye out -> STATE_QR_getOutofBox
    STATE_MISSION_3, //12 : "qr detection" -> bottomEye && frontEye out -> STATE_QR_getOutofBox
    STATE_MISSION_4, //13 : "qr detection" -> bottomEye && frontEye out -> STATE_QR_getOutofBox
} state_machine;
typedef enum { // inter-thread msg protocol to send mavlink msg through one thread
    MSG_DEFAULT,
    MSG_land,
    MSG_height,
    MSG_velocity,
    MSG_yaw, //watchdog
} internal_message;

internal_message imsg;
mutex frameMutex; //DEL never used
Mat frame; // shared between optFlow and bottomEye
int width, height; // of frame
int cam_fps; //DEL never used
int stateMachineTimer = 0;
const string debugForward = "\033[1;42mforward\033[0m ";
const string debugLeft = "\033[1;42mleft\033[0m ";
const string debugRight = "\033[1;42mright\033[0m ";
bool isLandingState = false, isScanningState = false, isRerouting = false; //pre-state flags
int toTheMission_ = MISSION_MODE; // indicates which mission quad's onto
int goOnMission_; // indicates which mission quad's gonna do after STATE_QR_turnToDir | 0 indicates no mission
int isOnMission_; // indicates which mission quad's on (single thread)
movement_direction _movementDirection = MOVE_YAW_ALT;
// movement_direction lastMovementDirection = _movementDirection;
state_machine _stateMachine = STATE_DEFAULT;
state_machine lastStateMachine = _stateMachine;
float MAVLINK_velocityX, MAVLINK_velocityY, MAVLINK_yaw, MAVLINK_alt = MAVLINK_ALT_normal;
float QUAD_yaw, QUAD_alt, QUAD_lidar;
string lastMissionData;

int serialPort;
uint8_t bufWrite[MAVLINK_MAX_PACKET_LEN];
mavlink_message_t msgWrite;
mavlink_status_t statusRead;
mavlink_optical_flow_t optical_flow;
uint8_t type = MAV_TYPE_QUADROTOR;
uint8_t autopilot = MAV_AUTOPILOT_GENERIC;
uint8_t base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
uint32_t custom_mode = 0;
uint8_t system_status = MAV_STATE_ACTIVE ;
//uint16_t mavlink_msg_heartbeat_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msgWrite, uint8_t type, uint8_t autopilot, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status);
uint32_t time_boot_ms = 0;
uint8_t target_system = 1;
uint8_t target_component = 1;
uint8_t coordinate_frame = MAV_FRAME_BODY_FRD;
uint16_t type_mask;
//int16_t mavlink_msg_set_position_target_local_ned_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msgWrite, uint32_t time_boot_ms,uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, uint16_t type_mask, float x, float y, float z, float MAVLINK_VELOCITY_X, float MAVLINK_VELOCITY_Y, float vz, float afx, float afy, float afz, float MAVLINK_yaw, float yaw_rate);
int MAVLINK_yaw_timer, MAVLINK_yaw_timeout;
int subStateMachineTimer = 0, subStateMachine = 0;

Scalar lower_black(COLOR_BLACK_lower_h, COLOR_BLACK_lower_s, COLOR_BLACK_lower_v);
Scalar upper_black(COLOR_BLACK_upper_h, COLOR_BLACK_upper_s, COLOR_BLACK_upper_v);
Scalar lower_white(COLOR_WHITE_lower_h, COLOR_WHITE_lower_s, COLOR_WHITE_lower_v);
Scalar upper_white(COLOR_WHITE_upper_h, COLOR_WHITE_upper_s, COLOR_WHITE_upper_v);
Scalar lower_blue(COLOR_BLUE_lower_h, COLOR_BLUE_lower_s, COLOR_BLUE_lower_v);
Scalar upper_blue(COLOR_BLUE_upper_h, COLOR_BLUE_upper_s, COLOR_BLUE_upper_v);
Scalar lower_red1(COLOR_RED1_lower_h, COLOR_RED1_lower_s, COLOR_RED1_lower_v);
Scalar upper_red1(COLOR_RED1_upper_h, COLOR_RED1_upper_s, COLOR_RED1_upper_v);
Scalar lower_red2(COLOR_RED2_lower_h, COLOR_RED2_lower_s, COLOR_RED2_lower_v);
Scalar upper_red2(COLOR_RED2_upper_h, COLOR_RED2_upper_s, COLOR_RED2_upper_v);
Scalar lower_yellow(COLOR_YELLOW_lower_h, COLOR_YELLOW_lower_s, COLOR_YELLOW_lower_v);
Scalar upper_yellow(COLOR_YELLOW_upper_h, COLOR_YELLOW_upper_s, COLOR_YELLOW_upper_v);

void SERVO_setPosition(int pin, int position);
void MAVLINK_decode();
void MAVLINK_setAlt(float alt);
void MAVLINK_moveForward(float rate = 1.0);
void MAVLINK_moveLeft(float rate = 1.0);
void MAVLINK_moveRight(float rate = 1.0);
void MAVLINK_stop();
void MAVLINK_setYaw(float direction);
bool MAVLINK_isYawInRange();
bool MAVLINK_isAltInRange();
char QR_readData(string &data);
void QR_decode(Mat &im, vector<decodedObject>&decodedObjects);
void QR_decodeMissions(Mat &im, vector<decodedObject>&decodedObjects);
void QR_display(Mat &im, vector<decodedObject>&decodedObjects);
void set_stateMachine(state_machine state);
state_machine get_stateMachine();
bool isOnMissions();
void set_movementDirection(movement_direction direction);
movement_direction get_movementDirection();

void intelUnit();
void bottomEye();
void frontEye();

// ############ KN2C Main #######
    // ##                      ##
    // ##        comment       ##
    // ##                      ##
    // ##########################

int main() {

    //TEST
    // serialPort = open("/dev/ttyACM0", O_RDWR);//"\\\\.\\COM11",   O_RDWR); //flag
    // if(serialPort == -1)  {
    //     cerr << "serialPort_init: Unable to open port" << endl;
    //     return 0;
    // } else if(DEBUG_TXT) cout << "serialPort is open" << endl;

    if(DEBUG_TXT && !(DEBUG_txt_opf || DEBUG_txt_line || DEBUG_txt_qr || DEBUG_txt_obs || DEBUG_txt_land || DEBUG_txt_mission)) {
        cerr << "activating AT LEAST ONE text debugger is necessary when DEBUG_TXT is 1 (Ln 32)" << endl;
        return -1;
    }
    if(DEBUG_CAM && !(DEBUG_cam_opf ^ DEBUG_cam_line ^ DEBUG_cam_qr ^ DEBUG_cam_obs ^ DEBUG_cam_land ^ DEBUG_cam_mission)) {
        cerr << "activating ONLY ONE camera debugger is necessary when DEBUG_CAM is 1 (Ln 38)" << endl;
        return -2;
    }
    while(toTheMission_ > 4 || toTheMission_ < 0) {
        cout << "input toTheMission_ : ";
        cin >> toTheMission_;
        if(toTheMission_ == 4) isLandingState = true;
    }
    //TEST 
    // if(MISSION_DO_3){
    //     if(wiringPiSetupGpio() == -1){
    //         cerr << "Failed to initialize WiringPi" << endl;
    //         return -3;
    //     }
    // }
    //creat threads
    thread BottomEye(bottomEye);
    thread FrontEye(frontEye);
    //TEST
    // thread IntelUnit(intelUnit);

    //joint threads
    if(BottomEye.joinable()) BottomEye.join();
    if(FrontEye.joinable()) FrontEye.join();
    //TEST
    // if(IntelUnit.joinable()) IntelUnit.join();

    return 1; 
}

// ######### KN2C Functions #####
    // ##                      ##
    // ##        comment       ##
    // ##                      ##
    // ##########################

//SERVO
void SERVO_setPosition(int pin, int position){
    int dutyCycle = (position*200/180)+SERVO_frequency; //+50
    pwmWrite(pin, dutyCycle);
}
//MAVLINK
void MAVLINK_decode() { // -> intelUnit
    uint8_t bufRead;
    mavlink_message_t msgRead;
    mavlink_status_t statusRead;
    mavlink_attitude_t attitude;
    mavlink_local_position_ned_t local_pos_ned;
    string debugID = "\033[1;47mMAVLINK :: MAVLINK_decode ::\033[0m ";
    bool flagYaw = false, flagAlt = false;
    while(1){
        read(serialPort, &bufRead, 1);
        if(mavlink_frame_char(MAVLINK_COMM_0, bufRead, &msgRead, &statusRead)){
            switch(msgRead.msgid) {
                case MAVLINK_MSG_ID_ATTITUDE:
                    mavlink_msg_attitude_decode(&msgRead, &attitude);
                    QUAD_yaw = attitude.yaw;
                    flagYaw = true;
                    if(DEBUG_txt_mavlink) cout << debugID << "yaw " << attitude.yaw << endl;
                case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
                    mavlink_msg_local_position_ned_decode(&msgRead, &local_pos_ned);
                    flagAlt = true;
                    if(get_stateMachine() != STATE_LINE_underAboveObs) QUAD_alt = local_pos_ned.z;
                    optical_flow.flow_comp_m_x = local_pos_ned.x;
                    optical_flow.flow_comp_m_y = local_pos_ned.y;
                    if(DEBUG_txt_mavlink) cout << debugID << "alt " << local_pos_ned.z << endl;
                //HERE decode rangefinder2
            } if(flagAlt && flagYaw) return;
        }
    }
}
void MAVLINK_moveForward(float rate) { //NOTE call out dontMove after movement completed
    movement_direction lastMovementDirection = get_movementDirection();
    set_movementDirection(MOVE_RIGHT);
    if(get_movementDirection() != lastMovementDirection) MAVLINK_stop();
    else {
        type_mask = MAVLINK_MSG_velocity_typeMASK; //HERE comman
        MAVLINK_velocityX = 0.0;
        MAVLINK_velocityY = MAVLINK_VELOCITY_Y*rate;
        imsg = MSG_velocity;
    }
}
void MAVLINK_moveLeft(float rate){ //NOTE call out dontMove after movement completed
    movement_direction lastMovementDirection = get_movementDirection();
    set_movementDirection(MOVE_RIGHT);
    if(get_movementDirection() != lastMovementDirection) MAVLINK_stop();
    else {
        type_mask = MAVLINK_MSG_velocity_typeMASK;
        MAVLINK_velocityX = -MAVLINK_VELOCITY_X*rate;
        MAVLINK_velocityY = MAVLINK_VELOCITY_X*MAVLINK_VELOCITY_zigzagRate*rate;
        imsg = MSG_velocity;
    }
}
void MAVLINK_moveRight(float rate){ //NOTE call out dontMove after movement completed
    movement_direction lastMovementDirection = get_movementDirection();
    set_movementDirection(MOVE_RIGHT);
    if(get_movementDirection() != lastMovementDirection) {
        MAVLINK_stop();
    } else {
        type_mask = MAVLINK_MSG_velocity_typeMASK;
        MAVLINK_velocityX = MAVLINK_VELOCITY_X*rate;
        MAVLINK_velocityY = MAVLINK_VELOCITY_X*MAVLINK_VELOCITY_zigzagRate*rate;
        imsg = MSG_velocity;
    }
}
void MAVLINK_stop(){
    type_mask = MAVLINK_MSG_velocity_typeMASK;
    MAVLINK_velocityX = 0.0;
    MAVLINK_velocityY = 0.0;
    imsg = MSG_velocity;
}
void MAVLINK_setYaw(float direction) {
    movement_direction lastMovementDirection = get_movementDirection();
    set_movementDirection(MOVE_YAW_ALT);
    if(get_movementDirection() != lastMovementDirection) MAVLINK_stop();
    else {
        type_mask = MAVLINK_MSG_yaw_typeMASK;
        string debugID = "\033[1;47mYAW :: MAVLINK_setYaw ::\033[0m ";
        if(DEBUG_TXT && DEBUG_txt_qr) cout << debugID << "turning to " << direction << endl;
        MAVLINK_yaw = direction;
        MAVLINK_yaw_timer++;
        MAVLINK_yaw_timeout = MAVLINK_yaw/MAVLINK_yawRate;
        imsg = MSG_yaw;
    }
}
bool MAVLINK_isYawInRange() {
    if(QUAD_yaw < MAVLINK_yaw+THRESH_YAW_offset && QUAD_yaw > MAVLINK_yaw-THRESH_YAW_offset){
        imsg = MSG_DEFAULT;
        return true;
    } return false;
}
void MAVLINK_setAlt(float alt){
    movement_direction lastMovementDirection = get_movementDirection();
    set_movementDirection(MOVE_YAW_ALT);
    if(get_movementDirection() != lastMovementDirection) MAVLINK_stop();
    else {
        //HERE cor
        type_mask = MAVLINK_MSG_position_typeMASK;
        MAVLINK_alt = alt;
        imsg = MSG_height;
    }
}
bool MAVLINK_isAltInRange() {
    if(QUAD_alt < MAVLINK_alt+THRESH_ALT_offset && QUAD_yaw > MAVLINK_alt-THRESH_ALT_offset){
        imsg = MSG_DEFAULT;
        return true;
    } return false;
}
//QR
char QR_readData(string &data) { // -> QR_decode
    int index = int(data.back())-48;
    string debugID = "\033[1;47mQR :: QR_readData ::\033[0m ";

    if(index - toTheMission_ == 1) {
        if(index == 1 && MISSION_DO_1) goOnMission_ = 1;
        else if(index == 2 && MISSION_DO_2) goOnMission_ = 2;
        else if(index == 3 && MISSION_DO_3) goOnMission_ = 3;
        else {
            goOnMission_ = 0;
            toTheMission_++;
        } if(DEBUG_TXT && DEBUG_txt_qr) cout << debugID << "toTheMission_ updated : " << toTheMission_ << " isOnMission_ = " << isOnMission_ << " goOnMission_ = " << goOnMission_ << endl;
    } if(DEBUG_TXT && DEBUG_txt_qr) cout << debugID << "returning " << data[toTheMission_*2] << endl;
    return data[toTheMission_*2];
}
void QR_decode(Mat &im, vector<decodedObject>&decodedObjects) { // -> bottomEye

    // Create zbar scanner
    ImageScanner scanner;
    // Configure scanner
    scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1); 
    // Convert image to grayscale
    Mat imGray;
    //pi camera color map NOT
    cvtColor(im, im, COLOR_BGR2RGB);
    cvtColor(im, imGray,COLOR_BGR2GRAY);
    // Wrap image data in a zbar image
    Image image(im.cols, im.rows, "Y800", (uchar *)imGray.data, im.cols * im.rows);
    // Scan the image for barcodes and QRCodes
    int n = scanner.scan(image);
    string debugID = "\033[1;47mQR :: QR_decode ::\033[0m ";

    float ch = (height/2)*SCALE_line_land_qr;
    float cw = (width/2)*SCALE_line_land_qr;
    float dx, dy;
    float meadX, meadY;
    int num;
    //cout << ch << cw << endl;
    // Print results
    for(Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol){
        decodedObject obj;
        obj.type = symbol->get_type_name();
        obj.data = symbol->get_data();

        // Print type and data
        if(DEBUG_TXT && DEBUG_txt_qr) cout << debugID << "Data = " << obj.data << endl;
        if(DEBUG_TXT && (DEBUG_txt_qr || DEBUG_txt_line)) cout << debugID << "qr detected ===> \033[1;42mstop\033[0m" << endl;

        //obtain location
        for(int i = 0; i < symbol->get_location_size(); i++) {
            obj.location.push_back(Point(symbol->get_location_x(i), symbol->get_location_y(i)));
        }
        decodedObjects.push_back(obj);
        meadX = 0.0;
        meadY = 0.0;
        num = 0;
        for(int i = 0; i < decodedObjects.size(); i++) { // 1 iteration
            vector<Point> points = decodedObjects[i].location;
            vector<Point> hull;
            if(points.size() > 4) convexHull(points, hull);
            else hull = points;
            num += hull.size();
            for(int j = 0; j < hull.size(); j++) {
                meadX += hull.at(j).x;
                meadY += hull.at(j).y;
            }
        }
        meadX /= num;
        meadY /= num;
        // if(DEBUG_TXT && DEBUG_txt_qr) cout << debugID << "cw " << cw << " meadX " << meadX << endl;
        // if(DEBUG_TXT && DEBUG_txt_qr) cout << debugID << "ch " << ch << " meadY " << meadY << endl;
        // find center box
        dx = cw - meadX; // symbol->get_location_x(symbol->get_location_size()-1); // here
        dy = ch - meadY; // here
        //HERE wtf??? if(dy < 0 && get_stateMachine() != STATE_QR_turnToDir) return;
        set_stateMachine(STATE_QR_isWithinCenter); //check center
        if(DEBUG_TXT && DEBUG_txt_qr) cout << debugID << "dx = " << dx << " dy = " << dy << endl;
        if(abs(dx) > THRESH_QR_dx) {
            if(dx < 0) {
                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "turning to " << debugRight << endl;
                MAVLINK_moveRight(); //flag
            } else {
                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "turning to " << debugLeft << endl;
                MAVLINK_moveLeft(); //flag
            } imsg = MSG_velocity;
        } else if(dy < -THRESH_QR_dy) {
            if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "turning to " << debugForward << endl;
            MAVLINK_moveForward(); //flag
        } else {
            set_stateMachine(STATE_QR_turnToDir);
            switch(QR_readData(obj.data)) { //state get out of box + turn to dir
                case 'N':
                    MAVLINK_setYaw(MAVLINK_YAW_0);
                    // if(MAVLINK_isYawInRange()) { //TEST
                        if(goOnMission_) isOnMission_ = goOnMission_;
                        else set_stateMachine(STATE_QR_getOutofBox);
                    // } //flag
                    break;
                case 'E':
                    MAVLINK_setYaw(MAVLINK_YAW_90);
                    // if(MAVLINK_isYawInRange()) {
                        if(goOnMission_) isOnMission_ = goOnMission_;
                        else set_stateMachine(STATE_QR_getOutofBox);
                    // } //flag
                    break;
                case 'S':
                    MAVLINK_setYaw(MAVLINK_YAW_180);
                    // if(MAVLINK_isYawInRange()) {
                        if(goOnMission_) isOnMission_ = goOnMission_;
                        else set_stateMachine(STATE_QR_getOutofBox);
                    // } //flag
                    break;
                case 'W':
                    MAVLINK_setYaw(MAVLINK_YAW_270);
                    // if(MAVLINK_isYawInRange()) {
                        if(goOnMission_) isOnMission_ = goOnMission_;
                        else set_stateMachine(STATE_QR_getOutofBox);
                    // } //flag
                    break;
                default:
                    MAVLINK_setYaw(MAVLINK_YAW_180);
                    if(MAVLINK_isYawInRange()) {
                        set_stateMachine(STATE_QR_getOutofBox);
                        isLandingState = true;
                    }
                    //flag
                    break;
            }
        }
    } if(DEBUG_TXT) cout << endl;
}
void QR_decodeMissions(Mat &im, vector<decodedObject>&decodedObjects) { // -> bottomEye

    // Create zbar scanner
    ImageScanner scanner;
    // Configure scanner
    scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1); 
    // Convert image to grayscale
    Mat imGray;
    //pi camera color map NOT
    cvtColor(im, im, COLOR_BGR2RGB);
    cvtColor(im, imGray,COLOR_BGR2GRAY);
    // Wrap image data in a zbar image
    Image image(im.cols, im.rows, "Y800", (uchar *)imGray.data, im.cols * im.rows);
    // Scan the image for barcodes and QRCodes
    int n = scanner.scan(image);
    string debugID = "\033[1;47mQR :: QR_decodeMissions ::\033[0m ";

    float ch = (height/2)*SCALE_line_land_qr;
    float cw = (width/2)*SCALE_line_land_qr;
    float dx, dz;
    float meadX, meadZ;
    int num;
    //cout << ch << cw << endl;
    // Print results
    for(Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol){
        decodedObject obj;
        obj.type = symbol->get_type_name();
        obj.data = symbol->get_data();

        // Print type and data
        if(lastMissionData != obj.data) {
            lastMissionData = obj.data;
            if(DEBUG_TXT && DEBUG_txt_qr) cout << debugID << "Data = " << obj.data << endl;
            if(DEBUG_TXT && (DEBUG_txt_qr || DEBUG_txt_line)) cout << debugID << "qr detected ===> \033[1;42mno need to stop\033[0m" << endl;
        }
        //obtain location
        for(int i = 0; i < symbol->get_location_size(); i++) obj.location.push_back(Point(symbol->get_location_x(i), symbol->get_location_y(i)));
        decodedObjects.push_back(obj);
    } if(DEBUG_TXT) cout << endl;
}
void QR_display(Mat &im, vector<decodedObject> &decodedObjects) { // -> bottomEye
    
    if(!(DEBUG_CAM && DEBUG_cam_qr) || isLandingState) return;
    string debugID = "\033[1;47mQR :: QR_display ::\033[0m ";
    // Loop over all decoded objects
    for(int i = 0; i < decodedObjects.size(); i++) {
        vector<Point> points = decodedObjects[i].location;
        vector<Point> hull;
        // If the points do not form a quad, find convex hull
        if(points.size() > 4) convexHull(points, hull);
        else hull = points;
    
        // Number of points in the convex hull
        int n = hull.size();
        int randColor1 = rand()%255;
        int randColor2 = rand()%255;
        int randColor3 = rand()%255;
        for(int j = 0; j < n; j++){
            polylines(im, hull, true, Scalar(randColor1,randColor2,randColor3), 2);
            putText(im, decodedObjects[i].data, hull[0], FONT_HERSHEY_DUPLEX, 0.75, Scalar(200, 0, 150),1.5);
        }
    } imshow(debugID+"Results", im); // Display results
}
//AUXILIARY
state_machine get_stateMachine() {
    // static int timer = 0;
    // if(_stateMachine == STATE_QR_getOutofBox) timer++;
    // if(timer > THRESH_QR_timer_getOutOfBox) {
    //     timer = 0;
    //     set_stateMachine(STATE_DEFAULT);
    // }
    return _stateMachine;
}
void set_stateMachine(state_machine state) {
    if(get_stateMachine() != state) lastStateMachine = get_stateMachine();
    if(!(state == STATE_MISSION_1 || state == STATE_MISSION_2 || state == STATE_MISSION_3)) {
        isOnMission_ = 0;
        goOnMission_ = 0;
        subStateMachine = 0;
        subStateMachineTimer = 0;
    }
    if(!(get_stateMachine() == STATE_LINE_getBackIntoRout || state == STATE_LINE_getBackIntoRout)) stateMachineTimer = 0;
    _stateMachine = state; //HERE bala paain
    if(DEBUG_TXT) {
        cout << "$ stateMachine has been set to " << get_stateMachine() << endl;
        cout << "$ lastStateMachine " << lastStateMachine << endl;
    }
}
bool isOnMissions() {
    switch(isOnMission_) {
        case 1:
            set_stateMachine(STATE_MISSION_1);
            return true;
        case 2:
            set_stateMachine(STATE_MISSION_2);
            return true;
        case 3:
            set_stateMachine(STATE_MISSION_3);
            return true;
        case 4:
            set_stateMachine(STATE_MISSION_4);
            return true;
    } return false;
}
movement_direction get_movementDirection() {
    return _movementDirection;
}
void set_movementDirection(movement_direction direction) {
    // if(get_movementDirection() != direction) lastMovementDirection = get_movementDirection();
    _movementDirection = direction;
}

// ########## KN2C THREADS ######
    // ##                      ##
    // ##        comment       ##
    // ##                      ##
    // ##########################

//INTEL UNIT : optical flow(down currently) + send mavlink msg + decode mavlink data
void intelUnit() {

    //make objects(variables)
    Mat old_frame, old_resize, old_gray;
    vector<Point2f> p0, p1; // Point2f d i
    vector<Scalar> colors;
    RNG rng;
    Mat new_frame,frame_resize, frame_gray;
    vector<uchar> status;
    vector<float> err;
    clock_t start, end;
    float fpslive;
    int num;
    float totalMovementx, dx;
    float totalMovementy, dy;
    string debugID = "\033[1;46mOPTICAL FLOW ::\033[0m ";
    int MSG_opf_delay = 0;
    
    while(true){
        switch(imsg) {
            case MSG_velocity: mavlink_msg_set_position_target_local_ned_pack(1, 200, &msgWrite, time_boot_ms, target_system, target_component, coordinate_frame, type_mask, 0, 0, 0, MAVLINK_velocityX, MAVLINK_velocityY, 0, 0, 0, 0, 0, 0);
            case MSG_yaw:
                if(MAVLINK_yaw_timer > MAVLINK_yaw_timeout) mavlink_msg_command_long_pack(MAVLINK_MSG_systemID, MAVLINK_MSG_componentID, &msgWrite, 1, 0, 115, 0, MAVLINK_yaw, MAVLINK_yawRate, 1, 1, 0, 0, 0);
                //mavlink_msg_set_position_target_local_ned_pack(1, 200, &msgWrite, time_boot_ms, target_system, target_component, coordinate_frame, type_mask, 0, 0, 0, 0, 0, 0, 0, 0, 0, MAVLINK_yaw, 0);
            case MSG_height: mavlink_msg_set_position_target_local_ned_pack(1, 200, &msgWrite, time_boot_ms, target_system, target_component, coordinate_frame, type_mask, 0, 0, MAVLINK_alt, 0, 0, 0, 0, 0, 0, 0, 0);                 
            case MSG_land: mavlink_msg_command_long_pack(MAVLINK_MSG_systemID, MAVLINK_MSG_componentID, &msgWrite, 1, 0, 21, 0, 0, 0, 0, 0, 0, 0, 1);
            //case MSG_DEFAULT: mavlink_msg_heartbeat_pack(1, 200, &msgWrite, type, autopilot, base_mode, custom_mode, system_status);
        }
        //HERE read serialPort
        if(imsg != MSG_DEFAULT) {
            mavlink_msg_to_send_buffer(bufWrite, &msgWrite); //HERE what??
            write(serialPort, &bufWrite, MAVLINK_MAX_PACKET_LEN);
        }
        MAVLINK_decode();
    }
}
//BOTTOM EYE : line + qr + land + mission 1 + mission 3 (using front cam)
void bottomEye() {
    //define variables
    int area_max_i_black, area_max_i_white;
    float area_max_black, area_max_white;
    Mat xframe,frameToProcess, toblur, hsv, mask_black, gray, thr, mask_white, mask_red1, mask_red2;
    Mat canny, canny1, canny2, colorframe;
    Mat crop1, crop2, crop3, crop_black, crop_white, mask2;
    vector<vector<Point>> contours_black, contours_blue, contours_white;
    Point2i p0;
    float left_right;
    int blackPixles, whitePixles;
    string debugID;
    vector<decodedObject> decodedObjects;
    Mat mask_blue;
    clock_t start, end;
    float area_blue, area_red;
    float fpslive;

    float max;
    float ch = (float)(height/2);//(float)(height)/2.0;
    float cw = (float)(width/2);
    //(float)(width)/2.0;
    // int ch = height/2;
    // int cw = width/2;
    // int greatestBlueContour;
    vector<Point> greatestBlueContour, greatestBlackContour, greatestWhiteContour;
    float greatestBlueContourArea;
    int greatestWhiteContourArea;
    Point2f contourCenter;
    Point2f imageCenter(cw, ch);
    Rect largestBlueRect, largestBlackRect, largestWhiteRect;
    float dis;
    float current;
    float dx, dy;
    float dxH, dyH;
    vector<Vec4i> linesP, linesP1, linesP2; // will hold the results of the detection

    const int crop_c_len = 5;
    const int BLACK = 0;
    const int WHITE = 1;
    crop_type c[crop_c_len];
    bool d[crop_c_len][2];
    const int CROP_left = 0;
    const int CROP_firstQuarter = (crop_c_len-1)/4;
    const int CROP_mid = (crop_c_len-1)/2;
    const int CROP_thirdQuarter = (crop_c_len-1)*3/4;
    const int CROP_right = crop_c_len-1;
    float posXmax = -1.0;
    float posXmin = 10000.0;
    
    int lineCounter = 0;
    Vec4i l;
    Point2f center,line_center;
    stateMachineTimer = 0;
    
    //webcam
    VideoCapture cap0(CAM_bottom, CAP_GSTREAMER);
    //VideoCapture cap0("video2.mp4"); //video

    if (!cap0.isOpened()) {
        cerr << "Failed to open bottom camera." << endl;
        return;
    }

    width = static_cast<int>(cap0.get(3));
    height = static_cast<int>(cap0.get(4));
    //cout << width <<"\t\t\t\t\t\t\t"<< height << endl;
    cam_fps = cap0.get(CAP_PROP_FPS);
    if(DEBUG_TXT && (DEBUG_txt_opf || DEBUG_txt_line || DEBUG_txt_qr)) cout << "$ bottom camera frame per second: " << cam_fps <<endl;
    
    //TEST
    // //ARM
    // uint8_t bufArm[MAVLINK_MAX_PACKET_LEN];
    // mavlink_message_t msgArm;
    // if(DEBUG_TXT) cout << "$ arm" << endl;
    // mavlink_msg_command_long_pack(MAVLINK_MSG_systemID, MAVLINK_MSG_componentID, &msgArm, 1, 1, MAVLINK_MSG_arm_typeMASK, 1, 1, 0, 0, 0, 0, 0, 0);
    // mavlink_msg_to_send_buffer(bufArm, &msgArm);
    // write(serialPort, &bufArm, MAVLINK_MAX_PACKET_LEN);
    // //usleep(USLEEP_arm);

    // //takeoff
    // uint8_t bufTakeoff[MAVLINK_MAX_PACKET_LEN];
    // mavlink_message_t msgTakeoff;
    // if(DEBUG_TXT) cout << "$ takeoff" << endl;
    // do {
    //     mavlink_msg_command_long_pack(MAVLINK_MSG_systemID, MAVLINK_MSG_componentID, &msgTakeoff, 1, 0, MAVLINK_MSG_takeoff_typeMASK, 0, 0, 0, 0, 0, 0, 0, MAVLINK_ALT_normal);
    //     mavlink_msg_to_send_buffer(bufTakeoff, &msgTakeoff);
    //     write(serialPort, &bufTakeoff, MAVLINK_MAX_PACKET_LEN);
    //     //usleep(USLEEP_takeoff);
    // } while(!MAVLINK_isAltInRange());

    // //enter rout
    // if(DEBUG_TXT) cout << "$ enter rout" << endl;
    // for(int i = 0; i < THRESH_BESMELLAH; i++) MAVLINK_moveForward(MAVLINK_VELOCITY_SLOW);
    // MAVLINK_stop();

    //TEST
    // cout << "setAlt" << endl;
    // do MAVLINK_setAlt(MAVLINK_ALT_normal); while(!MAVLINK_isAltInRange());
    // cout << "setYaw" << endl;
    // do MAVLINK_setYaw(MAVLINK_YAW_180); while(!MAVLINK_isYawInRange());
    while(true) {
        
        //cap0.set(CAP_PROP_FPS, 5);
        decodedObjects.clear();
        cap0 >> xframe; // Capture a frame
        // cvtColor(old_frame, old_gray, COLOR_BGR2GRAY);
        // if(xframe.empty()) continue;
        
        stateMachineTimer++;
        if(DEBUG_TXT) cout << "$ stateMachineTimer = " << stateMachineTimer << " | toTheMission_ = " << toTheMission_ << " | QUAD_alt = " << QUAD_alt << " | QUAD_yaw = " << QUAD_yaw << endl;
        if(isOnMissions()) {
            debugID = "\033[1;47mMISSION BOTTOM ::\033[0m ";
            float lastYaw = QUAD_yaw;
            while(get_stateMachine() == STATE_MISSION_1) {
                // decodedObjects.clear();
                // {
                //     lock_guard<mutex> lock(frameMutex);
                //     xframe = frame.clone(); // Store the frame in the shared variable
                // }
                // lock_guard<mutex> lock(frameMutex);
                // xframe = frame.clone(); //NOTE {}?
                resize(xframe, frameToProcess ,Size(), SCALE_line_land_qr, SCALE_line_land_qr);
                cvtColor(frameToProcess, frameToProcess, COLOR_BGR2RGB);
                cvtColor(frameToProcess, hsv, COLOR_BGR2HSV);
                QR_decodeMissions(frameToProcess, decodedObjects);
                if(DEBUG_CAM && DEBUG_cam_qr) QR_display(colorframe, decodedObjects);
                if(DEBUG_TXT && DEBUG_txt_mission) cout << debugID << "1 | subStateMachine = " << subStateMachine << " | subStateMachineTimer " << subStateMachineTimer  << endl;
                if(subStateMachine < 12) {
                    continue;
                } else if(subStateMachine == 12) {
                    inRange(hsv, lower_black, upper_black, mask_black);
                    GaussianBlur(mask_black, mask_black, Size(5, 5), 0);
                    Canny(mask_black, canny, 50, 200, 3);
                    HoughLinesP(canny, linesP, 1, CV_PI/180, 50, 80, 200 ); 
                    if(linesP.size() < 2) MAVLINK_moveRight(MAVLINK_VELOCITY_SLOW);
                    else subStateMachine++;
                } else if(subStateMachine == 13) {
                    MAVLINK_setYaw(lastYaw-MAVLINK_YAW_180);
                    //TEST if(MAVLINK_isYawInRange())
                    subStateMachine++;
                } else if(subStateMachine == 14) {
                    MAVLINK_setAlt(MAVLINK_ALT_normal);
                    //TEST if(MAVLINK_isAltInRange())
                    subStateMachine++;
                } else {
                    set_stateMachine(STATE_DEFAULT);
                    toTheMission_++;
                }
            }
            while(get_stateMachine() == STATE_MISSION_2) {
                // decodedObjects.clear();
                // lock_guard<mutex> lock(frameMutex);
                // xframe = frame.clone(); //NOTE {}?
                resize(xframe, frameToProcess ,Size(), SCALE_line_land_qr, SCALE_line_land_qr);
                cvtColor(frameToProcess, frameToProcess, COLOR_BGR2RGB);
                cvtColor(frameToProcess, hsv, COLOR_BGR2HSV);
                if(subStateMachine < 18) {
                    continue;
                } else if(subStateMachine == 18) {
                    inRange(hsv, lower_black, upper_black, mask_black);
                    GaussianBlur(mask_black, mask_black, Size(5, 5), 0);
                    Canny(mask_black, canny, 50, 200, 3);
                    HoughLinesP(canny, linesP, 1, CV_PI/180, 50, 80, 200 ); 
                    if(linesP.size() < 2) MAVLINK_moveRight(MAVLINK_VELOCITY_SLOW);
                    else subStateMachine++;
                } else if(subStateMachine == 19) {
                    MAVLINK_setYaw(lastYaw-MAVLINK_YAW_180);
                    //TEST if(MAVLINK_isYawInRange())
                    subStateMachine++;
                } else {
                    toTheMission_++;
                    set_stateMachine(STATE_DEFAULT);
                }
            }
            while(get_stateMachine() == STATE_MISSION_3) {
                // decodedObjects.clear();
                // {
                //     lock_guard<mutex> lock(frameMutex);
                //     xframe = frame.clone(); // Store the frame in the shared variable
                // }
                // lock_guard<mutex> lock(frameMutex);
                // xframe = frame.clone(); //NOTE {}?
                
                resize(xframe, frameToProcess ,Size(), SCALE_line_land_qr, SCALE_line_land_qr);
                cvtColor(frameToProcess, frameToProcess, COLOR_BGR2RGB);
                cvtColor(frameToProcess, hsv, COLOR_BGR2HSV);
                if(subStateMachine == 0) {
                    inRange(hsv, lower_red1, upper_red1, mask_red1);
                    inRange(hsv, lower_red2, upper_red2, mask_red2);
                    area_red = countNonZero(mask_red1) + countNonZero(mask_red2);
                    Canny(mask_red1, canny1, 50, 200, 3);
                    Canny(mask_red2, canny2, 50, 200, 3);
                    // runs the actual detection
                    HoughLinesP(canny1, linesP1, 1, CV_PI/180, 50, 80, 200 ); 
                    HoughLinesP(canny2, linesP2, 1, CV_PI/180, 50, 80, 200 ); 

                    if(linesP1.size()+linesP2.size() > 1){    
                        line_center.x = 0;
                        line_center.y = 0;
                        posXmin = 10000.0;
                        posXmax = -1.0;
                        lineCounter = 0;
                        // Draw the lines
                        for( size_t i = 0; i < linesP1.size(); i++ ) {
                            l = linesP1[i];
                            //only vertical lines
                            if(THRESH_HOUGHLINES_VERTICAL_RATE*abs(l[0]-l[2]) < abs(l[3]-l[1])){    
                                float posX = (float)(l[0]+l[2])/2.0;
                                lineCounter++;
                                //if(posX > 384) return;
                                if(posX > posXmax) posXmax = posX;
                                if(posX < posXmin) posXmin = posX;
                                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "l[0] " << l[0] << " l[2] " << l[2] << " posX " << posX << " min " << posXmin << " max "<< posXmax << endl;
                                
                                line_center.x += ((l[0]+l[2])/2);
                                line_center.y += ((l[1]+l[3])/2);
                            }
                        } for( size_t i = 0; i < linesP2.size(); i++ ) {
                            l = linesP2[i];
                            //only vertical lines
                            if(THRESH_HOUGHLINES_VERTICAL_RATE*abs(l[0]-l[2]) < abs(l[3]-l[1])){    
                                float posX = (float)(l[0]+l[2])/2.0;
                                lineCounter++;
                                //if(posX > 384) return;
                                if(posX > posXmax) posXmax = posX;
                                if(posX < posXmin) posXmin = posX;
                                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "l[0] " << l[0] << " l[2] " << l[2] << " posX " << posX << " min " << posXmin << " max "<< posXmax << endl;
                                
                                line_center.x += ((l[0]+l[2])/2);
                                line_center.y += ((l[1]+l[3])/2);
                            }
                        } line_center.x /= lineCounter; line_center.y /= lineCounter;
                        dxH = cw - line_center.x;
                        dyH = ch - line_center.y;
                        if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "dxH = " << dxH << "\tdyH = " << dyH << endl;
                        
                        if(abs(dxH) > THRESH_MISSION_3_dis_x) {//HERE delete dxH clause use posmin max
                            if(dxH < 0) {
                                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "turning to " << debugRight << endl;
                                MAVLINK_moveRight(); //flag
                            } else {
                                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "turning to " << debugLeft << endl;
                                MAVLINK_moveLeft(); //flag
                            }
                        } else if(dyH > 0) {
                            if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "turning to " << debugForward << endl;
                            MAVLINK_moveForward(); //flag
                        } else {
                            if(DEBUG_TXT && DEBUG_txt_land) cout << debugID << "center found" << endl;
                            subStateMachine++;//flag
                        }
                    }
                } else if(subStateMachine == 1) {
                    if(subStateMachineTimer < THRESH_MISSION_3_timer_nudge) {
                        MAVLINK_moveForward(MAVLINK_VELOCITY_SLOW);
                        subStateMachineTimer++;
                    } else {
                        subStateMachine++;
                        subStateMachineTimer = 0;
                    }
                } else if(subStateMachine == 2) {
                    //drop
                    pinMode(SERVO_pin, PWM_OUTPUT);
                    pwmSetMode(PWM_MODE_MS);
                    pwmSetClock(SERVO_PWM_clock_divisor);
                    pwmSetRange(SERVO_PWM_range);
                    for(int position = 0; position <= 180; position += SERVO_rotationRate){
                        SERVO_setPosition(SERVO_pin, position);
                        usleep(USLEEP_servo);
                    }
                    //HERE confirm
                    subStateMachine++; 
                } else if(subStateMachine == 3) {
                    MAVLINK_setYaw(lastYaw-MAVLINK_YAW_180);
                    //TEST if(MAVLINK_isYawInRange())
                    subStateMachine++;
                } else if(subStateMachine == 4) {
                    inRange(hsv, lower_black, upper_black, mask_black);
                    GaussianBlur(mask_black, mask_black, Size(5, 5), 0);
                    Canny(mask_black, canny, 50, 200, 3);
                    HoughLinesP(canny, linesP, 1, CV_PI/180, 50, 80, 200 ); 
                    if(linesP.size() < 2) MAVLINK_moveForward(MAVLINK_VELOCITY_SLOW);
                    else subStateMachine++;
                } else {
                    toTheMission_++;
                    set_stateMachine(STATE_DEFAULT);
                }
            } continue;
        }
        ch = (float)(height/2)*SCALE_line_land_qr;
        cw = (float)(width/2)*SCALE_line_land_qr;
        // cout << (float)(width)/2.0 << "  " << (float)(height)/2.0 << endl;
        // cout << cw << "  " << ch << endl; // KHALALJALEGH
        // {
        //     lock_guard<mutex> lock(frameMutex);
        //     xframe = frame.clone(); // Store the frame in the shared variable
        // }
        // lock_guard<mutex> lock(frameMutex);
        // xframe = frame.clone(); //NOTE {}?
        
        resize(xframe, frameToProcess ,Size(), SCALE_line_land_qr, SCALE_line_land_qr);
        //pi camera color map NOT
        cvtColor(frameToProcess, frameToProcess, COLOR_BGR2RGB);
        cvtColor(frameToProcess, hsv, COLOR_BGR2HSV);
        inRange(hsv, lower_blue, upper_blue, mask_blue);
        findContours(mask_blue, contours_blue, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        if(get_stateMachine() == STATE_LINE_landing) {
            // { // case while
            //     lock_guard<mutex> lock(frameMutex);
            //     xframe = frame.clone(); // Store the frame in the shared variable
            // }
                
            // if(xframe.empty()) continue;
            
            // resize(xframe, frameToProcess ,Size() ,SCALE_line_land_qr, SCALE_line_land_qr);
            // //pi camera color map NOT
            // cvtColor(frameToProcess, frameToProcess, COLOR_BGR2RGB);
            // cvtColor(frameToProcess, hsv, COLOR_BGR2HSV);
            // inRange(hsv, lower_blue, upper_blue, mask_blue);
            // findContours(mask_blue, contours_blue, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            
            debugID = "\033[1;44mLANDING ::\033[0m ";
            int i = 0;

            Canny(mask_blue, canny, 50, 200, 3);
            // runs the actual detection
            HoughLinesP(canny, linesP, 1, CV_PI/180, 50, 80, 200 ); 

            if(linesP.size() > 1){    
                line_center.x = 0;
                line_center.y = 0;
                posXmin = 10000.0;
                posXmax = -1.0;
                lineCounter = 0;
                // Draw the lines
                for( size_t i = 0; i < linesP.size(); i++ ) {
                    l = linesP[i];
                    //only vertical lines
                    if(4*abs(l[0]-l[2])<abs(l[3]-l[1])){    
                        float posX = (float)(l[0]+l[2])/2.0;
                        lineCounter++;
                        //if(posX > 384) return;
                        if(posX > posXmax) posXmax = posX;
                        if(posX < posXmin) posXmin = posX;
                        if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "l[0] " << l[0] << " l[2] " << l[2] << " posX " << posX << " min " << posXmin << " max "<< posXmax << endl;
                        
                        line_center.x += ((l[0]+l[2])/2);
                        line_center.y += ((l[1]+l[3])/2);
                    }
                } line_center.x /= lineCounter; line_center.y /= lineCounter;
                dxH = cw - line_center.x;
                dyH = ch - line_center.y;
                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "dxH = " << dxH << "\tdyH = " << dyH << endl;
                
                if(abs(dxH) > THRESH_QR_dx && posXmax-posXmin > THRESH_LAND_dis_linesX) {//HERE delete dxH clause
                    if(dxH < 0) {
                        if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "turning to " << debugRight << endl;
                        MAVLINK_moveRight(); //flag
                    } else {
                        if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "turning to " << debugLeft << endl;
                        MAVLINK_moveLeft(); //flag
                    }
                } else if(dyH > 0) {
                    if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "turning to " << debugForward << endl;
                    MAVLINK_moveForward(); //flag
                } else {
                    if(DEBUG_TXT && DEBUG_txt_land) cout << debugID << "\033[1;44mblue H found with houghline | Stop then decrease MAVLINK_alt\tlargest blue contour area = " << greatestBlueContourArea << "\tdx dy = " << dx << dy << "\033[0m" << endl;
                    MAVLINK_alt = 0;
                    imsg = MSG_land;
                    //flag
                    break;
                }
            } else {
                for (const auto& contour : contours_blue) {
                    double current = contourArea(contour);
                    if(current > max) {
                        max = current;
                        greatestBlueContour = contour;
                        if(DEBUG_TXT && DEBUG_txt_land) cout << debugID << "contour " << i << " is the largest with " << current << " area" << endl;
                    } i++;
                }
                if(!greatestBlueContour.empty()) {
                    dx = 0;
                    dy = 0;
                    largestBlueRect = boundingRect(greatestBlueContour);
                    contourCenter.x = largestBlueRect.x + largestBlueRect.width/2;
                    contourCenter.y = largestBlueRect.y + largestBlueRect.height/2;
                    if(DEBUG_CAM && DEBUG_cam_land) {
                        rectangle(frameToProcess, largestBlueRect, Scalar(255, 0, 0), 2);
                        circle(frameToProcess, imageCenter, 10, Scalar(0, 0, 0), -1);
                        circle(frameToProcess, contourCenter, 5, Scalar(0, 0, 255), -1);    
                        line(frameToProcess, contourCenter, imageCenter, Scalar(0,0,255), 3);
                    }
                    //dis = sqrt((cw - contourCenter.x)*(cw - contourCenter.x) + (ch - contourCenter.y)*(ch - contourCenter.y));
                    dx = abs(cw - contourCenter.x);
                    dy = abs(ch - contourCenter.y);
                    if(DEBUG_TXT && DEBUG_txt_land) cout << debugID << "dx = " << dx << " dy = " << dy << "\tfps = " << fpslive << endl;
                    end = clock();
                    fpslive = float(CLOCKS_PER_SEC)/(float(end)-float(start));
                    greatestBlueContourArea = contourArea(greatestBlueContour);
                    if(dx < THRESH_LAND_dis_x && dy < THRESH_LAND_dis_y && greatestBlueContourArea > THRESH_LAND_area) {
                        if(DEBUG_TXT && DEBUG_txt_land) cout << debugID << "\033[1;44mblue H found with contour | Stop then decrease MAVLINK_alt\tlargest blue contour area = " << greatestBlueContourArea << "\tdx dy = " << dx << dy << "\033[0m" << endl;
                        MAVLINK_alt = 0;
                        imsg = MSG_land;
                        //flag
                        break;
                    }
                }
            } continue;
        }
        if(stateMachineTimer > THRESH_STATE_TIMER) {
            cout << "$ state machine time out." << endl;
            set_stateMachine(STATE_LINE_goTo_rightAlt);
        }
        if(get_stateMachine() == STATE_LINE_goTo_rightAlt) {
            //TEST do MAVLINK_setAlt(MAVLINK_ALT_normal); while(!MAVLINK_isAltInRange());
            // here desiredAngle calc by previous qr
            set_stateMachine(STATE_DEFAULT);
        }
        
                
        debugID = "\033[1;42mFOLLOW LINE ::\033[0m ";
        //fine black road
        //resize(xframe, frameToProcess ,Size() ,SCALE_line_land_qr, SCALE_line_land_qr);
        //GaussianBlur(toblur, frameToProcess, Size(5, 5), 0);
        //cvtColor(frameToProcess, hsv, COLOR_BGR2HSV);
        inRange(hsv, lower_black, upper_black, mask_black);
        inRange(hsv, lower_white, upper_white, mask_white);
        GaussianBlur(mask_black, mask_black, Size(5, 5), 0);
        GaussianBlur(mask_white, mask_white, Size(5, 5), 0);
        findContours(mask_black, contours_black, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        findContours(mask_white, contours_white, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        
        
        if(isLandingState && (countNonZero(mask_black) < THRESH_LINE_justLand_pixles_black && countNonZero(mask_blue) > THRESH_LINE_justLand_pixles_blue)) set_stateMachine(STATE_LINE_landing);
        //qr
        if(get_stateMachine() != STATE_QR_getOutofBox) {
            QR_decode(frameToProcess, decodedObjects);
            if(DEBUG_CAM && DEBUG_cam_qr) QR_display(frameToProcess, decodedObjects);
        }
        if(get_stateMachine() == STATE_QR_isWithinCenter || get_stateMachine() == STATE_QR_turnToDir || get_stateMachine() == STATE_LINE_landing) continue;
        
        //follow line starrt
        //left right ajustment - find max area 
        // area_max_black = 0.0;
        // for (int i = 0; i < contours_black.size(); i++) {
        //     int current = contourArea(contours_black[i]);
        //     if(current>area_max_black) {
        //         area_max_i_black = i;
        //         area_max_black = current;
        //     }
        // }
        area_max_white = 0.0;
        for (int i = 0; i < contours_white.size(); i++) {
            int current = contourArea(contours_white[i]);
            if(current>area_max_white){
                area_max_i_white = i;
                area_max_white = current;
            }  
        }
        // greatestBlackContour = contours_black[area_max_i_black];
        if(!contours_white.empty()) {
            // if(!greatestBlackContour.empty() && !greatestWhiteContour.empty()) {
                // largestBlackRect = boundingRect(greatestBlackContour);
                // largestWhiteRect = boundingRect(greatestWhiteContour);
                
                // //find center to follow
                // p0.x = largestBlackRect.x + largestBlackRect.width/2;
                // p0.y = largestBlackRect.y + largestBlackRect.height/2;

                // if(DEBUG_CAM && DEBUG_cam_line){
                //     circle(frameToProcess, p0, 5, Scalar(0, 0, 255), -1);
                //     rectangle(frameToProcess, largestBlackRect, Scalar(0, 0, 255), 2);
                // }

                // //left right data normal btw -10 and 10
                // left_right = (((p0.x - (width/2)*SCALE_line_land_qr))/ ((width/2)*SCALE_line_land_qr))*10;

                // if(left_right>0){
                //     if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"left: "<<left_right<<endl;
                // } else if(left_right<0){
                //     if(DEBUG_TXT && DEBUG_txt_line)  cout << debugID <<"right: "<<left_right<<endl;
                // }
            greatestWhiteContour = contours_white[area_max_i_white];
            greatestWhiteContourArea = contourArea(greatestWhiteContour);
            if(greatestWhiteContourArea < THRESH_LINE_c_area_whiteLine) greatestWhiteContourArea = 0;
        }
        //crop, count number of black pixles, creat sensor output
        for(int i=0; i < crop_c_len ;i++){
            crop_black = mask_black(Rect(int((width/crop_c_len)*(i)*SCALE_line_land_qr),0,int((width/crop_c_len)*SCALE_line_land_qr),int(height*SCALE_line_land_qr)));
            crop_white = mask_white(Rect(int((width/crop_c_len)*(i)*SCALE_line_land_qr),0,int((width/crop_c_len)*SCALE_line_land_qr),int(height*SCALE_line_land_qr)));
            blackPixles = countNonZero(crop_black);
            whitePixles = countNonZero(crop_white);
            if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "crop " << i+1 << ") blackPixles = " << blackPixles << "\twhitePixles = " << whitePixles << endl;
            
            if(blackPixles > THRESH_LINE_d_pixles_black) d[i][BLACK] = true;
            else d[i][BLACK] = false;
            if(whitePixles > THRESH_LINE_d_pixles_white) d[i][WHITE] = true;
            else d[i][WHITE] = false;

            if(blackPixles > THRESH_LINE_c_pixles_black) {
                if(whitePixles > THRESH_LINE_c_pixles_white_inBlack) {
                    if(greatestWhiteContourArea && whitePixles > greatestWhiteContourArea-THRESH_LINE_c_area_whiteLine_offset && whitePixles < greatestWhiteContourArea+THRESH_LINE_c_area_whiteLine_offset) c[i] = black_with_white_line;
                    else c[i] = black_with_white;
                } else c[i] = black;
            } else if(whitePixles > THRESH_LINE_c_area_white_offroad) c[i] = white; else c[i] = unknown;
        }
        
        if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "c -> " << c[CROP_left] << c[CROP_firstQuarter] << c[CROP_mid] << c[CROP_thirdQuarter] << c[CROP_right] << endl;
        if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "d[:][BLACK] -> " << d[CROP_left][BLACK] << d[CROP_firstQuarter][BLACK] << d[CROP_mid][BLACK] << d[CROP_thirdQuarter][BLACK] << d[CROP_right][BLACK] << endl;
        if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "d[:][WHITE] -> " << d[CROP_left][WHITE] << d[CROP_firstQuarter][WHITE] << d[CROP_mid][WHITE] << d[CROP_thirdQuarter][WHITE] << d[CROP_right][WHITE] << endl;
        
        Canny(mask_black, canny, 50, 200, 3);
        // runs the actual detection
        HoughLinesP(canny, linesP, 1, CV_PI/180, 50, 80, 200 ); 

        if(linesP.size() > 1) {    
            line_center.x = 0;
            line_center.y = 0;
            posXmin = 10000.0;
            posXmax = -1.0;
            lineCounter = 0;
            // Draw the lines
            for( size_t i = 0; i < linesP.size(); i++ ) {
                l = linesP[i];
                //only vertical lines
                if(4*abs(l[0]-l[2])<abs(l[3]-l[1])){    
                    float posX = (float)(l[0]+l[2])/2.0;
                    lineCounter++;
                    //if(posX > 384) return;
                    if(posX > posXmax) posXmax = posX;
                    if(posX < posXmin) posXmin = posX;
                    if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << i << " )" << " sumLine = " << line_center.x << " lineCounter = " << lineCounter << "l[0] " << l[0] << " l[2] " << l[2] << " posX " << posX << " min " << posXmin << " max "<< posXmax << endl;
                    
                    line_center.x += posX;
                    if(l[3]<l[1]) line_center.y += l[3];
                    if(l[1]<l[3]) line_center.y += l[1];
                }
            } line_center.x /= lineCounter; line_center.y /= lineCounter;      
            if(!(get_stateMachine() == STATE_LINE_getBackIntoRout || get_stateMachine() == STATE_OBS_decrease_alt || get_stateMachine() == STATE_OBS_increase_alt) && (cw > posXmin && cw < posXmax)) {
                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "going " << debugForward << endl;
                MAVLINK_moveForward();
                //flag
            } else {
                set_stateMachine(STATE_LINE_getBackIntoRout);
                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "cw " << cw << " cw > posXmin " << (cw > posXmin) << "\tcw < posXmax " << (cw < posXmax) << endl;
                dxH = cw - line_center.x;
                dyH = ch - line_center.y;
                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "dxH = " << dxH << " line_center.x = " << line_center.x << endl;
                if(dxH > THRESH_LINE_hough_dx){//HERE || c[CROP_firstQuarter][WHITE]
                    isRerouting = true;
                    if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "turning to " << debugLeft << endl;
                    MAVLINK_moveLeft();
                    //flag
                } else if(dxH < -THRESH_LINE_hough_dx){// || c[CROP_thirdQuarter][WHITE]
                    isRerouting = true;
                    if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "turning to " << debugRight << endl;
                    MAVLINK_moveRight();
                    //flag
                } else {
                    isRerouting = false;
                    set_stateMachine(lastStateMachine);
                }
            }
        } else {
            if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "go " << debugForward << " khoda bozorgeh" << endl;
            MAVLINK_velocityX = 0;
            MAVLINK_velocityY = MAVLINK_VELOCITY_Y;
            imsg = MSG_velocity;
            //flag
        }

        /*  //movementDirectino -> previousMovementDirection
            // if(!(get_stateMachine() == STATE_LINE_getBackIntoRout || get_stateMachine() == STATE_OBS_decrease_alt || get_stateMachine() == STATE_OBS_increase_alt) && (d[CROP_mid][BLACK] || (d[CROP_mid][BLACK] && d[CROP_mid][BLACK]))) {
            //     if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "pitch 15 degree to " << debugForward << endl;
            // } else {
            //     set_stateMachine(STATE_LINE_getBackIntoRout);
            //     if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "couldn't find black in middle crops" << endl;
            //     Canny(mask_black, canny, 50, 200, 3);
            //     // runs the actual detection
            //     HoughLinesP(canny, linesP, 1, CV_PI/180, 50, 200, 200 ); 

            //     if(!linesP.empty()){    
            //         // Draw the lines
            //         for( size_t i = 0; i < linesP.size(); i++ ) {
            //             l = linesP[i];
            //             //only vertical lines
            //             if(4*abs(l[0]-l[2])<abs(l[3]-l[1])){    
            //                 line_center.x += l[0]+l[2];
            //                 if(l[3]<l[1]) line_center.y += l[3];
            //                 if(l[1]<l[3]) line_center.y += l[1];
            //             }
            //         }
            //         line_center.x = line_center.x/(2*linesP.size());
            //         line_center.y = line_center.y/(linesP.size());      
            //     }
            //     dxH = cw - line_center.x;
            //     dyH = ch - line_center.y;
            //     if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "dxH = " << dxH << "\tdyH = " << dyH << endl;
            //     // decode(frame,decodedObjects); //HERE
            //     // if(decodedObjects.empty()) {
            //         if(dxH > THRESH_LINE_hough_dx || d[CROP_firstQuarter][WHITE]){
            //             set_movementDirection(MOVE_RIGHT);
            //             if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "turning to " << debugRight << endl;
            //         } else if(dxH < -THRESH_LINE_hough_dx || d[CROP_thirdQuarter][WHITE]){
            //             set_movementDirection(MOVE_LEFT);
            //             if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "turning to " << debugLeft << endl;
            //         } else set_stateMachine(lastStateMachine); //HERE last state
            //     // }
            // }
            // // if(c[CROP_mid] == black_with_white_line) {
            //     if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "black_with_white_line in CROP_mid" << endl;
            //     if(!(c[CROP_left] == black || c[CROP_right] == black)) {
            //         if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"neither sides are black ===> pitch 15 degree to " << debugForward <<endl;
            //     } else if(c[CROP_firstQuarter] == black_with_white && c[CROP_thirdQuarter] == black_with_white) {
            //         if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"quarters are black_with_white ===> pitch 15 degree to " << debugForward <<endl;
            //     } else {
            //         if(c[CROP_left] == black) {
            //             set_movementDirection(MOVE_LEFT);
            //             if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"black in CROP_left ===> roll & pitch 15 degree to the " << debugLeft <<endl;
            //         } else if(c[CROP_right] == black) {
            //             set_movementDirection(MOVE_RIGHT);
            //             if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"black in CROP_right ===> roll & pitch 15 degree to the " << debugRight <<endl;
            //         } else {
            //             if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"unknown situation" << endl;
            //             if(get_movementDirection() == MOVE_LEFT) {
            //                 if(c[CROP_thirdQuarter] == white && c[CROP_right] == white) set_movementDirection(MOVE_RIGHT);
            //                 if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection is MOVE_LEFT ===> roll & pitch 15 degree to the " << debugRight << endl;
            //             } else {
            //                 if(c[CROP_left] == white && c[CROP_firstQuarter] == white) set_movementDirection(MOVE_LEFT);
            //                 if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection is MOVE_RIGHT ===> roll & pitch 15 degree to the " << debugLeft << endl;
            //             }
            //         }
            //     }
            // } else if(c[CROP_mid] == black_with_white) {
            //     if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "black_with_white in CROP_mid" << endl;
            //     if(!(c[CROP_left] == black || c[CROP_right] == black) && c[CROP_left] == c[CROP_right]) {
            //         if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"neither sides are black ===> pitch 15 degree to " << debugForward <<endl;
            //     } else {
            //         if(c[CROP_left] == black) {
            //             set_movementDirection(MOVE_LEFT);
            //             if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"black in CROP_left ===> roll & pitch 15 degree to the " << debugLeft <<endl;
            //         } else if(c[CROP_right] == black) {
            //             set_movementDirection(MOVE_RIGHT);
            //             if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"black in CROP_right ===> roll & pitch 15 degree to the " << debugRight <<endl;
            //         } else {
            //             if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"unknown situation" << endl;
            //             if(get_movementDirection() == MOVE_LEFT) {
            //                 if(c[CROP_thirdQuarter] == white && c[CROP_right] == white) set_movementDirection(MOVE_RIGHT);
            //                 if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection is MOVE_LEFT ===> roll & pitch 15 degree to the " << debugRight << endl;
            //             } else {
            //                 if(c[CROP_left] == white && c[CROP_firstQuarter] == white) set_movementDirection(MOVE_LEFT);
            //                 if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection is MOVE_RIGHT ===> roll & pitch 15 degree to the " << debugLeft << endl;
            //             }
            //         }
            //     }
            // } else if(c[CROP_mid] == black) {
            //     if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "black in CROP_mid" << endl;
            //     if(c[CROP_left] == white) {
            //         set_movementDirection(MOVE_RIGHT);
            //         if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"white in CROP_left ===> roll & pitch 15 degree to the " << debugRight <<endl;
            //     } else if(c[CROP_right] == white) {
            //         set_movementDirection(MOVE_LEFT);
            //         if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"white in CROP_right ===> roll & pitch 15 degree to the " << debugLeft <<endl;
            //     } else {
            //         if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"unknown situation" << endl;
            //         if(get_movementDirection() == MOVE_LEFT) {
            //             if(c[CROP_thirdQuarter] == white && c[CROP_right] == white) set_movementDirection(MOVE_RIGHT);
            //             if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection is MOVE_LEFT ===> roll & pitch 15 degree to the " << debugRight << endl;
            //         } else {
            //             if(c[CROP_left] == white && c[CROP_firstQuarter] == white) set_movementDirection(MOVE_LEFT);
            //             if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection is MOVE_RIGHT ===> roll & pitch 15 degree to the " << debugLeft << endl;
            //         }
            //     }
            // } else if(c[CROP_mid] == white) {
            //     if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "white in CROP_mid" << endl;
            //     if(!(c[CROP_left] == white || c[CROP_left] == unknown)) {
            //         set_movementDirection(MOVE_LEFT);
            //         if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"road in CROP_left ===> roll & pitch 15 degree to the " << debugLeft <<endl;
            //     } else if(!(c[CROP_right] == white || c[CROP_right] == unknown)) {
            //         set_movementDirection(MOVE_RIGHT);
            //         if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"road in CROP_right ===> roll & pitch 15 degree to the " << debugRight <<endl;
            //     } else {
            //         if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"unknown situation" << endl;
            //         if(get_movementDirection() == MOVE_LEFT) {
            //             if(c[CROP_thirdQuarter] == white && c[CROP_right] == white) set_movementDirection(MOVE_RIGHT);
            //             if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection is MOVE_LEFT ===> roll & pitch 15 degree to the " << debugRight << endl;
            //         } else {
            //             if(c[CROP_left] == white && c[CROP_firstQuarter] == white) set_movementDirection(MOVE_LEFT);
            //             if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection is MOVE_RIGHT ===> roll & pitch 15 degree to the " << debugLeft << endl;
            //         }
            //     }
            // } else {
            //     if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"unknown situation" << endl;
            //     if(get_movementDirection() == MOVE_LEFT) {
            //         if(c[CROP_thirdQuarter] == white && c[CROP_right] == white) set_movementDirection(MOVE_RIGHT);
            //         if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection is MOVE_LEFT ===> roll & pitch 15 degree to the " << debugRight << endl;
            //     } else {
            //         if(c[CROP_left] == white && c[CROP_firstQuarter] == white) set_movementDirection(MOVE_LEFT);
            //         if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection is MOVE_RIGHT ===> roll & pitch 15 degree to the " << debugLeft << endl;
            //     }
            // }
        */

        
        //send mavlink msgWrite

        //waitKey(100); //only for video !!!!!

        //if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<c[0]<<c[1]<<c[2]<<endl; //<<endl;
        if(DEBUG_CAM && (DEBUG_cam_line || DEBUG_cam_land)) imshow(debugID+"frame", frameToProcess);
    }
}
//FRONT EYE : obstacle + mission 1 + mission 2 (using bottom cam)
void frontEye() {

    //define variables
    Mat colorframe, hsv, mask_red1, mask_red2, mask_yellow;
    vector<vector<Point>> contours_red, contours_yellow;
    float area_red;
    float area_yellow;
    float max_red = 0.0;
    float max_yellow = 0.0;
    int iFrame = 0;
    int cam_fps = 0;
    float fpslive;
    clock_t start, end;
    string debugID = "\033[1;45mAVOID OBSTACLE ::\033[0m ";
    vector<decodedObject> decodedObjects;

    //webcam
    VideoCapture cap1(CAM_front, CAP_GSTREAMER);
    //VideoCapture cap1("video.mp4"); //video

    if (!cap1.isOpened()) {
        cerr << "Failed to open front camera." << endl;
        return;
    }
    
    cam_fps = cap1.get(CAP_PROP_FPS);

    if(DEBUG_TXT && DEBUG_txt_obs) cout <<"front camera frame per second: "<<cam_fps<<endl;
    while(true){
        // mavlink_msg_heartbeat_pack(1, 200, &msgObs, type, autopilot, base_mode, custom_mode, system_status);
        // mavlink_msg_to_send_buffer(bufObs, &msgObs);
        // write(serialPort, &bufObs, MAVLINK_MAX_PACKET_LEN);

        cap1 >> colorframe;
        //if(colorframe.empty()) break;
        if(!(get_stateMachine() == STATE_DEFAULT || get_stateMachine() == STATE_QR_getOutofBox || get_stateMachine() == STATE_OBS_decrease_alt || get_stateMachine() == STATE_OBS_increase_alt || get_stateMachine() == STATE_LINE_goTo_rightAlt) || isRerouting || stateMachineTimer > THRESH_STATE_TIMER) continue;
        start = clock();
        //resize and hsv
        if(isOnMissions()) {
            cout << "0" << endl;
            float lastYaw = QUAD_yaw;
            debugID = "\033[1;47mMISSION FRONT ::\033[0m ";
            while(get_stateMachine() == STATE_MISSION_1) {
                cout << "1" << endl;
                cap1 >> colorframe;
                resize(colorframe, colorframe ,Size(), SCALE_line_land_qr, SCALE_line_land_qr); //HERE obs -> qr=obs
                cvtColor(colorframe, hsv, COLOR_BGR2HSV);
                QR_decodeMissions(colorframe, decodedObjects);
                if(DEBUG_TXT && DEBUG_txt_mission) cout << debugID << "1 | subStateMachine = " << subStateMachine << " | subStateMachineTimer = " << subStateMachineTimer  << endl;
                if(DEBUG_CAM && DEBUG_cam_qr) QR_display(colorframe, decodedObjects);
                if(subStateMachine == 0) {
                    cout << "2" << endl;
                    if((MISSION_MEASURE_timer && subStateMachineTimer < THRESH_MISSION_1_timer_approachTower)
                        || (MISSION_MEASURE_lidar && QUAD_lidar < THRESH_MISSION_lidarDis_min)) {
                        cout << "3" << endl;
                        MAVLINK_moveForward(MAVLINK_VELOCITY_SLOW);
                        subStateMachineTimer++;
                    } else {
                        subStateMachine++;
                        subStateMachineTimer = 0;
                    }
                } else if(subStateMachine == 1) {
                    MAVLINK_setAlt(MAVLINK_ALT_lower);
                    //TEST if(MAVLINK_isAltInRange())
                    subStateMachine++;
                } else if(subStateMachine == 2) {
                    MAVLINK_setAlt(MAVLINK_ALT_higher);
                    //TEST if(MAVLINK_isAltInRange())
                    subStateMachine++;
                } else if(subStateMachine == 3) {
                    if((MISSION_MEASURE_timer && subStateMachineTimer < THRESH_MISSION_1_timer_travelLength/2)
                        || (MISSION_MEASURE_lidar && QUAD_lidar > THRESH_MISSION_lidarDis_max)) {
                        MAVLINK_moveRight(MAVLINK_VELOCITY_SLOW);
                        subStateMachineTimer++;
                    } else {
                        subStateMachine++;
                        subStateMachineTimer = 0;
                    }
                } else if(subStateMachine == 4) {
                    MAVLINK_setYaw(lastYaw-MAVLINK_YAW_90);
                    // if(MAVLINK_isYawInRange())
                    subStateMachine++;
                } else if(subStateMachine == 5) {
                    if((MISSION_MEASURE_timer && subStateMachineTimer < THRESH_MISSION_1_timer_travelWidth)
                        || (MISSION_MEASURE_lidar && QUAD_lidar < THRESH_MISSION_lidarDis_min)) {
                        MAVLINK_moveRight(MAVLINK_VELOCITY_SLOW);
                        subStateMachineTimer++;
                    } else {
                        subStateMachine++;
                        subStateMachineTimer = 0;
                    }
                } else if(subStateMachine == 6) {
                    MAVLINK_setAlt(MAVLINK_ALT_highest);
                    //TEST if(MAVLINK_isAltInRange())
                    subStateMachine++;
                } else if(subStateMachine == 7) {
                    if((MISSION_MEASURE_timer && subStateMachineTimer < THRESH_MISSION_1_timer_travelLength)
                        || (MISSION_MEASURE_lidar && QUAD_alt > THRESH_MISSION_DIS_altMax)) {
                        MAVLINK_moveForward(MAVLINK_VELOCITY_SLOW);
                        subStateMachineTimer++;
                    } else {
                        subStateMachine++;
                        subStateMachineTimer = 0;
                    }
                } else if(subStateMachine == 8) {
                    MAVLINK_setYaw(lastYaw+MAVLINK_YAW_90);
                    //TEST if(MAVLINK_isYawInRange())
                    subStateMachine++;
                } else if(subStateMachine == 9) {
                    MAVLINK_setAlt(MAVLINK_ALT_higher);
                    //TEST if(MAVLINK_isAltInRange())
                    subStateMachine++;
                } else if(subStateMachine == 10) {
                    if((MISSION_MEASURE_timer && subStateMachineTimer < THRESH_MISSION_1_timer_travelWidth)
                        || (MISSION_MEASURE_lidar && QUAD_lidar > THRESH_MISSION_lidarDis_max)) {
                        MAVLINK_moveRight(MAVLINK_VELOCITY_SLOW);
                        subStateMachineTimer++;
                    } else {
                        subStateMachine++;
                        subStateMachineTimer = 0;
                    }
                } else if(subStateMachine == 11) {
                    MAVLINK_setYaw(lastYaw);
                    //TEST if(MAVLINK_isYawInRange())
                    subStateMachine++;
                }
            }
            while(get_stateMachine() == STATE_MISSION_2) {
                cap1 >> colorframe;
                resize(colorframe, colorframe ,Size(), SCALE_line_land_qr, SCALE_line_land_qr); //HERE obs -> qr=obs
                cvtColor(colorframe, hsv, COLOR_BGR2HSV);
                QR_decodeMissions(colorframe, decodedObjects);
                if(DEBUG_TXT && DEBUG_txt_mission) cout << debugID << "2 | subStateMachine = " << subStateMachine << " | subStateMachineTimer " << subStateMachineTimer  << endl;
                if(DEBUG_CAM && DEBUG_cam_qr) QR_display(colorframe, decodedObjects);
                if(subStateMachine == 0) {
                    if((MISSION_MEASURE_timer && subStateMachineTimer < THRESH_MISSION_2_timer_approachTower)
                        || (MISSION_MEASURE_lidar && QUAD_lidar < THRESH_MISSION_lidarDis_min)) {
                        MAVLINK_moveForward(MAVLINK_VELOCITY_SLOW);
                        subStateMachineTimer++;
                    } else {
                        subStateMachine++;
                        subStateMachineTimer = 0;
                    }
                } else if(subStateMachine == 1) {
                    MAVLINK_setAlt(MAVLINK_ALT_highest);
                    if(QUAD_alt >= MAVLINK_alt) subStateMachine++;
                } else if(subStateMachine == 2) {
                    MAVLINK_setAlt(MAVLINK_ALT_lower);
                    if(QUAD_alt <= MAVLINK_alt) subStateMachine++;
                } else if(subStateMachine == 3) {
                    if((MISSION_MEASURE_timer && subStateMachineTimer < THRESH_MISSION_2_timer_travelHalfEdge)
                        || (MISSION_MEASURE_lidar && QUAD_lidar > THRESH_MISSION_lidarDis_max)) {
                        MAVLINK_moveRight(MAVLINK_VELOCITY_SLOW);
                        subStateMachineTimer++;
                    } else {
                        subStateMachine++;
                        subStateMachineTimer = 0;
                    }
                } else if(subStateMachine == 4) {
                    MAVLINK_setYaw(lastYaw-MAVLINK_YAW_90);
                    //TEST if(MAVLINK_isYawInRange())
                    subStateMachine++;
                } else if(subStateMachine == 5) {
                    if((MISSION_MEASURE_timer && subStateMachineTimer < THRESH_MISSION_2_timer_travelHalfEdge)
                        || (MISSION_MEASURE_lidar && QUAD_lidar < THRESH_MISSION_lidarDis_min)) {
                        MAVLINK_moveRight(MAVLINK_VELOCITY_SLOW);
                        subStateMachineTimer++;
                    } else {
                        subStateMachine++;
                        subStateMachineTimer = 0;
                    }
                } else if(subStateMachine == 6) {
                    MAVLINK_setAlt(MAVLINK_ALT_highest);
                    if(QUAD_alt >= MAVLINK_alt) subStateMachine++;
                } else if(subStateMachine == 7) {
                    if((MISSION_MEASURE_timer && subStateMachineTimer < THRESH_MISSION_2_timer_travelHalfEdge)
                        || (MISSION_MEASURE_lidar && QUAD_lidar > THRESH_MISSION_lidarDis_max)) {
                        MAVLINK_moveRight(MAVLINK_VELOCITY_SLOW);
                        subStateMachineTimer++;
                    } else {
                        subStateMachine++;
                        subStateMachineTimer = 0;
                    }
                } else if(subStateMachine == 8) {
                    MAVLINK_setYaw(lastYaw-MAVLINK_YAW_180);
                    //TEST if(MAVLINK_isYawInRange())
                    subStateMachine++;
                } else if(subStateMachine == 9) {
                    if((MISSION_MEASURE_timer && subStateMachineTimer < THRESH_MISSION_2_timer_travelHalfEdge)
                        || (MISSION_MEASURE_lidar && QUAD_lidar < THRESH_MISSION_lidarDis_min)) {
                        MAVLINK_moveRight(MAVLINK_VELOCITY_SLOW);
                        subStateMachineTimer++;
                    } else {
                        subStateMachine++;
                        subStateMachineTimer = 0;
                    }
                } else if(subStateMachine == 10) {
                    MAVLINK_setAlt(MAVLINK_ALT_lower);
                    if(QUAD_alt >= MAVLINK_alt) subStateMachine++;
                } else if(subStateMachine == 11) {
                    if((MISSION_MEASURE_timer && subStateMachineTimer < THRESH_MISSION_2_timer_travelHalfEdge)
                        || (MISSION_MEASURE_lidar && QUAD_lidar > THRESH_MISSION_lidarDis_max)) {
                        MAVLINK_moveRight(MAVLINK_VELOCITY_SLOW);
                        subStateMachineTimer++;
                    } else {
                        subStateMachine++;
                        subStateMachineTimer = 0;
                    }
                } else if(subStateMachine == 12) {
                    MAVLINK_setYaw(lastYaw+MAVLINK_YAW_90);
                    //TEST if(MAVLINK_isYawInRange())
                    subStateMachine++;
                } else if(subStateMachine == 13) {
                    if((MISSION_MEASURE_timer && subStateMachineTimer < THRESH_MISSION_2_timer_travelHalfEdge)
                        || (MISSION_MEASURE_lidar && QUAD_lidar < THRESH_MISSION_lidarDis_min)) {
                        MAVLINK_moveRight(MAVLINK_VELOCITY_SLOW);
                        subStateMachineTimer++;
                    } else {
                        subStateMachine++;
                        subStateMachineTimer = 0;
                    }
                } else if(subStateMachine == 14) {
                    MAVLINK_setAlt(MAVLINK_ALT_highest);
                    if(QUAD_alt >= MAVLINK_alt) subStateMachine++;
                } else if(subStateMachine == 15) {
                    if((MISSION_MEASURE_timer && subStateMachineTimer < THRESH_MISSION_2_timer_travelHalfEdge)
                        || (MISSION_MEASURE_lidar && QUAD_lidar > THRESH_MISSION_lidarDis_max)) {
                        MAVLINK_moveRight(MAVLINK_VELOCITY_SLOW);
                        subStateMachineTimer++;
                    } else {
                        subStateMachine++;
                        subStateMachineTimer = 0;
                    }
                } else if(subStateMachine == 16) {
                    MAVLINK_setYaw(lastYaw);
                    //TEST if(MAVLINK_isYawInRange())
                    subStateMachine++;
                } else if(subStateMachine == 17) {
                    MAVLINK_setAlt(MAVLINK_ALT_normal);
                    if(QUAD_alt <= MAVLINK_alt) subStateMachine++;
                }
            }
            while(get_stateMachine() == STATE_MISSION_3) {
                continue;
            } continue;
        }

        resize(colorframe, colorframe ,Size() ,SCALE_OBS, SCALE_OBS); //HERE obs -> qr=obs
        cvtColor(colorframe, hsv, COLOR_BGR2HSV);
        // Create masks for red and yellow objects
        inRange(hsv, lower_red1, upper_red1, mask_red1);
        inRange(hsv, lower_red2, upper_red2, mask_red2);
        inRange(hsv, lower_yellow, upper_yellow, mask_yellow);

        if(DEBUG_CAM && DEBUG_cam_obs) {
            // Find contours_black in the masks
            findContours(mask_red1, contours_red, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            findContours(mask_yellow, contours_yellow, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        }

        //area calculation
        area_yellow = countNonZero(mask_yellow);
        area_red = countNonZero(mask_red1) + countNonZero(mask_red2);
        
        //calculate live FPS
        end = clock();
        if(DEBUG_TXT && DEBUG_txt_obs && !(DEBUG_txt_line || DEBUG_txt_opf)) {
            fpslive = float(CLOCKS_PER_SEC)/(float(end)-float(start));
            cout << debugID << "area yellow = " << area_yellow <<"\tarea red = " << area_red << "\tfps = " << fpslive << endl;
        }
        //stop till alititude is changing
        if(area_yellow > THRESH_OBS_color){
            set_stateMachine(STATE_OBS_decrease_alt);
            if(DEBUG_TXT && DEBUG_txt_obs) cout << debugID << "\033[1;43myellow obstacle | Stop then decrease MAVLINK_alt. | \tarea yellow = " << area_yellow <<"\tfps = "<<fpslive << "\033[0m" << endl;
            //TEST do MAVLINK_setAlt(MAVLINK_ALT_low); while(!MAVLINK_isAltInRange());
            imsg = MSG_DEFAULT;
            set_stateMachine(STATE_LINE_underAboveObs);
        }
        if(area_red > THRESH_OBS_color){
            set_stateMachine(STATE_OBS_increase_alt);
            if(DEBUG_TXT && DEBUG_txt_obs) cout << debugID << "\033[1;41mred obstacle | Stop then increase MAVLINK_alt\tarea red = " << area_red <<"\tfps = " <<fpslive << "\033[0m" << endl;
            //TEST do MAVLINK_setAlt(MAVLINK_ALT_high); while(!MAVLINK_isAltInRange());
            imsg = MSG_DEFAULT;
            set_stateMachine(STATE_LINE_underAboveObs);
        }

        if(DEBUG_CAM && DEBUG_cam_obs) {
            // Draw rectangles around the detected objects
            for (const auto& contour : contours_red) {
                Rect rect = boundingRect(contour);
                rectangle(colorframe, rect, Scalar(0, 0, 255), 2);
            } 
            for (const auto& contour : contours_yellow) {
                Rect rect = boundingRect(contour);
                rectangle(colorframe, rect, Scalar(0, 255, 255), 2);
            } if(DEBUG_CAM && DEBUG_cam_obs) imshow(debugID+"Red and Yellow Object Detection", colorframe);
        }
    } cap1.release();
}