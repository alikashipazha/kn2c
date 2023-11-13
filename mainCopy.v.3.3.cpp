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

    // add dy < 0 qr +
    // obs states into forward follow line +
    // if line no continue +
    // decode center -> dir + center +
    // last state +
    // 3 main line codes -
    // finding center of qr code +
    // follow line and decrease increase alt fix <- thresh_haugh*zarib -----
    // states for geting back to default alt MASMALI
    // state thread conflicts ++++

// ############ Libraries #######
    // ##                      ##
    // ##        comment       ##
    // ##                      ##
    // ##########################

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
    using namespace cv;
    using namespace std;
    using namespace zbar; 

// ######### Consts & Flags #####
    // ##                      ##
    // ##        comment       ##
    // ##                      ##
    // ##########################

//CAMERAS | 0 : sabz , 1 : abi, qermez
#define CAM_front 0
#define CAM_bottom !CAM_front

//SCALE FACTORS
#define SCALE_opf 0.1
#define SCALE_line 0.3
#define SCALE_land 0.3
#define SCALE_qr 0.5
#define SCALE_obs 0.2

//COLOR CALIBRES

//color calibre red1
#define COLOR_RED1_lower_h 0
#define COLOR_RED1_upper_h 10
#define COLOR_RED1_lower_s 140 
#define COLOR_RED1_upper_s 255
#define COLOR_RED1_lower_v 140 
#define COLOR_RED1_upper_v 255
//color calibre red2
#define COLOR_RED2_lower_h 170
#define COLOR_RED2_upper_h 180
#define COLOR_RED2_lower_s 140 
#define COLOR_RED2_upper_s 255
#define COLOR_RED2_lower_v 140 
#define COLOR_RED2_upper_v 255
//color calibre yellow
#define COLOR_YELLOW_lower_h 20
#define COLOR_YELLOW_upper_h 35
#define COLOR_YELLOW_lower_s 110
#define COLOR_YELLOW_upper_s 255
#define COLOR_YELLOW_lower_v 120
#define COLOR_YELLOW_upper_v 255
//color calibre black
#define COLOR_BLACK_lower_h 0
#define COLOR_BLACK_lower_s 0
#define COLOR_BLACK_lower_v 0
#define COLOR_BLACK_upper_h 180
#define COLOR_BLACK_upper_s 255
#define COLOR_BLACK_upper_v 60
//color calibre white
#define COLOR_WHITE_lower_h 0
#define COLOR_WHITE_lower_s 0
#define COLOR_WHITE_lower_v 90
#define COLOR_WHITE_upper_h 255
#define COLOR_WHITE_upper_s 10
#define COLOR_WHITE_upper_v 255
//color calibre blue
#define COLOR_BLUE_lower_h 100
#define COLOR_BLUE_upper_h 130
#define COLOR_BLUE_lower_s 120
#define COLOR_BLUE_upper_s 255
#define COLOR_BLUE_lower_v 100
#define COLOR_BLUE_upper_v 255

//THRESHOLDS
#define THRESH_STATE_TIMER 5000
//obstacle thresholds
#define THRESH_OBS_color 1000
#define THRESH_OBS_alt_yellow 20
#define THRESH_OBS_alt_red 20
//line thresholds
#define THRESH_LINE_d_pixles_black 2000
#define THRESH_LINE_d_pixles_white 1000
#define THRESH_LINE_c_pixles_black 2000
#define THRESH_LINE_c_pixles_white_inBlack 1500
#define THRESH_LINE_c_area_white_offroad 2000
#define THRESH_LINE_c_area_whiteLine_offset 50 
#define THRESH_LINE_c_area_whiteLine 2000
#define THRESH_LINE_justLand_pixles_black 100
#define THRESH_LINE_justLand_pixles_blue 100
#define THRESH_LINE_hough_dx 50
//land thresholds
#define THRESH_LAND_area 10000
#define THRESH_LAND_dis_x 150
#define THRESH_LAND_dis_y 150
//qr thresholds
#define THRESH_QR_timer_getOutOfBox 2000
#define THRESH_QR_dx 50
#define THRESH_QR_dy 50
//DEBUG FLAGS

//text debug flags
#define DEBUG_TXT 1 // to del
#define DEBUG_txt_opf 0
#define DEBUG_txt_line 1
#define DEBUG_txt_land 0
#define DEBUG_txt_qr 1
#define DEBUG_txt_obs 1
//camera debug flags | activate one camera debugger only
#define DEBUG_CAM 1 // to del
#define DEBUG_cam_opf 0
#define DEBUG_cam_line 1
#define DEBUG_cam_land 0
#define DEBUG_cam_qr 0
#define DEBUG_cam_obs 0

// ######### KN2C Variables #####
    // ##                      ##
    // ##        comment       ##
    // ##                      ##
    // ##########################

typedef struct {
  string type;
  string data;
  vector <Point> location;
} decodedObject;
typedef enum {
    unknown,
    white,
    black,
    black_with_white,
    black_with_white_line,
} crop_type;
typedef enum {
    was_going_right,
    was_going_left,
} previous_direction;
typedef enum {
    STATE_DEFAULT, //0
    STATE_OBS_decrease_alt, //1
    STATE_OBS_increase_alt, //2
    STATE_QR_turnTo_dir, //3
    STATE_QR_goTo_center, //4
    STATE_QR_getOutOf_box, //5
    STATE_LINE_getBackInto_rout, //6
    STATE_LINE_landing, //7
    STATE_LINE_underAboveObs, //8
    STATE_LINE_goTo_rightAlt, //9
} state_machine;

mutex frameMutex;
Mat frame;
int width, height;
int cam_fps;
int mission = 0;
int stateMachineTimer = 0;
const string debugForward = "\033[1;42mforward\033[0m ";
const string debugLeft = "\033[1;42mleft\033[0m ";
const string debugRight = "\033[1;42mright\033[0m ";
bool isLandingState = false;
bool isScanningState = false;
bool isRerouting = false;
previous_direction _previousDirection = was_going_left;
state_machine _stateMachine = STATE_DEFAULT;
state_machine lastStateMachine = _stateMachine;

void set_stateMachine(state_machine state);
state_machine get_stateMachine();
void opticalFlow();
void obstacle();
void set_previousDirection(previous_direction direction);
previous_direction get_previousDirection();
void lineXqrXland();
int readData(string &data);
void display(Mat &im, vector<decodedObject>&decodedObjects);
void decode(Mat &im, vector<decodedObject>&decodedObjects);

// ############ KN2C Main #######
    // ##                      ##
    // ##        comment       ##
    // ##                      ##
    // ##########################

int main() {

    if(DEBUG_TXT && !(DEBUG_txt_opf || DEBUG_txt_line || DEBUG_txt_qr || DEBUG_txt_obs || DEBUG_txt_land)) {
        cerr << "activating AT LEAST ONE text debugger is necessary when DEBUG_TXT is 1 (Ln 32)" << endl;
        return -1;
    }
    if(DEBUG_CAM && !(DEBUG_cam_opf ^ DEBUG_cam_line ^ DEBUG_cam_qr ^ DEBUG_cam_obs ^ DEBUG_cam_land)) {
        cerr << "activating ONLY ONE camera debugger is necessary when DEBUG_CAM is 1 (Ln 38)" << endl;
        return 0;
    }
    while(mission > 4 || mission < 0) {
        cout << "input mission : ";
        cin >> mission;
        if(mission == 4) isLandingState = true;
    }

    //creat threads
    // thread FollowLine(lineXqrXland);
    // thread OpticalFlow(opticalFlow);
    // thread AvoidObstacle(obstacle);

    // //joint threads
    // if(FollowLine.joinable()) FollowLine.join();
    // if(OpticalFlow.joinable()) OpticalFlow.join();
    // if(AvoidObstacle.joinable()) AvoidObstacle.join();
    lineXqrXland();
    return 1; 
}

// ######### KN2C Functions #####
    // ##                      ##
    // ##        comment       ##
    // ##                      ##
    // ##########################


state_machine get_stateMachine() {
    static int timer = 0;
    if(_stateMachine == STATE_QR_getOutOf_box) timer++;
    if(timer > THRESH_QR_timer_getOutOfBox) {
        timer = 0;
        set_stateMachine(STATE_DEFAULT);
    } return _stateMachine;
}
void set_stateMachine(state_machine state) {
    if(get_stateMachine() != state) lastStateMachine = get_stateMachine();
    _stateMachine = state;
    if(!(get_stateMachine() == STATE_LINE_getBackInto_rout || state == STATE_LINE_getBackInto_rout)) stateMachineTimer = 0;
    if(DEBUG_TXT) cout << "$ stateMachine is set to " << get_stateMachine() << endl;
}

void obstacle() {

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

    // Define the lower and upper bounds for red color in HSV
    Scalar lower_red1(COLOR_RED1_lower_h, COLOR_RED1_lower_s, COLOR_RED1_lower_v);
    Scalar upper_red1(COLOR_RED1_upper_h, COLOR_RED1_upper_s, COLOR_RED1_upper_v);
    Scalar lower_red2(COLOR_RED2_lower_h, COLOR_RED2_lower_s, COLOR_RED2_lower_v);
    Scalar upper_red2(COLOR_RED2_upper_h, COLOR_RED2_upper_s, COLOR_RED2_upper_v);
    // Define the lower and upper bounds for yellow color in HSV
    Scalar lower_yellow(COLOR_YELLOW_lower_h, COLOR_YELLOW_lower_s, COLOR_YELLOW_lower_v);
    Scalar upper_yellow(COLOR_YELLOW_upper_h, COLOR_YELLOW_upper_s, COLOR_YELLOW_upper_v);

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
        cap1 >> colorframe;
        //if(colorframe.empty()) break;
        if(!(get_stateMachine() == STATE_DEFAULT || get_stateMachine() == STATE_QR_getOutOf_box || get_stateMachine() == STATE_OBS_decrease_alt || get_stateMachine() == STATE_OBS_increase_alt || get_stateMachine() == STATE_LINE_goTo_rightAlt) || isRerouting || stateMachineTimer > THRESH_STATE_TIMER) continue;
        start = clock();
        //resize and hsv
        resize(colorframe, colorframe ,Size() ,SCALE_obs, SCALE_obs);
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
        fpslive = float(CLOCKS_PER_SEC)/(float(end)-float(start));
        if(DEBUG_TXT && DEBUG_txt_obs && !(DEBUG_txt_line || DEBUG_txt_opf)) cout << debugID << "area yellow = " << area_yellow <<"\tarea red = " << area_red << "\tfps = " << fpslive << endl;
        //stop till alititude is changing
        if(area_yellow > THRESH_OBS_color){
            set_stateMachine(STATE_OBS_decrease_alt);
            if(DEBUG_TXT && DEBUG_txt_obs) cout << debugID << "\033[1;43myellow obstacle | Stop then decrease altitude. | \tarea yellow = " << area_yellow <<"\tfps = "<<fpslive << "\033[0m" << endl;
            //while alt > THRESH_OBS_alt_yellow decrease alt ->go to alt
            set_stateMachine(STATE_LINE_underAboveObs);
        }
        if(area_red > THRESH_OBS_color){
            set_stateMachine(STATE_OBS_increase_alt);
            if(DEBUG_TXT && DEBUG_txt_obs) cout << debugID << "\033[1;41mred obstacle | Stop then increase altitude\tarea red = " << area_red <<"\tfps = " <<fpslive << "\033[0m" << endl;
            //while alt < THRESH_OBS_alt_red increase  alt -> go to alt
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

void opticalFlow() {

    //make objects(variables)
    Mat old_frame,old_resize, old_gray;
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

    // Create some random colors
    if(DEBUG_CAM && DEBUG_cam_opf) {
        for(int i = 0; i < 100; i++) {
            int r = rng.uniform(0, 256);
            int g = rng.uniform(0, 256);
            int b = rng.uniform(0, 256);
            colors.push_back(Scalar(r,g,b));
        }
    }

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
    //cap0.set(CAP_PROP_FPS, 5);
    cap0 >> old_frame; // Capture a frame
    resize(old_frame, old_resize ,Size() , SCALE_opf, SCALE_opf);
    //pi camera color map NOT
    cvtColor(old_resize, old_resize, COLOR_BGR2RGB);
    cvtColor(old_resize, old_gray, COLOR_BGR2GRAY);
    // cvtColor(old_frame, old_gray, COLOR_BGR2GRAY);
    
    if(DEBUG_TXT && (DEBUG_txt_opf || DEBUG_txt_line || DEBUG_txt_qr)) cout <<"bottom camera frame per second: "<<cam_fps<<endl;
    TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);

    while(true){
        if(DEBUG_TXT) cout << "$ mission " << mission << " | stateMachine " << get_stateMachine() << " | lastStateMachine " << lastStateMachine << endl;
        {
            lock_guard<mutex> lock(frameMutex);
            cap0 >> frame;
        } //get new frame
      
        start = clock();
        resize(frame, frame_resize ,Size() , SCALE_opf, SCALE_opf);
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

        //calculate live FPS
        end = clock();
        fpslive = float(CLOCKS_PER_SEC)/(float(end)-float(start));
        
        //mavlink massage
        if(DEBUG_TXT && DEBUG_txt_opf) cout << debugID <<"right(dx)= "<<totalMovementx<<"   forward(dy)="<<totalMovementy <<"     flow quality = "<<flow_quality<<"     fps = "<<fpslive<<endl; //<<endl;
        if(DEBUG_CAM && DEBUG_cam_opf) {
            Point org(50, 50); 
            putText(frame_resize, to_string(fpslive), org , FONT_HERSHEY_DUPLEX, 1 , Scalar(200, 0, 150),1.5);
            imshow(debugID+"Frame", frame_resize);
        }  //waitKey(100); //only for video !!!!!
    } cap0.release();
}
//line
void set_previousDirection(previous_direction direction) {
    _previousDirection = direction;
}
previous_direction get_previousDirection() {
    return _previousDirection;
}
void lineXqrXland() {

    VideoCapture cap(0);
    if (!cap.isOpened()) {
        cerr << "Failed to open bottom camera." << endl;
        return;
    }
    //define variables
    int area_max_i_black, area_max_i_white;
    float area_max_black, area_max_white;
    Mat xframe,frameToProcess, toblur, hsv, mask_black, gray, thr, mask_white;
    Mat canny;
    Mat crop1, crop2, crop3, crop_black, crop_white, mask2;
    vector<vector<Point>> contours_black, contours_blue, contours_white;
    Scalar lower_black(COLOR_BLACK_lower_h, COLOR_BLACK_lower_s, COLOR_BLACK_lower_v);
    Scalar upper_black(COLOR_BLACK_upper_h, COLOR_BLACK_upper_s, COLOR_BLACK_upper_v);
    Scalar lower_white(COLOR_WHITE_lower_h, COLOR_WHITE_lower_s, COLOR_WHITE_lower_v);
    Scalar upper_white(COLOR_WHITE_upper_h, COLOR_WHITE_upper_s, COLOR_WHITE_upper_v);
    Scalar lower_blue(COLOR_BLUE_lower_h, COLOR_BLUE_lower_s, COLOR_BLUE_lower_v);
    Scalar upper_blue(COLOR_BLUE_upper_h, COLOR_BLUE_upper_s, COLOR_BLUE_upper_v);
    Point2i p0;
    float left_right;
    int blackPixles, whitePixles;
    string debugID;
    vector<decodedObject> decodedObjects;
    Mat mask_blue;
    clock_t start, end;
    float area_blue;
    float fpslive;

    float max;
    float ch = (float)(height/2);//(float)(height)/2.0;
    float cw = (float)(width/2);//(float)(width)/2.0;
    // int ch = height/2;
    // int cw = width/2;
    //int greatestBlueContour;
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
    vector<Vec4i> linesP; // will hold the results of the detection

    
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
    Vec4i l;
    Point2i center,line_center;
    stateMachineTimer = 0;
    while (true) {
        ch = (float)(height/2)*SCALE_line;
        cw = (float)(width/2)*SCALE_line;
        // cout << (float)(width)/2.0 << "  " << (float)(height)/2.0 << endl;
        // cout << cw << "  " << ch << endl; // KHALALJALEGH
        stateMachineTimer++;
        if(DEBUG_TXT) cout << "$ stateMachineTimer = " << stateMachineTimer << endl;
        if(stateMachineTimer > THRESH_STATE_TIMER) {
            cout << "$ state machine time out." << endl;
            set_stateMachine(STATE_LINE_goTo_rightAlt);
        }
        if(get_stateMachine() == STATE_LINE_goTo_rightAlt) {
            //while myAlt not at THRESH_ALT -> go to THRESH_ALT
            set_stateMachine(STATE_DEFAULT);
        }
        decodedObjects.clear();
        cap >> frame;
        {
            //lock_guard<mutex> lock(frameMutex);
            xframe = frame.clone(); // Store the frame in the shared variable
        }
            
        if(xframe.empty()) continue;

        resize(xframe, frameToProcess ,Size() ,SCALE_land, SCALE_land);
        //pi camera color map NOT
        cvtColor(frameToProcess, frameToProcess, COLOR_BGR2RGB);
        cvtColor(frameToProcess, hsv, COLOR_BGR2HSV);
        
        inRange(hsv, lower_blue, upper_blue, mask_blue);
        findContours(mask_blue, contours_blue, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        
        if(get_stateMachine() == STATE_LINE_landing) {
            debugID = "\033[1;44mLANDING ::\033[0m ";
            int i = 0;
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
                    if(DEBUG_TXT && DEBUG_txt_land) cout << debugID << "\033[1;44mblue H | Stop then decrease altitude\tlargest blue contour area = " << greatestBlueContourArea << "\tdx dy = " << dx << dy << "\033[0m" << endl;
                    break;
                }
            } continue;
        }       

        debugID = "\033[1;42mFOLLOW LINE ::\033[0m ";
        //fine black road
        //resize(xframe, frameToProcess ,Size() ,SCALE_line, SCALE_line);
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
        if(get_stateMachine() != STATE_QR_getOutOf_box) {
            decode(frameToProcess, decodedObjects);
            if(DEBUG_CAM && DEBUG_cam_qr) display(frameToProcess, decodedObjects);
        }
        if(get_stateMachine() == STATE_QR_goTo_center || get_stateMachine() == STATE_QR_turnTo_dir) continue;
        
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
            // left_right = (((p0.x - (width/2)*SCALE_line))/ ((width/2)*SCALE_line))*10;

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
            crop_black = mask_black(Rect(int((width/crop_c_len)*(i)*SCALE_line),0,int((width/crop_c_len)*SCALE_line),int(height*SCALE_line)));
            crop_white = mask_white(Rect(int((width/crop_c_len)*(i)*SCALE_line),0,int((width/crop_c_len)*SCALE_line),int(height*SCALE_line)));
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

        if(linesP.size() > 1){    
            line_center.x = 0;
            line_center.y = 0;
            posXmin = 10000.0;
            posXmax = -1.0;
            // Draw the lines
            for( size_t i = 0; i < linesP.size(); i++ ) {
                l = linesP[i];
                //only vertical lines
                if(4*abs(l[0]-l[2])<abs(l[3]-l[1])){  
                    if(DEBUG_CAM && DEBUG_cam_line) line(frame, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,255,0), 3, LINE_AA);  
                    float posX = (float)(l[0]+l[2])/2.0;
                    //if(posX > 384) return;
                    if(posX > posXmax) posXmax = posX;
                    if(posX < posXmin) posXmin = posX;
                    if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "l[0] " << l[0] << " l[2] " << l[2] << " posX " << posX << " min " << posXmin << " max "<< posXmax << endl;
                    
                    line_center.x += ((l[0]+l[2])/2);
                    if(l[3]<l[1]) line_center.y += l[3];
                    if(l[1]<l[3]) line_center.y += l[1];
                }
            } line_center.x /= linesP.size(); line_center.y /= linesP.size();
            if(DEBUG_CAM && DEBUG_cam_line) {
                circle(frame, line_center, 5, Scalar(0, 0, 255), -1);
                circle(frame, center, 5, Scalar(255, 0, 0), -1);
            }
            if(!(get_stateMachine() == STATE_LINE_getBackInto_rout || get_stateMachine() == STATE_OBS_decrease_alt || get_stateMachine() == STATE_OBS_increase_alt) && (cw > posXmin && cw < posXmax)) {
                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "pitch 15 degree to " << debugForward << endl;
            } else {
                set_stateMachine(STATE_LINE_getBackInto_rout);
                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "cw " << cw << " cw > posXmin " << (cw > posXmin) << "\tcw < posXmax " << (cw < posXmax) << endl;
                dxH = cw - line_center.x;
                dyH = ch - line_center.y;
                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "dxH = " << dxH << "\tdyH = " << dyH << endl;
                if(dxH > THRESH_LINE_hough_dx || d[CROP_firstQuarter][WHITE]){
                    isRerouting = true;
                    set_previousDirection(was_going_right);
                    if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "pitch & roll 15 degree to " << debugRight << endl;
                } else if(dxH < -THRESH_LINE_hough_dx || d[CROP_thirdQuarter][WHITE]){
                    isRerouting = true;
                    set_previousDirection(was_going_left);
                    if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "pitch & roll 15 degree to " << debugLeft << endl;
                } else {
                    isRerouting = false;
                    set_stateMachine(lastStateMachine); //here last state
                }
            }
        } else if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "go " << debugForward << " khoda bozorgeh" << endl;
        // if(!(get_stateMachine() == STATE_LINE_getBackInto_rout || get_stateMachine() == STATE_OBS_decrease_alt || get_stateMachine() == STATE_OBS_increase_alt) && (d[CROP_mid][BLACK] || (d[CROP_mid][BLACK] && d[CROP_mid][BLACK]))) {
        //     if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "pitch 15 degree to " << debugForward << endl;
        // } else {
        //     set_stateMachine(STATE_LINE_getBackInto_rout);
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
        //     // decode(frame,decodedObjects); //here
        //     // if(decodedObjects.empty()) {
        //         if(dxH > THRESH_LINE_hough_dx || d[CROP_firstQuarter][WHITE]){
        //             set_previousDirection(was_going_right);
        //             if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "pitch & roll 15 degree to " << debugRight << endl;
        //         } else if(dxH < -THRESH_LINE_hough_dx || d[CROP_thirdQuarter][WHITE]){
        //             set_previousDirection(was_going_left);
        //             if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "pitch & roll 15 degree to " << debugLeft << endl;
        //         } else set_stateMachine(lastStateMachine); //here last state
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
        //             set_previousDirection(was_going_left);
        //             if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"black in CROP_left ===> roll & pitch 15 degree to the " << debugLeft <<endl;
        //         } else if(c[CROP_right] == black) {
        //             set_previousDirection(was_going_right);
        //             if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"black in CROP_right ===> roll & pitch 15 degree to the " << debugRight <<endl;
        //         } else {
        //             if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"unknown situation" << endl;
        //             if(get_previousDirection() == was_going_left) {
        //                 if(c[CROP_thirdQuarter] == white && c[CROP_right] == white) set_previousDirection(was_going_right);
        //                 if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection is was_going_left ===> roll & pitch 15 degree to the " << debugRight << endl;
        //             } else {
        //                 if(c[CROP_left] == white && c[CROP_firstQuarter] == white) set_previousDirection(was_going_left);
        //                 if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection is was_going_right ===> roll & pitch 15 degree to the " << debugLeft << endl;
        //             }
        //         }
        //     }
        // } else if(c[CROP_mid] == black_with_white) {
        //     if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "black_with_white in CROP_mid" << endl;
        //     if(!(c[CROP_left] == black || c[CROP_right] == black) && c[CROP_left] == c[CROP_right]) {
        //         if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"neither sides are black ===> pitch 15 degree to " << debugForward <<endl;
        //     } else {
        //         if(c[CROP_left] == black) {
        //             set_previousDirection(was_going_left);
        //             if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"black in CROP_left ===> roll & pitch 15 degree to the " << debugLeft <<endl;
        //         } else if(c[CROP_right] == black) {
        //             set_previousDirection(was_going_right);
        //             if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"black in CROP_right ===> roll & pitch 15 degree to the " << debugRight <<endl;
        //         } else {
        //             if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"unknown situation" << endl;
        //             if(get_previousDirection() == was_going_left) {
        //                 if(c[CROP_thirdQuarter] == white && c[CROP_right] == white) set_previousDirection(was_going_right);
        //                 if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection is was_going_left ===> roll & pitch 15 degree to the " << debugRight << endl;
        //             } else {
        //                 if(c[CROP_left] == white && c[CROP_firstQuarter] == white) set_previousDirection(was_going_left);
        //                 if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection is was_going_right ===> roll & pitch 15 degree to the " << debugLeft << endl;
        //             }
        //         }
        //     }
        // } else if(c[CROP_mid] == black) {
        //     if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "black in CROP_mid" << endl;
        //     if(c[CROP_left] == white) {
        //         set_previousDirection(was_going_right);
        //         if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"white in CROP_left ===> roll & pitch 15 degree to the " << debugRight <<endl;
        //     } else if(c[CROP_right] == white) {
        //         set_previousDirection(was_going_left);
        //         if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"white in CROP_right ===> roll & pitch 15 degree to the " << debugLeft <<endl;
        //     } else {
        //         if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"unknown situation" << endl;
        //         if(get_previousDirection() == was_going_left) {
        //             if(c[CROP_thirdQuarter] == white && c[CROP_right] == white) set_previousDirection(was_going_right);
        //             if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection is was_going_left ===> roll & pitch 15 degree to the " << debugRight << endl;
        //         } else {
        //             if(c[CROP_left] == white && c[CROP_firstQuarter] == white) set_previousDirection(was_going_left);
        //             if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection is was_going_right ===> roll & pitch 15 degree to the " << debugLeft << endl;
        //         }
        //     }
        // } else if(c[CROP_mid] == white) {
        //     if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "white in CROP_mid" << endl;
        //     if(!(c[CROP_left] == white || c[CROP_left] == unknown)) {
        //         set_previousDirection(was_going_left);
        //         if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"road in CROP_left ===> roll & pitch 15 degree to the " << debugLeft <<endl;
        //     } else if(!(c[CROP_right] == white || c[CROP_right] == unknown)) {
        //         set_previousDirection(was_going_right);
        //         if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"road in CROP_right ===> roll & pitch 15 degree to the " << debugRight <<endl;
        //     } else {
        //         if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"unknown situation" << endl;
        //         if(get_previousDirection() == was_going_left) {
        //             if(c[CROP_thirdQuarter] == white && c[CROP_right] == white) set_previousDirection(was_going_right);
        //             if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection is was_going_left ===> roll & pitch 15 degree to the " << debugRight << endl;
        //         } else {
        //             if(c[CROP_left] == white && c[CROP_firstQuarter] == white) set_previousDirection(was_going_left);
        //             if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection is was_going_right ===> roll & pitch 15 degree to the " << debugLeft << endl;
        //         }
        //     }
        // } else {
        //     if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"unknown situation" << endl;
        //     if(get_previousDirection() == was_going_left) {
        //         if(c[CROP_thirdQuarter] == white && c[CROP_right] == white) set_previousDirection(was_going_right);
        //         if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection is was_going_left ===> roll & pitch 15 degree to the " << debugRight << endl;
        //     } else {
        //         if(c[CROP_left] == white && c[CROP_firstQuarter] == white) set_previousDirection(was_going_left);
        //         if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection is was_going_right ===> roll & pitch 15 degree to the " << debugLeft << endl;
        //     }
        // }

        
        //send mavlink msg

        //waitKey(100); //only for video !!!!!

        //if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<c[0]<<c[1]<<c[2]<<endl; //<<endl;
        if(DEBUG_CAM && (DEBUG_cam_line || DEBUG_cam_land)) imshow(debugID+"frame", frameToProcess);
    }
    cap.release();
}

//qr
int readData(string &data) { // -> decode
    int index = int(data.back())-48;
    string debugID = "\033[1;47mQR :: readData ::\033[0m ";

    if(index - mission == 1) {
        mission++;
        if(DEBUG_TXT && DEBUG_txt_qr) cout << debugID << "new mission = " << mission << endl;
    }
    if(DEBUG_TXT && DEBUG_txt_qr) cout << debugID << "returning " << data[mission*2] << endl;
    return data[mission*2];
}
void decode(Mat &im, vector<decodedObject>&decodedObjects) { // -> lineXqrXland

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
    string debugID = "\033[1;47mQR :: decode ::\033[0m ";

    float ch = (height/2)*SCALE_qr;
    float cw = (width/2)*SCALE_qr;
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
        if(dy < 0 && get_stateMachine() != STATE_QR_turnTo_dir) return;
        set_stateMachine(STATE_QR_goTo_center);
        if(DEBUG_TXT && DEBUG_txt_qr) cout << debugID << "dx = " << dx << " dy = " << dy << endl;
        if(abs(dx) > THRESH_QR_dx) {
            if(dx < 0) {
                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "pitch & roll 15 degree to " << debugRight << endl;
            } else {
                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "pitch & roll 15 degree to " << debugLeft << endl;
            }
        } else if(dy > THRESH_QR_dy) {
            if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "pitch & roll 15 degree to " << debugForward << endl;
        } else {
            set_stateMachine(STATE_QR_turnTo_dir);
            switch(readData(obj.data)) { //state get out of box + turn to dir
                case 'N':
                    // if myCompass == compass then set_state getOutOf box
                    if(DEBUG_TXT && DEBUG_txt_qr) cout << debugID << "compass 0 until it's 0 -+ 5" << endl;
                    break;
                case 'E':
                    // while compass then turn ... or if myCompass == compass then set_state getOutOf box
                    if(DEBUG_TXT && DEBUG_txt_qr) cout << debugID << "compass 90 until it's 90 -+ 5" << endl;
                    break;
                case 'S':
                    // while compass then turn ... or if myCompass == compass then set_state getOutOf box
                    if(DEBUG_TXT && DEBUG_txt_qr) cout << debugID << "compass 180 until it's 180 -+ 5" << endl;
                    break;
                case 'W':
                    // while compass then turn ... or if myCompass == compass then set_state getOutOf box
                    if(DEBUG_TXT && DEBUG_txt_qr) cout << debugID << "compass 270 until it's 270 -+ 5" << endl;
                    break;
                default:
                    // while compass then turn ... or if myCompass == compass then set_state getOutOf box
                    if(DEBUG_TXT && DEBUG_txt_qr) cout << debugID << "compass 0 until it's 0 -+ 5. to the landing" << endl;
                    isLandingState = true;
                    break;
            } set_stateMachine(STATE_QR_getOutOf_box);
        }
    } if(DEBUG_TXT) cout << endl;
}
void display(Mat &im, vector<decodedObject> &decodedObjects) { // -> lineXqrXland
    
    if(!(DEBUG_CAM && DEBUG_cam_qr) || isLandingState) return;
    string debugID = "\033[1;47mQR :: display ::\033[0m ";
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
