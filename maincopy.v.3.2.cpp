// TODO list:
    // line blur -
    // landing +
    // red color calibre +
    // line +
    // medianBlur() -
    // landline or land line -

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
#define CAM_front 1
#define CAM_bottom !CAM_front

//SCALE FACTORS
#define SCALE_opf 0.1
#define SCALE_line 0.5 // == land
#define SCALE_land 0.5 //0.3
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
#define COLOR_BLACK_upper_v 40
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

//obstacle thresholds
#define THRESH_obs_color 1000
//line thresholds
#define THRESH_line_pixles_black 2000
#define THRESH_line_pixles_white_inBlack 1500
#define THRESH_line_area_white_offroad 2000
#define THRESH_line_area_whiteLine_offset 50 
#define THRESH_line_area_whiteLine 2000
//land thresholds
#define THRESH_land_area 10000
#define THRESH_land_dis_x 150
#define THRESH_land_dis_y 150

//DEBUG FLAGS

//text debug flags
#define DEBUG_TXT 1 // to del
#define DEBUG_txt_opf 0
#define DEBUG_txt_line 1
#define DEBUG_txt_land 0
#define DEBUG_txt_qr 0
#define DEBUG_txt_obs 0
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

mutex frameMutex;
Mat frame;
int width, height;
int cam_fps;
int mission = 0;
bool isLandingState = false;
previous_direction _previousDirection = was_going_left;

void opticalFlow();
void obstacle();
void set_previousDirection(previous_direction direction);
previous_direction get_previousDirection();
void lineXqr();
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


    VideoCapture cap(0);
    if (!cap.isOpened()) {
        cerr << "Failed to open bottom camera." << endl;
        return -12;
    }
    //define variables
    int area_max_i_black, area_max_i_white;
    float area_max_black, area_max_white;
    Mat xframe,frameToProcess, toblur, hsv, mask_black, gray, thr, mask_white;
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
    width = static_cast<int>(cap.get(3));
    height = static_cast<int>(cap.get(4));
    // Mat canny_output;
    // vector<vector<Point> > contours_black;
    // vector<Vec4i> hierarchy;
    // vector<Moments> mu;
    // Moments m;
    // Point center;

    float max;
    float ch = height/2;
    float cw = width/2;
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

    
    const int crop_c_len = 5;
    crop_type c[crop_c_len];
    const int CROP_left = 0;
    const int CROP_firstQuarter = (crop_c_len-1)/4;
    const int CROP_mid = (crop_c_len-1)/2;
    const int CROP_thirdQuarter = (crop_c_len-1)*3/4;
    const int CROP_right = crop_c_len-1;
    const string debugForward = "\033[1;42mforward\033[0m ";
    const string debugLeft = "\033[1;42mleft\033[0m ";
    const string debugRight = "\033[1;42mright\033[0m ";

    while (true) {
        decodedObjects.clear();
        cap >> frame;
    
        // {
            // lock_guard<mutex> lock(frameMutex);
            xframe = frame.clone(); // Store the frame in the shared variable
        // }
            
        if(xframe.empty()) continue;

        resize(xframe, frameToProcess ,Size() ,SCALE_land, SCALE_land);
        //pi camera color map NOT
        cvtColor(frameToProcess, frameToProcess, COLOR_BGR2RGB);
        cvtColor(frameToProcess, hsv, COLOR_BGR2HSV);
        
        if(isLandingState) {
            debugID = "\033[1;44mLANDING ::\033[0m ";
            inRange(hsv, lower_blue, upper_blue, mask_blue);
            findContours(mask_blue, contours_blue, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
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
                if(dx < THRESH_land_dis_x && dy < THRESH_land_dis_y && greatestBlueContourArea > THRESH_land_area) {
                    if(DEBUG_TXT && DEBUG_txt_land) cout << debugID << "\033[1;44mblue H | Stop then decrease altitude\tlargest blue contour area = " << greatestBlueContourArea << "\tdx dy = " << dx << dy << "\033[0m" << endl;
                    break;
                }
            }
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
            if(greatestWhiteContourArea < THRESH_line_area_whiteLine) greatestWhiteContourArea = 0;
        }
        //crop, count number of black pixles, creat sensor output
        for(int i=0; i < crop_c_len ;i++){
            crop_black = mask_black(Rect(int((width/crop_c_len)*(i)*SCALE_line),0,int((width/crop_c_len)*SCALE_line),int(height*SCALE_line)));
            crop_white = mask_white(Rect(int((width/crop_c_len)*(i)*SCALE_line),0,int((width/crop_c_len)*SCALE_line),int(height*SCALE_line)));
            blackPixles = countNonZero(crop_black);
            whitePixles = countNonZero(crop_white);
            if(blackPixles > THRESH_line_pixles_black) {
                if(whitePixles > THRESH_line_pixles_white_inBlack) {
                    if(greatestWhiteContourArea && whitePixles > greatestWhiteContourArea-THRESH_line_area_whiteLine_offset && whitePixles < greatestWhiteContourArea+THRESH_line_area_whiteLine_offset) c[i] = black_with_white_line;
                    else c[i] = black_with_white;
                } else c[i] = black;
            } else if(whitePixles > THRESH_line_area_white_offroad) c[i] = white; else c[i] = unknown;
        }
        if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << c[CROP_left] << c[CROP_firstQuarter] << c[CROP_mid] << c[CROP_thirdQuarter] << c[CROP_right] << endl;
        
        if(c[CROP_mid] == black_with_white_line) {
            if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "black_with_white_line in CROP_mid" << endl;
            if(!(c[CROP_left] == black || c[CROP_right] == black)) {
                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"neither sides are black ===> pitch 15 degree to " << debugForward <<endl;
            } else if(c[CROP_firstQuarter] == black_with_white && c[CROP_thirdQuarter] == black_with_white) {
                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"quarters are black_with_white ===> pitch 15 degree to " << debugForward <<endl;
            } else {
                if(c[CROP_left] == black) {
                    set_previousDirection(was_going_left);
                    if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"black in CROP_left ===> roll & pitch 15 degree to the " << debugLeft <<endl;
                } else if(c[CROP_right] == black) {
                    set_previousDirection(was_going_right);
                    if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"black in CROP_right ===> roll & pitch 15 degree to the " << debugRight <<endl;
                } else {
                    if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"unknown situation" << endl;
                    if(get_previousDirection() == was_going_left) {
                        if(c[CROP_thirdQuarter] == white && c[CROP_right] == white) set_previousDirection(was_going_right);
                        if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection was was_going_left ===> roll & pitch 15 degree to the " << debugRight << endl;
                    } else {
                        if(c[CROP_left] == white && c[CROP_firstQuarter] == white) set_previousDirection(was_going_left);
                        if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection was was_going_right ===> roll & pitch 15 degree to the " << debugLeft << endl;
                    }
                }
            }
        } else if(c[CROP_mid] == black_with_white) {
            if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "black_with_white in CROP_mid" << endl;
            if(!(c[CROP_left] == black || c[CROP_right] == black) && c[CROP_left] == c[CROP_right]) {
                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"neither sides are black ===> pitch 15 degree to " << debugForward <<endl;
            } else {
                if(c[CROP_left] == black) {
                    set_previousDirection(was_going_left);
                    if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"black in CROP_left ===> roll & pitch 15 degree to the " << debugLeft <<endl;
                } else if(c[CROP_right] == black) {
                    set_previousDirection(was_going_right);
                    if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"black in CROP_right ===> roll & pitch 15 degree to the " << debugRight <<endl;
                } else {
                    if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"unknown situation" << endl;
                    if(get_previousDirection() == was_going_left) {
                        if(c[CROP_thirdQuarter] == white && c[CROP_right] == white) set_previousDirection(was_going_right);
                        if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection was was_going_left ===> roll & pitch 15 degree to the " << debugRight << endl;
                    } else {
                        if(c[CROP_left] == white && c[CROP_firstQuarter] == white) set_previousDirection(was_going_left);
                        if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection was was_going_right ===> roll & pitch 15 degree to the " << debugLeft << endl;
                    }
                }
            }
        } else if(c[CROP_mid] == black) {
            if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "black in CROP_mid" << endl;
            if(c[CROP_left] == white) {
                set_previousDirection(was_going_right);
                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"white in CROP_left ===> roll & pitch 15 degree to the " << debugRight <<endl;
            } else if(c[CROP_right] == white) {
                set_previousDirection(was_going_left);
                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"white in CROP_right ===> roll & pitch 15 degree to the " << debugLeft <<endl;
            } else {
                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"unknown situation" << endl;
                if(get_previousDirection() == was_going_left) {
                    if(c[CROP_thirdQuarter] == white && c[CROP_right] == white) set_previousDirection(was_going_right);
                    if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection was was_going_left ===> roll & pitch 15 degree to the " << debugRight << endl;
                } else {
                    if(c[CROP_left] == white && c[CROP_firstQuarter] == white) set_previousDirection(was_going_left);
                    if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection was was_going_right ===> roll & pitch 15 degree to the " << debugLeft << endl;
                }
            }
        } else if(c[CROP_mid] == white) {
            if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "white in CROP_mid" << endl;
            if(!(c[CROP_left] == white || c[CROP_left] == unknown)) {
                set_previousDirection(was_going_left);
                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"road in CROP_left ===> roll & pitch 15 degree to the " << debugLeft <<endl;
            } else if(!(c[CROP_right] == white || c[CROP_right] == unknown)) {
                set_previousDirection(was_going_right);
                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"road in CROP_right ===> roll & pitch 15 degree to the " << debugRight <<endl;
            } else {
                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"unknown situation" << endl;
                if(get_previousDirection() == was_going_left) {
                    if(c[CROP_thirdQuarter] == white && c[CROP_right] == white) set_previousDirection(was_going_right);
                    if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection was was_going_left ===> roll & pitch 15 degree to the " << debugRight << endl;
                } else {
                    if(c[CROP_left] == white && c[CROP_firstQuarter] == white) set_previousDirection(was_going_left);
                    if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection was was_going_right ===> roll & pitch 15 degree to the " << debugLeft << endl;
                }
            }
        } else {
            if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"unknown situation" << endl;
            if(get_previousDirection() == was_going_left) {
                if(c[CROP_thirdQuarter] == white && c[CROP_right] == white) set_previousDirection(was_going_right);
                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection was was_going_left ===> roll & pitch 15 degree to the " << debugRight << endl;
            } else {
                if(c[CROP_left] == white && c[CROP_firstQuarter] == white) set_previousDirection(was_going_left);
                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection was was_going_right ===> roll & pitch 15 degree to the " << debugLeft << endl;
            }
        }
        //qr
        // decode(frameToProcess, decodedObjects);
        // if(DEBUG_CAM && DEBUG_cam_qr) display(frameToProcess, decodedObjects);

        //send mavlink msg

        //waitKey(100); //only for video !!!!!

        //if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<c[0]<<c[1]<<c[2]<<endl; //<<endl;
        // if(DEBUG_CAM && (DEBUG_cam_line || DEBUG_cam_land))
        imshow("frame", frameToProcess);
    }

    cap.release();

    // lineXqr();

    // VideoCapture cap0(CAM_bottom);
    // lineXqr(cap0);
    //cap0.release();

    //creat threads
    // thread FollowLine(lineXqr);
    // thread OpticalFlow(opticalFlow);
    // thread AvoidObstacle(obstacle);
    //joint threads
    // if(FollowLine.joinable()) FollowLine.join();
    // if(OpticalFlow.joinable()) OpticalFlow.join();
    // if(AvoidObstacle.joinable()) AvoidObstacle.join();
    return 1; 
}

// ######### KN2C Functions #####
    // ##                      ##
    // ##        comment       ##
    // ##                      ##
    // ##########################

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
        if(colorframe.empty() || isLandingState) break;
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
        if(area_yellow > THRESH_obs_color){
            if(DEBUG_TXT && DEBUG_txt_obs) cout << debugID << "\033[1;43myellow obstacle | Stop then decrease altitude. | \tarea yellow = " << area_yellow <<"\tfps = "<<fpslive << "\033[0m" << endl;
        }
        if(area_red > THRESH_obs_color){
            if(DEBUG_TXT && DEBUG_txt_obs) cout << debugID << "\033[1;41mred obstacle | Stop then increase altitude\tarea red = " << area_red <<"\tfps = " <<fpslive << "\033[0m" << endl;
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
    //cout << width <<"\t"<< height << endl;
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
        if(DEBUG_TXT) cout << "$ mission " << mission << endl;
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
void lineXqr() {

    VideoCapture cap(0);
    if (!cap.isOpened()) {
        cerr << "Failed to open bottom camera." << endl;
        return;
    }
    //define variables
    int area_max_i_black, area_max_i_white;
    float area_max_black, area_max_white;
    Mat xframe,frameToProcess, toblur, hsv, mask_black, gray, thr, mask_white;
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
    width = static_cast<int>(cap.get(3));
    height = static_cast<int>(cap.get(4));
    // Mat canny_output;
    // vector<vector<Point> > contours_black;
    // vector<Vec4i> hierarchy;
    // vector<Moments> mu;
    // Moments m;
    // Point center;

    float max;
    float ch = height/2;
    float cw = width/2;
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

    
    const int crop_c_len = 5;
    crop_type c[crop_c_len];
    const int CROP_left = 0;
    const int CROP_firstQuarter = (crop_c_len-1)/4;
    const int CROP_mid = (crop_c_len-1)/2;
    const int CROP_thirdQuarter = (crop_c_len-1)*3/4;
    const int CROP_right = crop_c_len-1;
    const string debugForward = "\033[1;42mforward\033[0m ";
    const string debugLeft = "\033[1;42mleft\033[0m ";
    const string debugRight = "\033[1;42mright\033[0m ";

    while (true) {
        decodedObjects.clear();
        cap >> frame;
    
        // {
            // lock_guard<mutex> lock(frameMutex);
            xframe = frame.clone(); // Store the frame in the shared variable
        // }
            
        if(xframe.empty()) continue;

        resize(xframe, frameToProcess ,Size() ,SCALE_land, SCALE_land);
        //pi camera color map NOT
        cvtColor(frameToProcess, frameToProcess, COLOR_BGR2RGB);
        cvtColor(frameToProcess, hsv, COLOR_BGR2HSV);
        
        if(isLandingState) {
            debugID = "\033[1;44mLANDING ::\033[0m ";
            inRange(hsv, lower_blue, upper_blue, mask_blue);
            findContours(mask_blue, contours_blue, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
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
                if(dx < THRESH_land_dis_x && dy < THRESH_land_dis_y && greatestBlueContourArea > THRESH_land_area) {
                    if(DEBUG_TXT && DEBUG_txt_land) cout << debugID << "\033[1;44mblue H | Stop then decrease altitude\tlargest blue contour area = " << greatestBlueContourArea << "\tdx dy = " << dx << dy << "\033[0m" << endl;
                    break;
                }
            }
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
            if(greatestWhiteContourArea < THRESH_line_area_whiteLine) greatestWhiteContourArea = 0;
        }
        //crop, count number of black pixles, creat sensor output
        for(int i=0; i < crop_c_len ;i++){
            crop_black = mask_black(Rect(int((width/crop_c_len)*(i)*SCALE_line),0,int((width/crop_c_len)*SCALE_line),int(height*SCALE_line)));
            crop_white = mask_white(Rect(int((width/crop_c_len)*(i)*SCALE_line),0,int((width/crop_c_len)*SCALE_line),int(height*SCALE_line)));
            blackPixles = countNonZero(crop_black);
            whitePixles = countNonZero(crop_white);
            if(blackPixles > THRESH_line_pixles_black) {
                if(whitePixles > THRESH_line_pixles_white_inBlack) {
                    if(greatestWhiteContourArea && whitePixles > greatestWhiteContourArea-THRESH_line_area_whiteLine_offset && whitePixles < greatestWhiteContourArea+THRESH_line_area_whiteLine_offset) c[i] = black_with_white_line;
                    else c[i] = black_with_white;
                } else c[i] = black;
            } else if(whitePixles > THRESH_line_area_white_offroad) c[i] = white; else c[i] = unknown;
        }
        if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << c[0] << c[1] << c[2] << c[3] << c[4] << endl;
        if(c[CROP_mid] == black_with_white_line) {
            if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "black_with_white_line in CROP_mid" << endl;
            if(!(c[CROP_left] == black || c[CROP_right] == black)) {
                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"neither sides are black ===> pitch 15 degree to " << debugForward <<endl;
            } else if(c[CROP_firstQuarter] == black_with_white && c[CROP_thirdQuarter] == black_with_white) {
                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"quarters are black_with_white ===> pitch 15 degree to " << debugForward <<endl;
            } else {
                if(c[CROP_left] == black) {
                    set_previousDirection(was_going_left);
                    if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"black in CROP_left ===> roll & pitch 15 degree to the " << debugLeft <<endl;
                } else if(c[CROP_right] == black) {
                    set_previousDirection(was_going_right);
                    if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"black in CROP_right ===> roll & pitch 15 degree to the " << debugRight <<endl;
                } else {
                    if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"unknown situation" << endl;
                    if(get_previousDirection() == was_going_left) {
                        if(c[CROP_thirdQuarter] == white && c[CROP_right] == white) set_previousDirection(was_going_right);
                        if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection was was_going_left ===> roll & pitch 15 degree to the " << debugRight << endl;
                    } else {
                        if(c[CROP_left] == white && c[CROP_firstQuarter] == white) set_previousDirection(was_going_left);
                        if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection was was_going_right ===> roll & pitch 15 degree to the " << debugLeft << endl;
                    }
                }
            }
        } else if(c[CROP_mid] == black_with_white) {
            if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "black_with_white in CROP_mid" << endl;
            if(!(c[CROP_left] == black || c[CROP_right] == black) && c[CROP_left] == c[CROP_right]) {
                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"neither sides are black ===> pitch 15 degree to " << debugForward <<endl;
            } else {
                if(c[CROP_left] == black) {
                    set_previousDirection(was_going_left);
                    if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"black in CROP_left ===> roll & pitch 15 degree to the " << debugLeft <<endl;
                } else if(c[CROP_right] == black) {
                    set_previousDirection(was_going_right);
                    if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"black in CROP_right ===> roll & pitch 15 degree to the " << debugRight <<endl;
                } else {
                    if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"unknown situation" << endl;
                    if(get_previousDirection() == was_going_left) {
                        if(c[CROP_thirdQuarter] == white && c[CROP_right] == white) set_previousDirection(was_going_right);
                        if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection was was_going_left ===> roll & pitch 15 degree to the " << debugRight << endl;
                    } else {
                        if(c[CROP_left] == white && c[CROP_firstQuarter] == white) set_previousDirection(was_going_left);
                        if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection was was_going_right ===> roll & pitch 15 degree to the " << debugLeft << endl;
                    }
                }
            }
        } else if(c[CROP_mid] == black) {
            if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "black in CROP_mid" << endl;
            if(c[CROP_left] == white) {
                set_previousDirection(was_going_right);
                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"white in CROP_left ===> roll & pitch 15 degree to the " << debugRight <<endl;
            } else if(c[CROP_right] == white) {
                set_previousDirection(was_going_left);
                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"white in CROP_right ===> roll & pitch 15 degree to the " << debugLeft <<endl;
            } else {
                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"unknown situation" << endl;
                if(get_previousDirection() == was_going_left) {
                    if(c[CROP_thirdQuarter] == white && c[CROP_right] == white) set_previousDirection(was_going_right);
                    if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection was was_going_left ===> roll & pitch 15 degree to the " << debugRight << endl;
                } else {
                    if(c[CROP_left] == white && c[CROP_firstQuarter] == white) set_previousDirection(was_going_left);
                    if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection was was_going_right ===> roll & pitch 15 degree to the " << debugLeft << endl;
                }
            }
        } else if(c[CROP_mid] == white) {
            if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "white in CROP_mid" << endl;
            if(!(c[CROP_left] == white || c[CROP_left] == unknown)) {
                set_previousDirection(was_going_left);
                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"road in CROP_left ===> roll & pitch 15 degree to the " << debugLeft <<endl;
            } else if(!(c[CROP_right] == white || c[CROP_right] == unknown)) {
                set_previousDirection(was_going_right);
                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"road in CROP_right ===> roll & pitch 15 degree to the " << debugRight <<endl;
            } else {
                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"unknown situation" << endl;
                if(get_previousDirection() == was_going_left) {
                    if(c[CROP_thirdQuarter] == white && c[CROP_right] == white) set_previousDirection(was_going_right);
                    if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection was was_going_left ===> roll & pitch 15 degree to the " << debugRight << endl;
                } else {
                    if(c[CROP_left] == white && c[CROP_firstQuarter] == white) set_previousDirection(was_going_left);
                    if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection was was_going_right ===> roll & pitch 15 degree to the " << debugLeft << endl;
                }
            }
        } else {
            if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<"unknown situation" << endl;
            if(get_previousDirection() == was_going_left) {
                if(c[CROP_thirdQuarter] == white && c[CROP_right] == white) set_previousDirection(was_going_right);
                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection was was_going_left ===> roll & pitch 15 degree to the " << debugRight << endl;
            } else {
                if(c[CROP_left] == white && c[CROP_firstQuarter] == white) set_previousDirection(was_going_left);
                if(DEBUG_TXT && DEBUG_txt_line) cout << debugID << "previousDirection was was_going_right ===> roll & pitch 15 degree to the " << debugLeft << endl;
            }
        }
        //qr
        // decode(frameToProcess, decodedObjects);
        // if(DEBUG_CAM && DEBUG_cam_qr) display(frameToProcess, decodedObjects);

        //send mavlink msg

        //waitKey(100); //only for video !!!!!

        //if(DEBUG_TXT && DEBUG_txt_line) cout << debugID <<c[0]<<c[1]<<c[2]<<endl; //<<endl;
        if(DEBUG_CAM && (DEBUG_cam_line || DEBUG_cam_land)) imshow("frame", frameToProcess);
    }
}

//qr
int readData(string &data) { // -> decode
    int index = int(data.back())-48;
    string debugID = "\033[1;47mQR :: readData ::\033[0m ";

    if(index - mission == 1) mission++;
    if(DEBUG_TXT && DEBUG_txt_qr) {
        cout << debugID << "new mission " << mission << endl;
        cout << debugID << "returning " << data[mission*2] << endl;
    } return data[mission*2];
}
void decode(Mat &im, vector<decodedObject>&decodedObjects) { // -> lineXqr

    if(isLandingState) return;
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

    // Print results
    for(Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol){
        decodedObject obj;

        obj.type = symbol->get_type_name();
        obj.data = symbol->get_data();

        // Print type and data
        if(DEBUG_TXT && DEBUG_txt_qr){
            cout << debugID << "Data = " << obj.data << endl;
        }

        //obtain location
        for(int i = 0; i < symbol->get_location_size(); i++) {
        obj.location.push_back(Point(symbol->get_location_x(i), symbol->get_location_y(i)));
        }
        switch(readData(obj.data)) {
            case 'N':
                // set_previousDirection(was_going_forward);
                if(DEBUG_TXT && DEBUG_txt_qr) cout << debugID << "go North" << endl;
                break;
            case 'E':
                // set_previousDirection(was_going_forward);
                if(DEBUG_TXT && DEBUG_txt_qr) cout << debugID << "go East" << endl;
                break;
            case 'S':
                //set_previousDirection(was_going_forward);
                if(DEBUG_TXT && DEBUG_txt_qr) cout << debugID << "go South" << endl;
                break;
            case 'W':
                //set_previousDirection(was_going_forward);
                if(DEBUG_TXT && DEBUG_txt_qr) cout << debugID << "go West" << endl;
                break;
            default:
                // set_previousDirection(was_going_forward);
                isLandingState = true;
                if(DEBUG_TXT && DEBUG_txt_qr) cout << debugID << "go north" << endl;
                break;
        } decodedObjects.push_back(obj);
    } if(DEBUG_TXT) cout << endl;
}
void display(Mat &im, vector<decodedObject> &decodedObjects) { // -> lineXqr
    
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
