// TODO list:
// line : blure
// landing : color or 
//  

/*########################
##                      ##
##       Libraries      ##
##                      ##
########################*/

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

/*########################
##                      ##
##    Consts & Flags    ##
##                      ##`
########################*/

//cameras | 0 : sabz , 1 : abi, webcam
#define CAM_front 1
#define CAM_bottom !CAM_front
//scale factors
#define SCALE_opf 0.1
#define SCALE_line 0.5
#define SCALE_qr 0.5
#define SCALE_obs 0.2
//color calibre
#define COLOR_RED_lower_h 0 //0
#define COLOR_RED_upper_h 10 //10
#define COLOR_RED_lower_sv 120 //140
#define COLOR_RED_upper_sv 255 //255
#define COLOR_YELLOW_lower_h 20 //20
#define COLOR_YELLOW_upper_h 30 //30
#define COLOR_YELLOW_lower_sv 160 //160
#define COLOR_YELLOW_upper_sv 255 //255
#define THRESH 1000 //2500
//text debug flags
#define DEBUG_TXT 1 // to del
#define debug_txt_opf 0
#define debug_txt_line 0
#define debug_txt_qr 0
#define debug_txt_obs 1
//camera debug flags | activate one camera debugger only
#define DEBUG_CAM 0 // to del
#define debug_cam_opf 1
#define debug_cam_line 1
#define debug_cam_qr 0
#define debug_cam_obs 0

/*########################
##                      ##
##    KN2C Variables    ##
##                      ##
########################*/

typedef struct {
  string type;
  string data;
  vector <Point> location;
} decodedObject;

mutex frameMutex;
Mat frame;
int width, height;
int cam_fps;
int mission = 0;

void opticalFlow();
void lineXqr();
void obstacle();
int readData(string &data);
void display(Mat &im, vector<decodedObject>&decodedObjects);
void decode(Mat &im, vector<decodedObject>&decodedObjects);

/*########################
##                      ##
##       KN2C Main      ##
##                      ##
########################*/

int main() {
    
    if(DEBUG_TXT && !(debug_txt_opf || debug_txt_line || debug_txt_line || debug_txt_obs)) {
        cerr << "activating AT LEAST ONE text debugger is necessary when DEBUG_TXT is 1 (Ln 32)" << endl;
        return -1;
    }
    if(DEBUG_CAM && !(debug_cam_opf ^ debug_cam_line ^ debug_cam_qr ^ debug_cam_obs)) {
        cerr << "activating ONLY ONE camera debugger is necessary when DEBUG_CAM is 1 (Ln 38)" << endl;
        return 0;
    }

    //creat threads
    thread FollowLine(lineXqr);
    thread OpticalFlow(opticalFlow);
    thread AvoidObstacle(obstacle);

    //joint threads
    if(FollowLine.joinable()) FollowLine.join();
    if(OpticalFlow.joinable()) OpticalFlow.join();
    if(AvoidObstacle.joinable()) AvoidObstacle.join();

    return 1; 
}

/*########################
##                      ##
##    KN2C Functions    ##
##                      ##
########################*/

void obstacle() {

    //define variables
    Mat colorframe, hsv, mask_red, mask_yellow;
    vector<vector<Point>> contours_red, contours_yellow;
    float area_red;
    float area_yellow;
    float max_red = 0.0;
    float max_yellow = 0.0;
    int iFrame = 0;
    int cam_fps = 0;
    double fpslive;
    clock_t start, end;
    string debugID = "AVOID OBSTACLE :: ";

    // Define the lower and upper bounds for red color in HSV
    Scalar lower_red(COLOR_RED_lower_h, COLOR_RED_lower_sv, COLOR_RED_lower_sv);
    Scalar upper_red(COLOR_RED_upper_h, COLOR_RED_upper_sv, COLOR_RED_upper_sv);
    // Define the lower and upper bounds for yellow color in HSV
    Scalar lower_yellow(COLOR_YELLOW_lower_h, COLOR_YELLOW_lower_sv, COLOR_YELLOW_lower_sv);
    Scalar upper_yellow(COLOR_YELLOW_upper_h, COLOR_YELLOW_upper_sv, COLOR_YELLOW_upper_sv);

    //webcam
    VideoCapture cap1(CAM_front, CAP_GSTREAMER);
    //video
    //VideoCapture cap1("video.mp4");

    if (!cap1.isOpened()) {
        cerr << "Failed to open front camera." << endl;
        return;
    }
    
    cam_fps = cap1.get(CAP_PROP_FPS);

    if(DEBUG_TXT && debug_txt_obs) cout <<"front camera frame per second: "<<cam_fps<<endl;

    while(true){
        cap1 >> colorframe;
        if(colorframe.empty()) break;
        start = clock();
        //resize and hsv
        resize(colorframe, colorframe ,Size() ,SCALE_obs, SCALE_obs);
        cvtColor(colorframe, hsv, COLOR_BGR2HSV);

        // Create masks for red and yellow objects
        inRange(hsv, lower_red, upper_red, mask_red);
        inRange(hsv, lower_yellow, upper_yellow, mask_yellow);

        // Find contours in the masks
        //findContours(mask_red, contours_red, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        //findContours(mask_yellow, contours_yellow, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        //area calculation
        area_yellow = countNonZero(mask_yellow);
        area_red = countNonZero(mask_red);
        
        //calculate live FPS
        end = clock();
        fpslive = double(CLOCKS_PER_SEC)/(double(end)-double(start));
        cout << "yellow = " << area_yellow <<"\tred = " << area_red << endl;
        //stop till alititude is changing
        if(area_yellow > THRESH){
            if(DEBUG_TXT && debug_txt_obs) cout << debugID << "yellow obstacle | Stop then decrease altitude\tarea yellow = " << area_yellow <<"\tfps = "<<fpslive << endl;
        }
        if(area_red > THRESH){
            if(DEBUG_TXT && debug_txt_obs) cout << debugID << "RED OBSTACLE | Stop then increase altitude\tarea red = " << area_red <<"\tfps = " <<fpslive<< endl;
        }

        if(DEBUG_CAM && debug_cam_obs) {
            // Draw rectangles around the detected objects
            for (const auto& contour : contours_red) {
                Rect rect = boundingRect(contour);
                //if(DEBUG_CAM && debug_cam_obs)
                rectangle(colorframe, rect, Scalar(0, 0, 255), 2);
            }
            for (const auto& contour : contours_yellow) {
                Rect rect = boundingRect(contour);
                //if(DEBUG_CAM && debug_cam_obs)
                rectangle(colorframe, rect, Scalar(0, 255, 255), 2);
            } if(DEBUG_CAM && debug_cam_obs) imshow(debugID+"Red and Yellow Object Detection", colorframe);
        }
    } cap1.release();
}

void lineXqr() {
    //define variables
    const int thresh = 20000;
    int area_max_i;
    double area_max=0;
    Mat xframe,frameToProcess, toblur, hsv, mask;
    Mat crop1, crop2, crop3, crop, mask2;
    vector<vector<Point>> contours;
    Scalar lower_black(0, 0, 0);
    Scalar upper_black(179, 255, 40);
    Point2i p0;
    double left_right;
    int num_black;
    bool c[2];
    string debugID = "FOLLOW LINE :: ";
    vector<decodedObject> decodedObjects;
    
    while (true) {
        decodedObjects.clear();
        
        {
            //lock_guard<mutex> lock(frameMutex);
            xframe = frame.clone(); // Store the frame in the shared variable
        }
            
        if(xframe.empty()) continue;

        //fine black road
        resize(xframe, frameToProcess ,Size() ,SCALE_line, SCALE_line);
        //GaussianBlur(toblur, frameToProcess, Size(5, 5), 0);
        cvtColor(frameToProcess, hsv, COLOR_BGR2HSV);
        inRange(hsv, lower_black, upper_black, mask);
        GaussianBlur(mask, mask, Size(5, 5), 0);
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        //left right ajustment
        //find max area 
        for (int i = 0; i < contours.size(); i++) {
            if(contourArea(contours[i])>area_max){
                area_max_i = i;
                area_max = contourArea(contours[i]);
            }  
        }
        Rect rect = boundingRect(contours[area_max_i]);
        
        //find center to follow
        p0.x = rect.x + rect.width/2;
        p0.y = rect.y + rect.height/2;

        if(DEBUG_CAM && debug_cam_opf){
            circle(frameToProcess, p0, 5, Scalar(0, 0, 255), -1);
            rectangle(frameToProcess, rect, Scalar(0, 0, 255), 2);
        }

        //left right data normal btw -10 and 10
        left_right = (((p0.x - width/4))/ width/4)*10;

        if(left_right>0){
            if(DEBUG_TXT && debug_txt_line) cout << debugID <<"left"<<left_right<<endl;
        } else if(left_right<0){
            if(DEBUG_TXT && debug_txt_line)  cout << debugID <<"right:"<<left_right<<endl;
        }

        //crop, count number of black pixles, creat sensor output
        for(int i=0; i<3 ;i++){
            crop = mask(Rect(int((width/6)*(i)),0,(width/6),height/2));
            num_black = countNonZero(crop);
            if(num_black>thresh) c[i]=1;
            else c[i]=0;
        }
        if((c[0]==0 && c[1]==1 && c[2]==0)||(c[0]==0 && c[1]==0 && c[2]==0)){
            if(DEBUG_TXT && debug_txt_line) cout << debugID <<"forward"<<endl;
        } else if(c[0]==1 && c[1]==0 && c[2]==0){ //shold not happen in real
            if(DEBUG_TXT && debug_txt_line) cout << debugID <<"yaw +15 to turn left "<<endl;
        } else if(c[0]==0 && c[1]==0 && c[2]==1){
            if(DEBUG_TXT && debug_txt_line) cout << debugID <<"yaw -15 to turn right "<<endl;
        } else {
            if(DEBUG_TXT && debug_txt_line) cout << debugID <<"stop"<<endl;
            //read qr and find aproprate yaw 
        }
        
        //qr
        decode(frameToProcess, decodedObjects);
        if(DEBUG_CAM && debug_cam_qr) display(frameToProcess, decodedObjects);

        //send mavlink msg

        //only for video !!!!!
        //waitKey(100);

        if(DEBUG_TXT && debug_txt_line) cout << debugID <<c[0]<<c[1]<<c[2]<<endl; //<<endl;
        if(DEBUG_CAM && debug_cam_line) imshow(debugID+"frame", frameToProcess);
        if(waitKey(5) >= 0) break; //risky
    } 
}

void opticalFlow() {

    //make objects(variables)
    Mat old_frame,old_resize, old_gray;
    vector<Point2f> p0, p1;
    vector<Scalar> colors;
    RNG rng;
    Mat new_frame,frame_resize, frame_gray;
    vector<uchar> status;
    vector<float> err;
    vector<Point2f> good_new, good_old;
    clock_t start, end;
    double fpslive;
    int num;
    double totalMovementx, dx;
    double totalMovementy, dy;
    string debugID = "OPTICAL FLOW :: ";

    // Create some random colors
    if(DEBUG_CAM && debug_cam_opf) {
        for(int i = 0; i < 100; i++) {
            int r = rng.uniform(0, 256);
            int g = rng.uniform(0, 256);
            int b = rng.uniform(0, 256);
            colors.push_back(Scalar(r,g,b));
        }
    }

    //webcam
    VideoCapture cap0(CAM_bottom, CAP_GSTREAMER);
    //video
    //VideoCapture cap0("video2.mp4");

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
    cvtColor(old_resize, old_gray, COLOR_BGR2GRAY);
    // cvtColor(old_frame, old_gray, COLOR_BGR2GRAY);
    
    if(DEBUG_TXT && (debug_txt_opf || debug_txt_line || debug_txt_qr)) cout <<"bottom camera frame per second: "<<cam_fps<<endl;
    TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);

    while(true){
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
        calcOpticalFlowPyrLK(old_gray, frame_gray, p0, p1, status, err, Size(30,30), 2, criteria);
        
        //calculate flow quality
        // calculate mean of vector and print dx and dy
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
        fpslive = double(CLOCKS_PER_SEC)/(double(end)-double(start));
        
        //mavlink massage
        if(DEBUG_TXT && debug_txt_opf) cout << debugID <<"right(dx)= "<<totalMovementx<<"   forward(dy)="<<totalMovementy <<"     flow quality = "<<flow_quality<<"     fps = "<<fpslive<<endl; //<<endl;
        if(DEBUG_CAM && debug_cam_opf) {
            Point org(50, 50); 
            putText(frame_resize, to_string(fpslive), org , FONT_HERSHEY_DUPLEX, 1 , Scalar(200, 0, 150),1.5);
            imshow(debugID+"Frame", frame_resize);
        }  if(waitKey(5) >= 0) break; //risky //waitKey(100); //only for video !!!!!
    } cap0.release();
}
//QR
int readData(string &data) { // -> decode
    //static int mission = 0;
    int index = int(data.back())-48;
    string debugID = "QR :: readData :: ";

    if(index - mission == 1) mission++;
    if(DEBUG_TXT) {
        cout << "mission " << mission << endl;
        if(debug_txt_qr) cout << debugID << "returning " << data[mission*2] << endl;
    } return data[mission*2];
}
void decode(Mat &im, vector<decodedObject>&decodedObjects) { // -> lineXqr

  // Create zbar scanner
  ImageScanner scanner;
  // Configure scanner
  scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1); 
  // Convert image to grayscale
  Mat imGray;
  cvtColor(im, imGray,COLOR_BGR2GRAY);
  // Wrap image data in a zbar image
  Image image(im.cols, im.rows, "Y800", (uchar *)imGray.data, im.cols * im.rows);
  // Scan the image for barcodes and QRCodes
  int n = scanner.scan(image);
  string debugID = "QR :: decode :: ";

  // Print results
  for(Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol){
    decodedObject obj;

    obj.type = symbol->get_type_name();
    obj.data = symbol->get_data();

    // Print type and data
    if(DEBUG_TXT && debug_txt_qr){
        cout << debugID << "Data = " << obj.data << endl;
    }

    //obtain location
    for(int i = 0; i < symbol->get_location_size(); i++) {
      obj.location.push_back(Point(symbol->get_location_x(i), symbol->get_location_y(i)));
    }
    switch(readData(obj.data)) {
        case 'N':
            if(DEBUG_TXT && debug_txt_qr) cout << debugID << "go North" << endl;
            break;
        case 'E':
            if(DEBUG_TXT && debug_txt_qr) cout << debugID << "go East" << endl;
            break;
        case 'S':
            if(DEBUG_TXT && debug_txt_qr) cout << debugID << "go South" << endl;
            break;
        case 'W':
            if(DEBUG_TXT && debug_txt_qr) cout << debugID << "go West" << endl;
            break;
        default:
            if(DEBUG_TXT && debug_txt_qr) cout << debugID << "land" << endl;
            break;
    } decodedObjects.push_back(obj);
  } if(DEBUG_TXT) cout << endl;
}
void display(Mat &im, vector<decodedObject> &decodedObjects) { // -> lineXqr
    
    if(!(DEBUG_CAM && debug_cam_qr)) return;
    string debugID = "QR :: display :: ";
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
    }
 
  // Display results
  imshow(debugID+"Results", im);
}

