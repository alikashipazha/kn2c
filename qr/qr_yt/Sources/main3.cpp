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
    

#define TRESH_area 2500
#define DEBUG 0
#define debugCam 0

#define debugOpf 1
#define debugLine 1
#define debugQr 0
#define debugObs 0


#define greenWebcam 0
#define redWebcam 1
    

using namespace cv;
using namespace std;
using namespace zbar; 

using namespace zbar;

typedef struct
{
  string type;
  string data;
  vector <Point> location;
}decodedObject;

int mission = 0;

mutex frameMutex;
Mat frame;
int width;
int height;
int cam_fps;

void opticallflow();
void lineqr();
void obstacle();
int readData(string data);
void display(Mat im, vector<decodedObject> decodedObjects);
void decode(Mat &im, vector<decodedObject>&decodedObjects);



int main(){

//creat threads
thread line_follow(lineqr);
thread opticall_flow(opticallflow);
//thread obstacle_avoid(obstacle);



//joint threads
if(line_follow.joinable()){
    line_follow.join();
}
if(opticall_flow.joinable()){
    opticall_flow.join();
}
// if(obstacle_avoid.joinable()){
//     obstacle_avoid.join();
// }


return 0; 
}

void obstacle(){

    //define variables
    Mat colorframe, hsv, mask_red, mask_yellow;
    vector<vector<Point>> contours_red, contours_yellow;
    float area_red;
    float area_yellow;
    float max_red = 0.0;
    float max_yellow = 0.0;
    int iFrame = 0;

    // Define the lower and upper bounds for red color in HSV
    Scalar lower_red(0, 140, 140);
    Scalar upper_red(10, 255, 255);
    // Define the lower and upper bounds for yellow color in HSV
    Scalar lower_yellow(20, 160, 160);
    Scalar upper_yellow(30, 255, 255);

    //webcam
    VideoCapture cap1(greenWebcam);

    //video
    //string path = "video.mp4";
    //VideoCapture cap1(path);

    while (true)
{
    
    cap1 >> colorframe;
    if(colorframe.empty()){
        break;;
    }

    //resize and hsv
    resize(colorframe, colorframe ,Size() ,0.5, 0.5);
    cvtColor(colorframe, hsv, COLOR_BGR2HSV);

    // Create masks for red and yellow objects
    inRange(hsv, lower_red, upper_red, mask_red);
    inRange(hsv, lower_yellow, upper_yellow, mask_yellow);

    // Find contours in the masks
    findContours(mask_red, contours_red, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    findContours(mask_yellow, contours_yellow, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    //area calculation
    area_yellow = countNonZero(mask_yellow);
    area_red = countNonZero(mask_red);

    //stop till alititude is changing
    if(area_yellow > TRESH_area) if(DEBUG && debugObs) cout << "YELLOW OBSTACLE!!! ********** Stop then decrease altitude" << std::endl;
    if(area_red > TRESH_area) if(DEBUG && debugObs) cout << "red OBSTACLE!!! ********** Stop then increase altitude" << std::endl;

    // Draw rectangles around the detected objects
    for (const auto& contour : contours_red) {
        Rect rect = boundingRect(contour);
        //test if(DEBUG && debugObs && debugCam) rectangle(colorframe, rect, Scalar(0, 0, 255), 2);
    }

    for (const auto& contour : contours_yellow) {
        Rect rect = boundingRect(contour);
        //test if(DEBUG && debugObs && debugCam) rectangle(colorframe, rect, Scalar(0, 255, 255), 2);
    }

    // Show the resulting image
    //test if(DEBUG && debugObs && debugCam) imshow("Red and Yellow Object Detection", colorframe);


}
cap1.release();
}

void lineqr(){

    //define variables
    int thresh = 20000;
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
    vector<decodedObject> decodedObjects;
    

    

while (true)
{
    decodedObjects.clear();

    {
    //lock_guard<mutex> lock(frameMutex);
    xframe = frame.clone(); // Store the frame in the shared variable
    }
        
    if(xframe.empty()){
        continue;
    }

    //fine black road
    resize(xframe, frameToProcess ,Size() ,0.5, 0.5);
    //GaussianBlur(toblur, frameToProcess, Size(5, 5), 0);
    cvtColor(frameToProcess, hsv, COLOR_BGR2HSV);
    inRange(hsv, lower_black, upper_black, mask);
    GaussianBlur(mask, mask, Size(5, 5), 0);
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    //left right ajustment
    //find max area 
    for (int i = 0; i < contours.size(); i++)
	{
        if(contourArea(contours[i])>area_max){
	        area_max_i = i;
            area_max = contourArea(contours[i]);
        }  
    }
    Rect rect = boundingRect(contours[area_max_i]);

    //find center to follow
    p0.x = rect.x + rect.width/2;
    p0.y = rect.y + rect.height/2;
    //test if(DEBUG && debugLine && debugCam){
    //test circle(frameToProcess, p0, 5, Scalar(0, 0, 255), -1);
    //test rectangle(frameToProcess, rect, Scalar(0, 0, 255), 2);
    //test }

    //left right data normal btw -10 and 10
    left_right = (((p0.x - width/4))/ width/4)*10;

    //print left or right 
    // if(-2<left_right<2){
    //     cout<<"forward"<<endl;
    // }
    // else 
    if(left_right>0){
         if(DEBUG && debugLine) cout<<"left"<<left_right<<endl;
    }
    else if(left_right<0){
        if(DEBUG && debugLine)  cout<<"right:"<<left_right<<endl;
    }

    //yaw 
    //crop frame to 3
    // mask2 = mask(Rect(0,int(height/3),int(width),int(height/3)));
    // crop1 = mask2(Rect(0,0,int(width/3),height/3));
    // crop2 = mask2(Rect(int(width/3),0,width/3,height/3));
    // crop3 = mask2(Rect(int(2*width/3),0,width/3,height/3));


    //crop, count number of black pixles, creat sensor output
    for(int i=0; i<3 ;i++){
        crop = mask(Rect(int((width/6)*(i)),0,(width/6),height/2));
        num_black = countNonZero(crop);
        if(num_black>thresh){
            c[i]=1;
        }
        else{
            c[i]=0;
        }
    }
    if((c[0]==0 && c[1]==1 && c[2]==0)||(c[0]==0 && c[1]==0 && c[2]==0)){
        if(DEBUG && debugLine) cout<<"forward"<<endl;
    }
    // shold not happen in real
    else if(c[0]==1 && c[1]==0 && c[2]==0){
        if(DEBUG && debugLine) cout<<"yaw +15 to turn left "<<endl;
    }
    else if(c[0]==0 && c[1]==0 && c[2]==1){
        if(DEBUG && debugLine) cout<<"yaw -15 to turn right "<<endl;
    }
    else{
        if(DEBUG && debugLine) cout<<"stop"<<endl;
        //read qr and find aproprate yaw 
    }
    
    //qr
    decode(frameToProcess, decodedObjects);
    //test if(DEBUG && debugQr && debugCam) display(frameToProcess, decodedObjects);

        
    //send mavlink msg

    //only for video !!!!!
    //waitKey(100);

    if(DEBUG && debugLine) {
        cout<<c[0]<<c[1]<<c[2]<<endl;
        //test if(debugCam) imshow("frame", frameToProcess);
    }
}

}

void opticallflow(){



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
    double totalMovementx;
    double totalMovementy;
    int num;


    // Create some random colors
    //test if(DEBUG && debugOpf && debugCam) {
    //test     for(int i = 0; i < 100; i++)
    //test     {
    //test         int r = rng.uniform(0, 256);
    //test         int g = rng.uniform(0, 256);
    //test         int b = rng.uniform(0, 256);
    //test         colors.push_back(Scalar(r,g,b));
    //test     }
    //test }

    //webcam
    VideoCapture cap0(redWebcam);

    //video
    //string path = "video2.mp4";
    //VideoCapture cap0(path);

    if (!cap0.isOpened()) {
        std::cerr << "Failed to open the camera." << std::endl;
        return;
    }

    width = static_cast<int>(cap0.get(3));
    height = static_cast<int>(cap0.get(4));
    cam_fps = cap0.get(CAP_PROP_FPS);
    cap0 >> old_frame; // Capture a frame

    resize(old_frame, old_resize ,Size() ,0.5, 0.5);
    cvtColor(old_resize, old_gray, COLOR_BGR2GRAY);
    // cvtColor(old_frame, old_gray, COLOR_BGR2GRAY);
    //Get cammera FPS
    
    if(DEBUG && debugOpf) cout<<"cammera frame per second: "<<cam_fps<<endl;
        
     while(true) {
    {
        lock_guard<mutex> lock(frameMutex);
        cap0 >> frame;
        }
        //get new frame
      
        start = clock();
        resize(frame, frame_resize ,Size() ,0.5, 0.5);
        cvtColor(frame_resize, frame_gray, COLOR_BGR2GRAY);
        // cvtColor(frame, frame_gray, COLOR_BGR2GRAY);

        //find feature points in old
        goodFeaturesToTrack(old_gray, p0, 20, 0.3, 7, Mat(), 7, false, 0.04);

        
        // calculate optical flow
        TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
        // if(!p0.empty()){
        //     continue;
        // }
        calcOpticalFlowPyrLK(old_gray, frame_gray, p0, p1, status, err, Size(30,30), 2, criteria);
        
        //calculate flow quality
        // calculate mean of vector and print dx and dy
        totalMovementx = 0.0;
        totalMovementy = 0.0;
        num =0;
        for(uint i = 0; i < p0.size(); i++)
        {
            if(status[i] == 1) {
                totalMovementx += (p1[i].x - p0[i].x);
                totalMovementy += (p1[i].y - p0[i].y);
                line(frame_resize,p0[i], p1[i], colors[i], 1);
                circle(frame_resize, p1[i], 2, colors[i], -1);
                num++;
        }}

        // calculate mean of vector and print dx and dy
        old_gray = frame_gray.clone();
        int flow_quality = round(255.0 * num / status.size());


        //mavlink massage
        
        //only for video
        //waitKey(100);

        //calculate FPS
        end = clock();
        fpslive = double(CLOCKS_PER_SEC)/(double(end)-double(start));
        //fpslive = (double(end)-double(start));
        
        string fps=to_string(fpslive);
        if(DEBUG && debugOpf) cout << "Total Camera Movement:  dx = " << totalMovementx <<"  dy= "<< totalMovementy <<"  flow quality = "<<flow_quality<<"  live fps = "<<fps<<endl;
        
        //test if(DEBUG && debugOpf && debugCam) {
	    //test Point org(50, 50); 
        //test putText(frame_resize, fps, org , FONT_HERSHEY_DUPLEX, 1 , Scalar(200, 0, 150),1.5);
        //test imshow("Frame", frame_resize);
        //test }

    }
    cap0.release();
    }

void opticallflow2(){


Mat prevFrame, currFrame, prevGray, currGray;
vector<Point2f> prevPoints, currPoints;
vector<uchar> status;
vector<float> err;
clock_t start, end;
double fpslive;
int num;

TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS, 20, 0.03);
Size winSize(31, 31);

VideoCapture cap(redWebcam);  // Open the camera
//VideoCapture cap("video.mp4");

if(!cap.isOpened()) {
    cerr << "Error: Could not open camera." << endl;
    return;
}
//Get cammera FPS
int cam_fps = cap.get(CAP_PROP_FPS);
cout<<"cammera frame per second: "<<cam_fps<<endl;

cap >> prevFrame;
cvtColor(prevFrame, prevGray, COLOR_BGR2GRAY);


while(true) {
    cap >> currFrame;
    start = clock();
    if (currFrame.empty()) {
        break;
    }
    //find good points to track
    goodFeaturesToTrack(prevGray, prevPoints, 20, 0.01, 10);

    cvtColor(currFrame, currGray, cv::COLOR_BGR2GRAY);    
    calcOpticalFlowPyrLK(prevGray, currGray, prevPoints, currPoints, status, err, winSize, 3, termcrit, 0, 0.001);
    
    // Calculate and display camera movement
    double totalMovementx = 0.0;
    double totalMovementy = 0.0;
    num=0;
    for (size_t i = 0; i < prevPoints.size(); i++) {
        if (status[i] == 1) {
            double dx = (currPoints[i].x - prevPoints[i].x);
            double dy = (currPoints[i].y - prevPoints[i].y);
            totalMovementx += dx;
            totalMovementy += dy;
            line(currFrame, prevPoints[i], currPoints[i], cv::Scalar(0, 0, 255), 2);
            num++;
        }
    }

    int flow_quality = round(255.0 * num / status.size());

    // Exit on key press
    if (waitKey(1) == 27) {
        break;
    }
    //waitKey(100);

    // Update previous frame and points
    prevGray = currGray.clone();


    end = clock();
    fpslive = double(CLOCKS_PER_SEC)/(double(end)-double(start));
    string fps=to_string(fpslive);
    Point org(50, 50); 
    putText(currFrame, fps, org , FONT_HERSHEY_DUPLEX, 1 , Scalar(200, 0, 150),1.5);
    cout << "Total Camera Movement:  dx = " << totalMovementx <<"  dy= "<< totalMovementy <<"  flow quality = "<<flow_quality<<endl;

    // Display the result
    //test imshow("Optical Flow", currFrame);
}
}

int readData(string data) {
    //static int mission = 0;
    int index = int(data.back())-48;
    if(index - mission == 1) {
        if(DEBUG && debugQr) cout << data[mission*2] << endl;
        mission++;
    }
    if(DEBUG && debugQr) cout << mission << endl;
    if(DEBUG && debugQr) cout << data[mission*2] << endl;
    return data[mission*2];
}

void decode(Mat &im, vector<decodedObject>&decodedObjects)
{

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

  // Print results
  for(Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
  {
    decodedObject obj;

    obj.type = symbol->get_type_name();
    obj.data = symbol->get_data();

    // Print type and data
    if(DEBUG && debugQr) cout << "Type : " << obj.type << endl;
    if(DEBUG && debugQr) cout << "Data : " << obj.data << endl << endl;
    
    //obtain location
    // for(int i = 0; i < symbol->get_location_size(); i++) {
    //   obj.location.push_back(cv::Point(symbol->get_location_x(i), symbol->get_location_y(i)));
    // }
    switch(readData(obj.data)) {
      case 'N':
        if(DEBUG && debugQr) cout << "go North" << endl;
        break;
      case 'E':
        if(DEBUG && debugQr) cout << "go East" << endl;
        break;
      case 'S':
        if(DEBUG && debugQr) cout << "go South" << endl;
        break;
      case 'W':
        if(DEBUG && debugQr) cout << "go West" << endl;
        break;
      default:
        if(DEBUG && debugQr) cout << "land" << endl;
	    break;
    }

    decodedObjects.push_back(obj);
  }
}

void display(Mat &im, vector<decodedObject> &decodedObjects) {
    //test if(!(DEBUG && debugQr && debugCam)) return;
  // Loop over all decoded objects
  for(int i = 0; i < decodedObjects.size(); i++)
  {
    vector<Point> points = decodedObjects[i].location;
    vector<Point> hull;
    // If the points do not form a quad, find convex hull
    if(points.size() > 4)
      convexHull(points, hull);
    else
      hull = points;
 
    // Number of points in the convex hull
    int n = hull.size();
    int randColor1 = rand()%255;
    int randColor2 = rand()%255;
    int randColor3 = rand()%255;
    for(int j = 0; j < n; j++)
    {
      polylines(im, hull, true, Scalar(randColor1,randColor2,randColor3), 2);
      putText(im, decodedObjects[i].data, hull[0], FONT_HERSHEY_DUPLEX, 0.75, Scalar(200, 0, 150),1.5);
    }
 
  }
 
  // Display results
  //test imshow("Results", im);

}

