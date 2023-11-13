#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include <opencv2/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <time.h>       
using namespace cv;
using namespace std;
int main(){


Mat prevFrame, currFrame, prevGray, currGray;
vector<Point2f> prevPoints, currPoints;
vector<uchar> status;
vector<float> err;
clock_t start, end;
double fpslive;
int num;

TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS, 20, 0.03);
Size winSize(31, 31);

//VideoCapture cap(0);  // Open the camera
VideoCapture cap("video.mp4");

if(!cap.isOpened()) {
    cerr << "Error: Could not open camera." << endl;
    return -1;
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
    waitKey(100);

    // Update previous frame and points
    prevGray = currGray.clone();


    end = clock();
    fpslive = double(CLOCKS_PER_SEC)/(double(end)-double(start));
    string fps=to_string(fpslive);
    Point org(50, 50); 
    putText(currFrame, fps, org , FONT_HERSHEY_DUPLEX, 1 , Scalar(200, 0, 150),1.5);
    cout << "Total Camera Movement:  dx = " << totalMovementx <<"  dy= "<< totalMovementy <<"  flow quality = "<<flow_quality<<endl;

    // Display the result
    imshow("Optical Flow", currFrame);
}
return 0; 
}