#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

#define TRESH_area 250
int main(int argc, char *argv[]) {
    // Open the default camera or read an image from file
    VideoCapture cap(1);

    // string path = "video.mp4";
	// VideoCapture cap(path);

    if (!cap.isOpened()) {
        cout << "Error: Unable to open the camera or file." << endl;
        return -1;
    }
    

    Mat frame, hsv, mask_red, mask_yellow;
    vector<vector<Point>> contours_red, contours_yellow;
    float area_red;
    float area_yellow;
    float max_red = 0.0;
    float max_yellow = 0.0;
    int iFrame = 0; 

    
    while (true) {

        area_yellow = 0;
        area_red = 0;
        // Capture frame from the camera or file
        cap >> frame;
        if (frame.empty()) {
            std::cout << "End of video stream or image file." << endl;
            break;
        }

        // Convert BGR to HSV
        cvtColor(frame, hsv, COLOR_BGR2HSV);

        // Define the lower and upper bounds for red color in HSV
        Scalar lower_red(0, 140, 140);
        Scalar upper_red(10, 255, 255);
        // Define the lower and upper bounds for yellow color in HSV
        Scalar lower_yellow(20, 160, 160);
        Scalar upper_yellow(30, 255, 255);

        // Create masks for red and yellow objects
        inRange(hsv, lower_red, upper_red, mask_red);
        inRange(hsv, lower_yellow, upper_yellow, mask_yellow);

        // Find contours in the masks
        findContours(mask_red, contours_red, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE); // contours_red[i]? 
        findContours(mask_yellow, contours_yellow, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // Draw rectangles around the detected objects
        for (const auto& contour : contours_red) {
            Rect rect = boundingRect(contour);
            rectangle(frame, rect, Scalar(0, 0, 255), 2);
        }

        for (const auto& contour : contours_yellow) {
            Rect rect = boundingRect(contour);
            rectangle(frame, rect, Scalar(0, 255, 255), 2);
        }

        // yellow area calculation
        for (int i = 0; i < contours_yellow.size(); i++)
	    {
		area_yellow = area_yellow + contourArea(contours_yellow[i]);
		string objectType;
        }
        //stop till alititude is decreasing
        if(area_yellow > TRESH_area) cout << "YELLOW OBSTACLE!!! ********** Stop then decrease altitude" << std::endl;

        // red area calculation
        for (int i = 0; i < contours_red.size(); i++)
	    {
		area_red = area_red + contourArea(contours_red[i]);
		string objectType;
        }
        if(area_red > TRESH_area) cout << "red OBSTACLE!!! ********** Stop then increase altitude" << std::endl;

        // Show the resulting image
        //imshow("Red and Yellow Object Detection", frame);
  
        // Press 'q' to exit the loop
        if (waitKey(1) == 'q') {
            break;
        }
    }

    // Release the video capture and close the display window
    cap.release();
    destroyAllWindows();

    return 0;
}