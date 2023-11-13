#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

#define COLOR_BLUE_lower_h 100
#define COLOR_BLUE_upper_h 130
#define COLOR_BLUE_lower_sv 120 // s 120 v 100
#define COLOR_BLUE_upper_sv 255

#define DEBUG_TXT 1
#define DEBUG_CAM 1
#define debugID 100

#define TRESH_area 2500
#define TRESH 2000
// Mat src, src_gray;
// Mat dst, detected_edges;

// int lowThreshold = 0;
// const int max_lowThreshold = 100;
// const int ratio = 3;
// const int kernel_size = 3;
// const char* window_name = "Edge Map";


// static void CannyThreshold(int, void*)
// {
//  blur( src_gray, detected_edges, Size(3,3) );
//  Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
//  dst = Scalar::all(0);
//  src.copyTo( dst, detected_edges);
//  imshow( window_name, dst );
// }

// int getMaxAreaContourId(vector <vector<cv::Point>> contours) {
//     double maxArea = 0;
//     int maxAreaContourId = -1;
//     for (int j = 0; j < contours.size(); j++) {
//         double newArea = cv::contourArea(contours.at(j));
//         if (newArea > maxArea) {
//             maxArea = newArea;
//             maxAreaContourId = j;
//         } // End if
//     } // End for
//     return maxAreaContourId;
// } // End function


int main(int argc, char *argv[]) {
    // Open the default camera or read an image from file
    VideoCapture cap(0);

    //string path = "video.mp4";
	//VideoCapture cap(path);

    if (!cap.isOpened()) {
        cout << "Error: Unable to open the camera or file." << endl;
        return -1;
    }
    

    Mat frameToProcess, hsv, mask_red, mask_yellow, mask_blue, gray, thr;
    vector<vector<Point>> contours_red, contours_yellow, contours_blue;
    float area_red;
    float area_yellow;
    float area_blue;
    float max_red = 0.0;
    float max_yellow = 0.0;
    int iFrame = 0; 
    Moments m;
    Point center;
    Mat canny_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    vector<Moments> mu;

    float width = static_cast<int>(cap.get(3));
    float height = static_cast<int>(cap.get(4));
    float ch = height/2;
    float cw = width/2;
    vector<Point> largestContour;
    Point2f contourCenter;
    Point2f imageCenter(cw, ch);

    
    while (true) {

        area_yellow = 0;
        area_red = 0;
        // Capture frameToProcess from the camera or file
        cap >> frameToProcess;
        if (frameToProcess.empty()) {
            std::cout << "End of video stream or image file." << endl;
            break;
        }

        // Convert BGR to HSV
        cvtColor(frameToProcess, frameToProcess, COLOR_BGR2RGB);
        cvtColor(frameToProcess, hsv, COLOR_BGR2HSV);


        // Define the lower and upper bounds for red color in HSV
        Scalar lower_red(0, 140, 140);
        Scalar upper_red(10, 255, 255);
        // Define the lower and upper bounds for yellow color in HSV
        Scalar lower_yellow(20, 110, 120);
        Scalar upper_yellow(35, 255, 255);
        Scalar lower_blue(COLOR_BLUE_lower_h, COLOR_BLUE_lower_sv, COLOR_BLUE_lower_sv);
        Scalar upper_blue(COLOR_BLUE_upper_h, COLOR_BLUE_upper_sv, COLOR_BLUE_upper_sv);
    
        // Create masks for red and yellow objects
        inRange(hsv, lower_red, upper_red, mask_red);
        inRange(hsv, lower_yellow, upper_yellow, mask_yellow);
        inRange(hsv, lower_blue, upper_blue, mask_blue);
        //findContours(mask_blue, contours_blue, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

///////////////////////////////////////// center
        // cvtColor(frameToProcess, gray, COLOR_BGR2GRAY);
            // Canny( gray, canny_output, 50, 150, 3 );
            
            // // find contours
            // findContours( canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
            
            // // get the moments
            // vector<Moments> mu(contours.size());
            // for( int i = 0; i<contours.size(); i++ )
            // { mu[i] = moments( contours[i], false ); }
            
            // // get the centroid of figures.
            // vector<Point2f> mc(contours.size());
            // for( int i = 0; i<contours.size(); i++)
            // { mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }
            
            // // draw contourshsv or rgb
            // Mat drawing(canny_output.size(), CV_8UC3, Scalar(255,255,255));
            // for( int i = 0; i<contours.size(); i++ ) {
            //     Scalar color = Scalar(167,151,0); // B G R values
            //     drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
            //     circle( drawing, mc[i], 4, color, -1, 8, 0 );
            // }
        // threshold( gray, thr, 100,255,THRESH_BINARY );
        
        // // find moments of the image
        // m = moments(thr,true);
        // center.x = m.m10/m.m00; 
        // center.y = m.m01/m.m00;
        
        // coordinates of centroid
        if(DEBUG_TXT) cout << debugID << "center.x = " << center.x << "\tcenter.y = " << center.y << endl;
        if(DEBUG_CAM) circle(frameToProcess, center, 5, Scalar(128,0,0), -1);

///////////////////////////////////////////
        // Find contours in the masks
        findContours(mask_red, contours_red, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE); // contours_red[i]?
        findContours(mask_yellow, contours_yellow, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        findContours(mask_blue, contours_blue, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // Draw rectangles around the detected objects
        for (const auto& contour : contours_red) {
            Rect rect = boundingRect(contour);
            rectangle(frameToProcess, rect, Scalar(0, 0, 255), 2);
        }

        for (const auto& contour : contours_yellow) {
            Rect rect = boundingRect(contour);
            rectangle(frameToProcess, rect, Scalar(0, 255, 255), 2);
        }
        double max = 0.0;
        int i = 0;
        for (const auto& contour : contours_blue) {
            double current = contourArea(contour);
            if(current > max) {
                max = current;
                largestContour = contour;
                cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! contour " << i++ << " is the largest with " << max << "area" << endl;
            }
            Rect rect = boundingRect(contour);
            rectangle(frameToProcess, rect, Scalar(255, 0, 0), 2);
        }
        //largestContour = contours_blue.at(getMaxAreaContourId(contours_blue));
        Rect largestRect = boundingRect(largestContour);
        rectangle(frameToProcess, largestRect, Scalar(0, 0, 0), 2);

        contourCenter.x = largestRect.x + largestRect.width/2;
        contourCenter.y = largestRect.y + largestRect.height/2;
        circle(frameToProcess, imageCenter, 10, Scalar(0, 0, 0), -1);
        circle(frameToProcess, contourCenter, 5, Scalar(0, 0, 255), -1);    
        line(frameToProcess, contourCenter, imageCenter, Scalar(0,0,255), 3);

        float dis = sqrt((cw - contourCenter.x)*(cw - contourCenter.x) + (ch - contourCenter.y)*(ch - contourCenter.y));
        cout << "dis " << dis << endl;
        // yellow area calculation
        // for (int i = 0; i < contours_yellow.size(); i++) {
        //     area_yellow = area_yellow + contourArea(contours_yellow[i]);
        //     string objectType;
        /crop_black = mask_black(Rect(int((width/crop_c_len)*(i)*SCALE_line),0,int((width/crop_c_len)*SCALE_line),int(height*SCALE_line)));
            / }
        //stop till alititude is decreasing

        // red area calculation
        // for (int i = 0; i < contours_red.size(); i++) {
        //     area_red = area_red + contourArea(contours_red[i]);
        //     string objectType;
        // }
        
        //area calculation
        area_yellow = countNonZero(mask_yellow);
        area_red = countNonZero(mask_red);
        area_blue = countNonZero(mask_blue);
        
        cout << "yellow " << area_yellow << " red " << area_red << " blue " << area_blue << endl;
        if(area_yellow > TRESH_area) cout << "YELLOW OBSTACLE!!! ********** Stop then decrease altitude" << std::endl;
        if(area_red > TRESH_area) cout << "red OBSTACLE!!! ********** Stop then increase altitude" << std::endl;
        if(area_blue > TRESH_area) cout << "blue OBSTACLE!!! ********** land" << std::endl;

        // Show the resulting image
        
        imshow("Red and Yellow and Blue Object Detection", frameToProcess);
        if(dis < 20 && contourArea(largestContour) > TRESH) {
            cout << "@!#$%^&*!@#$%^ LAND !!!!!!!!!!!!";
            break;
        }
        // Press 'q' to exit the loop
        if (waitKey(1) == 'q') {
            break;
        }
    }

    // Release the video capture and close the display window
    cap.release();
    //destroyAllWindows();

    return 0;
}