#include <iostream>
//#include <opencv2/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
     
using namespace cv;
using namespace std;

int main(int argc, char **argv){

//webcam
// VideoCapture cap(0);

//video
string path = "video.mp4";
VideoCapture cap(path);

//define variables
int thresh = 10000;
int area_max_i;
double area_max=0;
Mat frame, hsv, mask;
Mat crop1, crop2, crop3, crop;
vector<vector<Point>> contours;
Scalar lower_black(0, 0, 130);
Scalar upper_black(179, 255, 255);
Point2i p0;
int width = static_cast<int>(cap.get(3));
int height = static_cast<int>(cap.get(4));
double left_right;
int num_black;
bool c[2];

while (true)
{
    cap >> frame;
    
    //fine black road
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    inRange(hsv, lower_black, upper_black, mask);
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
    rectangle(frame, rect, Scalar(0, 0, 255), 2);
    // cout<<area_max<<endl; 
    
    //find center to follow
    p0.x = rect.x + rect.width/2;
    p0.y = rect.y + rect.height/2;
    circle(frame, p0, 5, Scalar(0, 0, 255), -1);

    //left right data normal btw -10 and 10
    left_right = (((p0.x - width/2))/ width/2)*10;

    //print left or right 
    if(-2<left_right<2){
        cout<<"forward"<<endl;
    }
    else if(left_right>2){
        cout<<"left"<<left_right<<endl;
    }
    else if(left_right<-2){
        cout<<"right:"<<left_right<<endl;
    }

    //yaw 
    //crop frame to 3
    crop1 = mask(Rect(0,0,int(width/3),height));
    crop2 = mask(Rect(int(width/3),0,width/3,height));
    crop3 = mask(Rect(int(2*width/3),0,width/3,height));

    //count black pixles
    // c1 = countNonZero(crop1);
    // c2 = countNonZero(crop2);
    // c3 = countNonZero(crop3);

    //crop, count number of black pixles, creat sensor output
    for(int i=0; i<3 ;i++){
        crop = mask(Rect(int((width/3)*(i)),0,(width/3),height));
        num_black = (width/3*height) - countNonZero(crop);
        if(num_black>thresh){
            c[i]=1;
        }
        else{
            c[i]=0;
        }
    }
    if((c[0]==0 && c[1]==1 && c[2]==0)||(c[0]==0 && c[1]==0 && c[2]==0)){
        cout<<"forward"<<endl;
    }
    //shold not happen
    // else if(c[0]==1 && c[1]==0 && c[2]==0){
    //     cout<<"yaw +15 to turn left "<<endl;
    // }
    // else if(c[0]==0 && c[1]==0 && c[2]==1){
    //     cout<<"yaw -15 to turn right "<<endl;
    // }
    else{
        cout<<"stop"<<endl;
        //read qr and find aproprate yaw 
    }
    cout<<c[0]<<c[1]<<c[2]<<endl;


    //send mavlink msg


    waitKey(100);
    imshow("Frame", frame);
    imshow("crop1",crop1);
    imshow("crop2",crop2);
    imshow("crop3",crop3);
}


// Release the video capture and close the display window
cap.release();
//destroyAllWindows();


return 0;
}