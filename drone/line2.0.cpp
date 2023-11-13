#include <iostream>
//#include <opencv2/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
     
using namespace cv;
using namespace std;


void lineqr(VideoCapture &cap);

int main(){

//webcam
VideoCapture cap0(1);
// VideoCapture cap1(1);

//video
//string path = "video2.mp4";
//VideoCapture cap0(path);
// VideoCapture cap1(path);

//
lineqr(cap0);

//creat threads



// Release the video capture and close the display window
cap0.release();
// cap1.release();
//destroyAllWindows();
return 0; 
}

void lineqr(VideoCapture &cap){

    //define variables
    int thresh = 80000;
    int area_max_i;
    double area_max=0;
    Mat frame, hsv, mask;
    Mat crop1, crop2, crop3, crop, mask2;
    vector<vector<Point>> contours;
    Scalar lower_black(0, 0, 0);
    Scalar upper_black(179, 255, 40);
    Point2i p0;
    int width = static_cast<int>(cap.get(3));
    int height = static_cast<int>(cap.get(4));
    float left_right, fpslive;
    int num_black;
    bool c[2];
    QRCodeDetector qrCodeDetector;
    clock_t start, end;

while (true)
{
    cap >> frame;
    start = clock();
    //fine black road
    resize(frame, frame ,Size() ,0.5, 0.5);
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
    // if(-2<left_right<2){
    //     cout<<"forward"<<endl;
    // }
    // else 
    if(left_right>0){
        cout<<"left"<<left_right<<endl;
    }
    else if(left_right<0){
        cout<<"right:"<<left_right<<endl;
    }

    //yaw 
    //crop frame to 3
    // mask2 = mask(Rect(0,int(height/3),int(width),int(height/3)));
    // crop1 = mask2(Rect(0,0,int(width/3),height/3));
    // crop2 = mask2(Rect(int(width/3),0,width/3,height/3));
    // crop3 = mask2(Rect(int(2*width/3),0,width/3,height/3));

    //crop, count number of black pixles, creat sensor output
    for(int i=0; i<3 ;i++){
        crop = mask(Rect(int((width/3)*(i)),0,(width/3),height));
        num_black = countNonZero(crop);
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
    // shold not happen in real
    else if(c[0]==1 && c[1]==0 && c[2]==0){
        cout<<"yaw +15 to turn left "<<endl;
    }
    else if(c[0]==0 && c[1]==0 && c[2]==1){
        cout<<"yaw -15 to turn right "<<endl;
    }
    else{
        cout<<"stop"<<endl;
        //read qr and find aproprate yaw 
    }
    

    vector<Point> qrCodeCorners;
    string qrCodeData = qrCodeDetector.detectAndDecode(frame, qrCodeCorners);

    if (!qrCodeData.empty())
    {
        cout << "QR code detected: " << qrCodeData << endl;
        polylines(frame, qrCodeCorners, true, cv::Scalar(255, 0, 255 ), 2);
        putText(frame, qrCodeData, qrCodeCorners[0], FONT_HERSHEY_DUPLEX, 0.75, Scalar(200, 0, 150),1.5);
    }

        
    //send mavlink msg

    //only for video !!!!!
    //waitKey(100);
    end = clock();
    fpslive = double(CLOCKS_PER_SEC)/(double(end)-double(start));
    cout<<c[0]<<c[1]<<c[2]<<endl;
    imshow("frame", frame);

    //for debug
    // imshow("crop1",crop1);
    // imshow("crop2",crop2);
    // imshow("crop3",crop3);
}

}
