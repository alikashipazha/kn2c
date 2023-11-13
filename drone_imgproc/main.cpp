#include <iostream>
#include <opencv2/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <thread>
#include <mutex>
     
using namespace cv;
using namespace std;

mutex frameMutex;
Mat cameraFrame;
int width;
int height;
int cam_fps;

void cameraThread();
void lineqr();
void opticallflow();

int main(){



//creat threads
thread camera_Thread(cameraThread);
thread line_follow(lineqr);
thread opticall_flow(opticallflow);

//joint threads
if(line_follow.joinable()){
    line_follow.join();
}
if(opticall_flow.joinable()){
    opticall_flow.join();
}


return 0; 
}

void cameraThread() {
    //webcam
    // VideoCapture cap0(0);
    // VideoCapture cap1(1);

    //video
    string path = "video2.mp4";
    VideoCapture cap0(path);
    // VideoCapture cap1(path);

    if (!cap0.isOpened()) {
        std::cerr << "Failed to open the camera." << std::endl;
        return;
    }

    width = static_cast<int>(cap0.get(3));
    height = static_cast<int>(cap0.get(4));
    cam_fps = cap0.get(CAP_PROP_FPS);

    while (true) {
        Mat frame;
        cap0 >> frame; // Capture a frame
        {
        lock_guard<mutex> lock(frameMutex);
        cameraFrame = frame.clone(); // Store the frame in the shared variable
        }
        imshow("cam",cameraFrame);
    }
    // Release the video capture and close the display window
    cap0.release();
    // cap1.release();
    //destroyAllWindows();
}


void lineqr(){

    //define variables
    int thresh = 80000;
    int area_max_i;
    double area_max=0;
    Mat frameToProcess, hsv, mask;
    Mat crop1, crop2, crop3, crop, mask2;
    vector<vector<Point>> contours;
    Scalar lower_black(0, 0, 0);
    Scalar upper_black(179, 255, 40);
    Point2i p0;
    double left_right;
    int num_black;
    bool c[2];
    QRCodeDetector qrCodeDetector;

while (true)
{
    
    {
        lock_guard<mutex> lock(frameMutex);
        frameToProcess = cameraFrame.clone(); // Get a copy of the shared frame
    }
    if(frameToProcess.empty()){
        continue;
    }

    //fine black road
    cvtColor(frameToProcess, hsv, COLOR_BGR2HSV);
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
    rectangle(frameToProcess, rect, Scalar(0, 0, 255), 2);
    // cout<<area_max<<endl; 
    
    //find center to follow
    p0.x = rect.x + rect.width/2;
    p0.y = rect.y + rect.height/2;
    circle(frameToProcess, p0, 5, Scalar(0, 0, 255), -1);

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
    mask2 = mask(Rect(0,int(height/3),int(width),int(height/3)));
    crop1 = mask2(Rect(0,0,int(width/3),height/3));
    crop2 = mask2(Rect(int(width/3),0,width/3,height/3));
    crop3 = mask2(Rect(int(2*width/3),0,width/3,height/3));


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
    string qrCodeData = qrCodeDetector.detectAndDecode(frameToProcess, qrCodeCorners);

    if (!qrCodeData.empty())
    {
        cout << "QR code detected: " << qrCodeData << endl;
        polylines(frameToProcess, qrCodeCorners, true, cv::Scalar(255, 0, 255 ), 2);
        putText(frameToProcess, qrCodeData, qrCodeCorners[0], FONT_HERSHEY_DUPLEX, 0.75, Scalar(200, 0, 150),1.5);
    }

        
    //send mavlink msg

    //only for video !!!!!
    waitKey(100);

    cout<<c[0]<<c[1]<<c[2]<<endl;
    imshow("frame", frameToProcess);

    //for debug
    imshow("crop1",crop1);
    imshow("crop2",crop2);
    imshow("crop3",crop3);
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
    int num;


    // Create some random colors
    for(int i = 0; i < 100; i++)
    {
    int r = rng.uniform(0, 256);
    int g = rng.uniform(0, 256);
    int b = rng.uniform(0, 256);
    colors.push_back(Scalar(r,g,b));
    }

    // Take first frame 
       while (old_frame.empty())
       {
        lock_guard<mutex> lock(frameMutex);
        old_frame = cameraFrame.clone(); // Get a copy of the shared frame
         }

    resize(old_frame, old_resize ,Size() ,0.5, 0.5);
    cvtColor(old_resize, old_gray, COLOR_BGR2GRAY);
    // cvtColor(old_frame, old_gray, COLOR_BGR2GRAY);
    //Get cammera FPS
    
    cout<<"cammera frame per second: "<<cam_fps<<endl;
        
    while(true){
    
    //get new frame
    {
        lock_guard<mutex> lock(frameMutex);
        new_frame = cameraFrame.clone(); // Get a copy of the shared frame
    }
       if(new_frame.empty()){
        continue;
    }
    start = clock();
    resize(new_frame, frame_resize ,Size() ,0.5, 0.5);
    cvtColor(frame_resize, frame_gray, COLOR_BGR2GRAY);
    // cvtColor(frame, frame_gray, COLOR_BGR2GRAY);

    //find feature points in old
    goodFeaturesToTrack(old_gray, p0, 20, 0.3, 7, Mat(), 7, false, 0.04);

    
    // calculate optical flow
    TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
    calcOpticalFlowPyrLK(old_gray, frame_gray, p0, p1, status, err, Size(30,30), 2, criteria);
    
    //calculate flow quality
    num =0;
    for(uint i = 0; i < p0.size(); i++)
    {
        if(status[i] == 1) {
            good_new.push_back(p1[i]);
            good_old.push_back(p0[i]);
            // draw the tracks
            line(frame_resize,p1[i], p0[i], colors[i], 1);
            circle(frame_resize, p1[i], 2, colors[i], -1);
            num++;
    }}


    // calculate mean of vector and print dx and dy
    old_gray = frame_gray.clone();
    Mat delta = Mat(good_new)- Mat(good_old) ;
    Scalar x = mean(delta);
    int flow_quality = round(255.0 * num / status.size());


    //mavlink massage
    cout<<"right(dx)= "<<x[0]<<"   forward(dy)="<<x[1]<<"     flow quality = "<<flow_quality<<endl;
    
    //only for video
    waitKey(100);

    //calculate FPS
    end = clock();
    fpslive = double(CLOCKS_PER_SEC)/(double(end)-double(start));
    string fps=to_string(fpslive);
    Point org(50, 50); 
    putText(frame_resize, fps, org , FONT_HERSHEY_DUPLEX, 1 , Scalar(200, 0, 150),1.5);
    

    imshow("Frame", frame_resize);

    }}
