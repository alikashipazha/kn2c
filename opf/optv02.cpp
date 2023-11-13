#include <iostream>
#include <opencv2/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <chrono>       //time

using namespace cv;
using namespace std;
int main(int argc, char **argv)
{

VideoCapture cap(0);

//string path = "video.mp4";
//VideoCapture cap(path);

Mat old_frame, old_gray;
vector<Point2f> p0, p1;
vector<Scalar> colors;
RNG rng;
Mat frame, frame_gray;
vector<uchar> status;
vector<float> err;
int num;

if (!cap.isOpened()){
//error in opening the video input
cerr << "Unable to open file!" << endl;
return 0;
}
// Create some random colors

for(int i = 0; i < 100; i++)
{
   int r = rng.uniform(0, 256);
   int g = rng.uniform(0, 256);
   int b = rng.uniform(0, 256);
   colors.push_back(Scalar(r,g,b));
   }

// Take first frame and find corners in it
cap>> old_frame;
// auto begin = chrono::high_resolution_clock::now();
cvtColor(old_frame, old_gray, COLOR_BGR2GRAY);
   
   

while(true){
   
   goodFeaturesToTrack(old_gray, p0, 100, 0.3, 7, Mat(), 7, false, 0.04);
   // Create a mask image for drawing purposes
   Mat mask = Mat::zeros(old_frame.size(), old_frame.type());
   cap>> frame;

   // auto end = chrono::high_resolution_clock::now();
   // auto elapsed = chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
   // auto begin = chrono::high_resolution_clock::now();
   string fps;
   int aa = cap.get(CAP_PROP_FPS);
   fps = to_string(aa);
   Point org(50, 50); 
   putText(frame, fps, org , FONT_HERSHEY_DUPLEX, 1 , Scalar(200, 0, 150),1.5);


   if (frame.empty())
      break;
   cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
   // calculate optical flow
   TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
   calcOpticalFlowPyrLK(old_gray, frame_gray, p0, p1, status, err, Size(30,30), 2, criteria);
   vector<Point2f> good_new, good_old;
   num =0;
   for(uint i = 0; i < p0.size(); i++)
   {
   // Select good points
   
      if(status[i] == 1) {
         good_new.push_back(p1[i]);
         good_old.push_back(p0[i]);
         // draw the tracks
         line(mask,p1[i], p0[i], colors[i], 2);
         circle(frame, p1[i], 5, colors[i], -1);
         num++;
   }
   }
   Mat img;
   add(frame, mask, img);
   imshow("Frame", img);
   int keyboard = waitKey(20);
   if (keyboard == 'q' || keyboard == 27){
   break;}
   // calculate mean of vector and print dx and dy
   old_gray = frame_gray.clone();
   Mat delta = Mat(good_new)- Mat(good_old) ;
   Scalar x = mean(delta);
   
   // Now update the previous frame and previous points
   // p0 = good_new;
   int flow_quality = round(255.0 * num / status.size());
   // cout<<"flow quality = "<<flow_quality<<endl;
   cout<<"right(dx)= "<<x[0]<<"   forward(dy)="<<x[1]<<"     flow quality = "<<flow_quality<<endl;
   // cout<<"dt = "<<elapsed.count() * 1e-9<<endl;
 

   }
    }
