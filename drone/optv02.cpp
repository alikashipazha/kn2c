#include <iostream>
#include <opencv2/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
    
using namespace cv;
using namespace std;


int main(int argc, char **argv)
{
//webcam
// VideoCapture cap(0);

//video
string path = "video.mp4";
VideoCapture cap(path);

//make objects(variables)
Mat old_frame,old_resize, old_gray;
vector<Point2f> p0, p1;
vector<Scalar> colors;
RNG rng;
Mat frame,frame_resize, frame_gray;
vector<uchar> status;
vector<float> err;
vector<Point2f> good_new, good_old;
clock_t start, end;
double fpslive;
int num;

//error in opening the video input
if (!cap.isOpened())
{
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

// Take first frame 
cap>> old_frame;
resize(old_frame, old_resize ,Size() ,0.5, 0.5);
cvtColor(old_resize, old_gray, COLOR_BGR2GRAY);
// cvtColor(old_frame, old_gray, COLOR_BGR2GRAY);
//Get cammera FPS
int cam_fps = cap.get(CAP_PROP_FPS);
cout<<"cammera frame per second: "<<cam_fps<<endl;
     
while(true){
   
   //get new frame
   cap>> frame;
   start = clock();
   if (frame.empty())
      break;
   resize(frame, frame_resize ,Size() ,0.5, 0.5);
   cvtColor(frame_resize, frame_gray, COLOR_BGR2GRAY);
   // cvtColor(frame, frame_gray, COLOR_BGR2GRAY);

   //find feature points in old
   goodFeaturesToTrack(old_gray, p0, 20, 0.3, 7, Mat(), 7, false, 0.04);

   // Create a mask image for drawing purposes
   // Mat mask = Mat::zeros(old_frame.size(), old_frame.type());

   
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
   cout<<"right(dx)= "<<x[0]<<"   forward(dy)="<<x[1]<<"     flow quality = "<<flow_quality<<endl;

   waitKey(100);

   //calculate FPS
   end = clock();
   fpslive = double(CLOCKS_PER_SEC)/(double(end)-double(start));
   string fps=to_string(fpslive);
   Point org(50, 50); 
   putText(frame_resize, fps, org , FONT_HERSHEY_DUPLEX, 1 , Scalar(200, 0, 150),1.5);
   

   imshow("Frame", frame_resize);

}

// Release the video capture and close the display window
   cap.release();
   destroyAllWindows();
   return 0;

}