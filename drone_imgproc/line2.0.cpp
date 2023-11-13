#include <iostream>
//#include <opencv2/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <zbar.h>
     
using namespace cv;
using namespace std;
using namespace zbar; 

typedef struct {
  string type;
  string data;
  vector <Point> location;
} decodedObject;
void lineqr(VideoCapture &cap);

void decode(Mat &im, vector<decodedObject>&decodedObjects);
void display(Mat &im, vector<decodedObject> &decodedObjects);

int main(){

//webcam
VideoCapture cap0(0);
// VideoCapture cap1(1);

//video
// string path = "video2.mp4";
// VideoCapture cap0(path);
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
    float area_max=0.0;
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
    clock_t start, end;
    vector<decodedObject> decodedObjects;
    
while (true)
{
    decodedObjects.clear();
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
    

    //qr
    decode(frame, decodedObjects);
    display(frame, decodedObjects);
        
    //send mavlink msg

    //only for video !!!!!
    //waitKey(100);
    end = clock();
    fpslive = double(CLOCKS_PER_SEC)/(double(end)-double(start));
    cout<<c[0]<<c[1]<<c[2]<<endl;
    imshow("frame", frame);

}}

int readData(string &data) { // -> decode
    static int mission = 0;
    int index = int(data.back())-48;
    string debugID = "QR :: readData :: ";

    if(index - mission == 1) mission++;
    //if(DEBUG_TXT) {
        cout << "mission " << mission << endl;
        //if(debug_txt_qr) cout << debugID << "returning " << data[mission*2] << endl;
    //} 
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
    cout << "Type : " << obj.type << endl;
    cout << "Data : " << obj.data << endl << endl;
    
    //obtain location
    for(int i = 0; i < symbol->get_location_size(); i++) {
      obj.location.push_back(Point(symbol->get_location_x(i), symbol->get_location_y(i)));
    }
    switch(readData(obj.data)) {
      case 'N':
        cout << "go North" << endl;
        break;
      case 'E':
        cout << "go East" << endl;
        break;
      case 'S':
        cout << "go South" << endl;
        break;
      case 'W':
        cout << "go West" << endl;
        break;
      default:
        cout << "land" << endl;
    }

    decodedObjects.push_back(obj);
  }
}

void display(Mat &im, vector<decodedObject> &decodedObjects) { // -> lineXqr

    
    //if(!(DEBUG_CAM && debug_cam_qr)) return;
    string debugID = "QR :: display ::";
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
  //imshow(debugID+"Results", im);
}
