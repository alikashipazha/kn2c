//Reference:https://www.learnopencv.com/opencv-qr-code-scanner-c-and-python/

#include <iostream>
#include <algorithm>
#include <vector>
#include <zbar.h>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
/*#include </home/kn2c/zbar-master/zbar/qrcode/bch15_5.h>
#include </home/kn2c/zbar-master/zbar/qrcode/binarize.h>
#include </home/kn2c/zbar-master/zbar/qrcode/isaac.h>
#include </home/kn2c/zbar-master/zbar/qrcode/qrdec.h>
#include </home/kn2c/zbar-master/zbar/qrcode/rs.h>
#include </home/kn2c/zbar-master/zbar/qrcode/util.h>*/

using namespace std;
using namespace cv;
using namespace zbar;

typedef struct
{
  string type;
  string data;
  vector <Point> location;
}decodedObject;

// Find and decode barcodes and QR codes
void decode(Mat &im, vector<decodedObject>&decodedObjects)
{

  // Create zbar scanner
  ImageScanner scanner;

  // Configure scanner
  //scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 0); //https://stackoverflow.com/questions/56068886/how-to-configure-c-zbar-scanner-to-decode-only-qr-code-data-type
  scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1); //youtube
  //scanner.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);

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
      obj.location.push_back(cv::Point(symbol->get_location_x(i), symbol->get_location_y(i)));
    }

    decodedObjects.push_back(obj);
  }
}

// Display barcode and QR code location
void display(Mat im, Mat show, vector<decodedObject> decodedObjects) //& &
{
  // Loop over all decoded objects
  
    show = im.clone();
 
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
      //line(show, hull[j], hull[ (j+1) % n], Scalar(randColor1,randColor2,randColor3), 3);
      polylines(show, hull, true, Scalar(randColor1,randColor2,randColor3), 2);
      putText(show, decodedObjects[i].data, hull[0], FONT_HERSHEY_DUPLEX, 0.75, Scalar(200, 0, 150),1.5);
      //im.release();
    }
 
  }
 
  // Display results
  imshow("Results", show);
  waitKey(10);
 
}

int main(int argc, char *argv[])
{
  // Read image
  //string imagepath = argv[1];
  //Mat im = imread(imagepath);
  //// vector<vector<Point>>contours; ////
  //// vector<Point>contours_approx; ////

  //youtube
    Mat im, show;
    VideoCapture cap(0); //-1
    cap.set(CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CAP_PROP_FRAME_HEIGHT, 720);
 
  vector<decodedObject> decodedObjects;
   // Variable for decoded objects
    //QrCodeDetector qrCodeDetector; //
    while(1) {
        //im = Mat(); /////////////////////////////////
        im.release();
        decodedObjects.clear();
        cap>>im;
      //  vector<Point> qrCodeCorners; //
      //  qrCodeDetector.detectAndDecode(im, qrCodeCorners); //
        decode(im, decodedObjects);
      //  if(decodedObjects.size()) polylines(im, qrCodeCorners, true, cv::Scalar(255, 0, 255), 2);
      //// approxPolyDP(contours[0],contours_approx,100,true); ////
      //// drawContours(im,contours,-1,Scalar(255,0,0)); ////
        display(im,show, decodedObjects);
        //////////// imshow("Results", im);
        //////////// waitKey(100);
        //if(decodedObjects.size()) waitKey(0);
        
    }

   // Find and decode barcodes and QR codes
   
   return 0;
 }