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
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

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
  scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 0); //https://stackoverflow.com/questions/56068886/how-to-configure-c-zbar-scanner-to-decode-only-qr-code-data-type
  scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1); //youtube
  //scanner.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);

  // Convert image to grayscale
  Mat imGray;
  cvtColor(im, imGray,CV_BGR2GRAY);

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
    decodedObjects.push_back(obj);
  }
}

int main(int argc, char *argv[])
{
  // Read image
  //string imagepath = argv[1];
  //Mat im = imread(imagepath);
  
  //youtube
    Mat im;
    VideoCapture cap(0); //-1
    cap.set(CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CAP_PROP_FRAME_HEIGHT, 720);
    namedWindow("Results", -1);

   // Variable for decoded objects
    while(1) {
        vector<decodedObject> decodedObjects;
        cap>>im;
        vector<Point> qrCodeCorners; //
        polylines(im, qrCodeCorners, true, cv::Scalar(255, 0, 255 ), 2); //
        decode(im, decodedObjects);
        imshow("Results", im);
        waitKey(1);
        if(decodedObjects.size()) waitKey(0);
    }

   // Find and decode barcodes and QR codes
   
   return 0;
 }
