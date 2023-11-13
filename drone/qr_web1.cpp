#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <iostream>

using namespace cv;
using namespace std;


int main()
{
    VideoCapture cap(1);
    if (!cap.isOpened())
    {
        cout << "Failed to open the camera." << endl;
        return 0;
    }

    QRCodeDetector qrCodeDetector;
    
    while (true)
    {
        Mat frame;
        cap >> frame; 

        if (frame.empty())
        {
            cout << "No frame captured from the camera." << endl;
            break;
        }

        vector<Point> qrCodeCorners;
        string qrCodeData = qrCodeDetector.detectAndDecode(frame, qrCodeCorners);

        if (!qrCodeData.empty())
        {
            cout << "QR code detected: " << qrCodeData << endl;
            polylines(frame, qrCodeCorners, true, cv::Scalar(255, 0, 255 ), 2);
            putText(frame, qrCodeData, qrCodeCorners[0], FONT_HERSHEY_DUPLEX, 0.75, Scalar(200, 0, 150),1.5);
        }

        imshow("QR Code Detection", frame);

        if (waitKey(1) == 'q')
        {
            break;
        }
    }

    cap.release();
    destroyAllWindows();
    
    return 0;
}
