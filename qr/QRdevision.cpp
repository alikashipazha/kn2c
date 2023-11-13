#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

int main()
{
    cv::VideoCapture cap(0);
    if (!cap.isOpened())
    {
        std::cout << "Failed to open the camera." << std::endl;
        return -1;
    }

    cv::QRCodeDetector qrCodeDetector;
    
    while (true)
    {
        cv::Mat frame;
        cap >> frame; 

        if (frame.empty())
        {
            std::cout << "No frame captured from the camera." << std::endl;
            break;
        }

        std::vector<cv::Point> qrCodeCorners;
        std::string qrCodeData = qrCodeDetector.detectAndDecode(frame, qrCodeCorners);

        if (!qrCodeData.empty())
        {
            std::cout << "QR code detected: " << qrCodeData << std::endl;

            cv::polylines(frame, qrCodeCorners, true, cv::Scalar(0, 255, 0), 2);
        }

        cv::imshow("QR Code Detection", frame);

        if (cv::waitKey(1) == 'q')
        {
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();
    
    return 0;
}
