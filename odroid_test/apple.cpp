#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

// the lower and upper boundaries of the HSV color space
const int hsv_lower[3] = {5, 92, 0};
const int hsv_upper[3] = {180, 255, 150};

void getImageMask(const cv::Mat& hsv, cv::Mat& mask) {
	// smooths the image
	cv::Mat blur;
	cv::GaussianBlur(hsv, mask, Size(5, 5), 0, 0 );
	cv::inRange(mask, Scalar(hsv_lower[0], hsv_lower[1], hsv_lower[2]),
				Scalar(hsv_upper[0], hsv_upper[1], hsv_upper[2]), mask);
	// Element = 0: Rect - 1: Cross - 2: Ellipse
	cv::Mat opening_kernel = getStructuringElement(/*Element*/2, Size(4,4));
	cv::morphologyEx(mask, mask, cv::MORPH_OPEN, opening_kernel);
	cv::Mat closing_kernel = getStructuringElement(/*Element*/2, Size(10,10));
	cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, closing_kernel);
}

int main(int, char**)
{
    cv::Mat frame;
    cv::Mat hsv, mask;
    std::vector<Vec4i> hierarchy;
	std::vector<std::vector<cv::Point> > contours;

    frame = cv::imread("DSCN1771.JPG");

    // convert frame to the HSV color space
    cv::cvtColor(frame, hsv, CV_BGR2HSV);

    // get mask
    getImageMask(hsv, mask);

    // find contours in the mask 
    cv::findContours(mask, contours, hierarchy,
                RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0) );

    for(int i = 0; i< contours.size(); i++) {
        Point2f center;
        float radius;
        cv::minEnclosingCircle(contours[i], center, radius);
        // calculate the circularity of the contour
        double area = cv::contourArea(contours[i]);
        double perimeter = cv::arcLength(contours[i], true);
        double circularity = 4*/*PI*/3.14*area/perimeter/perimeter;
        if (radius > 30 && circularity > 0.)
            cv::circle(frame, center, radius, Scalar(0, 255, 255), 2);
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}