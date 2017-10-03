#include <opencv2/opencv.hpp>
#include <time.h>
#include <chrono>
#include <thread>

#include "Geometry.h"

using namespace cv;

void drawCenterOfRoad(Mat & drawIn,std::vector<Point2D> centerOfRoad) {
    for(int i=0; i<centerOfRoad.size()-1; ++i) {
        Point start(centerOfRoad[i].x,centerOfRoad[i].y);
        Point end(centerOfRoad[i+1].x,centerOfRoad[i+1].y);
        line(drawIn,start,end,Scalar(0),2);
    }
}

int main(int argc, char** argv) {

/* Mat img = imread(argv[1]);
// Create binary image from source image
cv::Mat bw;
cv::cvtColor(img, bw, CV_BGR2GRAY);
imshow("img",bw);
cv::threshold(bw, bw, 128, 255, CV_THRESH_BINARY); */

    std::vector<Point2D> centerOfRoad = { Point2D(10,10), Point2D(100,170), Point2D(400,500) };

    Mat bw(500,500,CV_8UC1);
    bw = Scalar(255);
    drawCenterOfRoad(bw, centerOfRoad);


    imshow("bw",bw);

    clock_t start = clock();
    cv::Mat dist;
    cv::distanceTransform(bw, dist, CV_DIST_L2, 3);


    std::cout << "value at 10,10 = " << dist.at<float>(Point(20,10)) << std::endl;

    std::cout << "runtime (DT) = " << (float)((clock() - start))/CLOCKS_PER_SEC << std::endl;

     cv::normalize(dist, dist, 0, 1., cv::NORM_MINMAX);
    imshow("distance",dist);
    waitKey();

}
