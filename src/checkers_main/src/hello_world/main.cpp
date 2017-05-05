#include "mainwindow.h"
#include <QApplication>
//#include <opencv2/opencv.hpp>
#include <iostream>

int main(int argc, char *argv[])
{
//    cv::Mat image = cv::imread("/home/rijad/Pictures/image1.jpg", CV_LOAD_IMAGE_COLOR);
//    cv::namedWindow("DISPLAY", cv::WINDOW_AUTOSIZE);
//    cv::imshow("DISPLAY", image);
    std::cout<<"Nije uspjelo"<<std::endl;
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
