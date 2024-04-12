#include "../headers/cameraTag.h"

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <cmath>
using namespace cv;


double tagSolar(){
    system("libcamera-still  -o tag0.jpg –camera 0 -t 1 -v 0 ");
    cv::Mat image = cv::imread("tag0.jpg");
    if (image.empty()) {
        std::cerr << "Erreur : Impossible de charger l'image." << std::endl;
        return 600;
    }
    

    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    //Detection ARUCO
    cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, parameters);

    //Calibration matrix
    Mat cameraMatrix,  distCoeffs;    
    cameraMatrix = (Mat1f(3, 3) << 2013.1301, 0, 719.5, 0, 2013.1301, 539.5, 0, 0, 1);
    distCoeffs = (Mat1f(4, 1) <<-0.236433, 0.0344246, 0, 0, 0);
    double a1, a2,a3,d1, d2,d3, d;
    //Orientation
    double dtot=100;
    if (markerIds.size() > 0) {
        std::vector<Vec3d> rvecs, tvecs;
        aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);
        for (size_t i = 0; i < markerIds.size(); ++i) {
            if (markerIds[i]==47&&(abs(tvecs[i][0])<abs(dtot))){
                //panneau solaire
                a1 = rvecs[i][0];
                a2 = rvecs[i][1];
                a3 = rvecs[i][2];
                d1 = tvecs[i][0];
                d2 = tvecs[i][1];
                d3 = tvecs[i][2];

                dtot=d1;
                a1 = std::fmod((a1 +M_PI), (2*M_PI))-M_PI;
                a2 = std::fmod((a2 +M_PI), (2*M_PI))-M_PI;
                a3 = std::fmod((a3 +M_PI), (2*M_PI))-M_PI;
                //printf("0\n %.2f\n %.2f\n %.2f\n %.2f\n %.2f\n %.2f\n\n",a1*180/M_PI, a2*180/M_PI,a3*180/M_PI, d1, d2, d3);
            }
        }
    }
    a1= a1*180.0/M_PI;
    a2 = a2*180.0/M_PI;
    if (a1>a2){
        double angle = (a1+260.3)/(-1.29);
        while (angle>180){
            angle-=360;
        }
        while (angle<-180){
            angle+=360;
        }
        return angle;
    }else{
        double angle= (a2-34.9)/1.17;
        while (angle>180){
            angle-=360;
        }
        while (angle<-180){
            angle+=360;
        }
        return angle;
    }
    return 5000;
}