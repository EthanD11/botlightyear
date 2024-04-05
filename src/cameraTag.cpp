#include "../headers/cameraTag.h"

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

#include <cmath>
using namespace cv;


double tagFromJpg(){
    system("libcamera-still -o tag0.jpg –camera 0 -t 1 -v 0");
    cv::Mat image = cv::imread("tag0.jpg");
    if (image.empty()) {
        std::cerr << "Erreur : Impossible de charger l'image." << std::endl;
        return 600;
    }
    cv::resize(image, image, cv::Size(800, 600)); //TODO DELETE juste utile pour voir ce qu'il se passe
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    //Detection ARUCO
    cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, parameters);


    //Calibration matrix
    Mat cameraMatrix,  distCoeffs;
    cameraMatrix = (Mat1f(3, 3) << 600, 0, 400, 0, 510, 170, 0, 0, 1);
    distCoeffs = (Mat1f(4, 1) << 0.2, -0.3, -0.002, 0.005);

    //Orientation
    if (markerIds.size() > 0) {
        std::vector<Vec3d> rvecs, tvecs;
        aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);
        for (size_t i = 0; i < markerIds.size(); ++i) {
            if (markerIds[i]==47){
                //panneau solaire
                double a1 = rvecs[i][0];
                double a2 = rvecs[i][1];
                double d1 = tvecs[i][0];
                double d2 = tvecs[i][1];
                double d3 = tvecs[i][2];
                printf("%f %f     %f %f %f\n",a1*180/M_PI, a2*180/M_PI, d1, d2, d3);
                return a2-50*d1;

            }
        }
    }
    return 500;
}

