#include "cameraTag.h"

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <cmath>
using namespace cv;

int tagDetectionOrientation(){
    VideoCapture cap(0, CAP_V4L);
    if (!cap.isOpened()) {
        std::cerr << "Camera start error" << std::endl;
        return 11;
    }


    //TODO modif param 50 ? nbr max de tag detecté
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);

    Mat frame;
    cap >> frame;

    // Convertir l'image en niveaux de gris
    Mat gray;
    cvtColor(frame, gray, COLOR_BGR2GRAY);

    //TODO PARAM calibrage
    Mat cameraMatrix,  distCoeffs;
    cameraMatrix = (Mat1f(3, 3) << 600, 0, 400, 0, 510, 170, 0, 0, 1);
    distCoeffs = (Mat1f(4, 1) << 0.2, -0.3, -0.002, 0.005);

    // Détection des tags ArUco
    std::vector<int> markerIds;
    std::vector<std::vector<Point2f>> markerCorners;
    aruco::detectMarkers(gray, dictionary, markerCorners, markerIds);
    int* tag = new int[markerIds.size()+1];
    tag[0]=markerIds.size();
    if (markerIds.size() > 0) {
        std::vector<Vec3d> rvecs, tvecs;
        aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);

        for (size_t i = 0; i < markerIds.size(); ++i) {
            tag[i+1]=markerIds[i];

            //color
            double a1 = rvecs[i][0];
            double a2 = rvecs[i][1];
            if (std::abs(a1-a2)<0.25){
                cap.release();
                destroyAllWindows();
                return 1;//yellow
            }
            else{
                if (std::abs(a1+a2)<0.25){
                    cap.release();
                    destroyAllWindows();
                    return 0;//blue
                }
                else{
                    cap.release();
                    destroyAllWindows();
                    return 10;//blue and yellow
                }
            }
        }

    }
    // Libérer la caméra
    cap.release();
    destroyAllWindows();
    return 11;
}


void tagDetectionValue(int* tag){
    VideoCapture cap(0, CAP_V4L);
    if (!cap.isOpened()) {
        std::cerr << "Camera start error" << std::endl;
        return;
    }

    //TODO modif param 50 ? nbr max de tag detecté
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);

	Mat frame;
	cap >> frame;

	// Convertir l'image en niveaux de gris
	Mat gray;
	cvtColor(frame, gray, COLOR_BGR2GRAY);

    //TODO PARAM calibrage 
    Mat cameraMatrix,  distCoeffs;
    cameraMatrix = (Mat1f(3, 3) << 600, 0, 400, 0, 510, 170, 0, 0, 1);
    distCoeffs = (Mat1f(4, 1) << 0.2, -0.3, -0.002, 0.005);

	// Détection des tags ArUco
	std::vector<int> markerIds;
	std::vector<std::vector<Point2f>> markerCorners;
	aruco::detectMarkers(gray, dictionary, markerCorners, markerIds);
    tag[0]=markerIds.size();
	if (markerIds.size() > 0) {
        std::vector<Vec3d> rvecs, tvecs;
        aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);
        for (size_t i = 0; i < markerIds.size(); ++i) {
            tag[i+1]=markerIds[i];
            }
	}
    // Libérer la caméra
    cap.release();
    destroyAllWindows();
    return ;
}


int main() {
    int tags[50];
    tagDetectionValue(tags);
    int size = tags[0];
    if (size!=0){
        for (int i = 1; i <= size; ++i) {
            printf("tag : %i\n", tags[i]);
        }
    }
    else{
        printf("no tags detect\n");
    }
    
    int color = tagDetectionOrientation();
    if (color==0){
        printf("blue\n");
    } else if (color==1){
        printf("yellow\n");
    } else if (color==10){
        printf("blue and yellow\n");
    } else {
        printf("no tag\n");
    }
    
    return 0;
}