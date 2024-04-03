#include "../headers/cameraTag.h"

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

#include <cmath>
using namespace cv;
bool blue = true; //TODO voir ou mettre ceci

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

void tagFromJpg(std::string filename){
    cv::Mat image = cv::imread(filename.c_str());
    if (image.empty()) {
        std::cerr << "Erreur : Impossible de charger l'image." << std::endl;
        return;
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
            //TODO check les vrais code et return bleu-jaune ou blanc-mauve + dist eventuelle
            /*std::cout << "Code ArUco ID: " << markerIds[i] << std::endl;
            std::cout << "Rotation (en degrés) : " << rvecs[i] * 180 / CV_PI << std::endl; // Convertir l'orientation en radians en degrés
            std::cout << "Translation : " << tvecs[i] << std::endl;*/

            if (markerIds[i]==47){
                //panneau solaire
                double a1 = rvecs[i][0];
                double a2 = rvecs[i][1];
                printf("%f %f\n",a1*180/M_PI, a2*180/M_PI);
                //double a = asin(((a1*180/M_PI)-120)/120)*180/M_PI;
                //double a = asin((a1-2)/2)*180/M_PI;
                /*if (a2>0&&a2<M_PI){
                    a=180-a;
                }
                if (blue){
                    printf("turn %f°\n",a);
                }else{
                    printf("turn %f°\n",(a-180));
                }*/
                if (blue){
                    if (std::abs(a1+a2)<0.25){
                        printf("blue\n");
                        break;
                    }
                    if (std::abs(a1-a2)<0.25){
                        printf("180°\n");
                        break;
                    }
                    if (std::abs(a1)<0.25){
                        printf("-90°");
                        break;
                    }
                    if (std::abs(a2)<0.25){
                        printf("90");
                        break;
                    }

                }else{
                    if (std::abs(a1+a2)<0.25){
                        printf("180°\n");
                        break;
                    }
                    if (std::abs(a1-a2)<0.25){
                        printf("yellow\n");
                        break;
                    }
                    if (std::abs(a1)<0.25){
                        printf("90°");
                        break;
                    }
                    if (std::abs(a2)<0.25){
                        printf("-90");
                        break;
                    }
                }
            }
        }
    }

    //printf("\n");
    /*cv::aruco::drawDetectedMarkers(image, markerCorners, markerIds); //TODO DELETE juste utile pour voir ce qu'il se passe (dessine contour tag)
    cv::imshow("Codes ArUco détectés", image);  //TODO DELETE juste utile pour voir ce qu'il se passe (affiche image)
    cv::waitKey(0);*/
    return;
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

/*
int main(int argc, char const *argv[]) {
    for (int i = 0; i < 14; ++i) {
        tagFromJpg("CameraTest/"+std::to_string(i)+".jpg");

    }
}*/
