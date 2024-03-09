/*
#include <iostream>
#include <ctime>
// #include "../raspicam-master/raspicam-master/src/raspicam_cv.h"
#include <unistd.h>
#include <opencv2/opencv.hpp>

int main22() {
    // Initialiser la caméra
    raspicam::RaspiCam_Cv camera;
    if (!camera.open()) {
        std::cerr << "Erreur : impossible d'ouvrir la caméra." << std::endl;
        return -1;
    }

    // Attendre que la caméra se stabilise (temps de démarrage)
    sleep(1);
    //std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Capturer une image
    cv::Mat image;
    camera.grab();
    camera.retrieve(image);

    // Vérifier si l'image capturée est vide
    if (image.empty()) {
        std::cerr << "Erreur : l'image capturée est vide." << std::endl;
        return -1;
    }

    // Sauvegarder l'image capturée
    std::string filename = "image_capturee.jpg";
    cv::imwrite(filename, image);
    std::cout << "Image capturée et sauvegardée sous : " << filename << std::endl;

    // Fermer la caméra
    camera.release();

    return 0;
}
*/
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


void capture() {
    // Ouvrir la webcam
    cv::VideoCapture cap(1, CAP_V4L);
    /*if (!cap.isOpened()) {
        std::cerr << "Erreur : Impossible d'ouvrir la webcam." << std::endl;
        return;
    }

    // Capturer une image
    cv::Mat frame;
    cap >> frame;

    // Vérifier si l'image est vide
    if (frame.empty()) {
        std::cerr << "Erreur : Aucune image capturée." << std::endl;
        return;
    }

    // Enregistrer l'image sur le disque
    std::string filename = "captured_image.jpg";
    bool success = cv::imwrite(filename, frame);

    // Vérifier si l'enregistrement a réussi
    if (!success) {
        std::cerr << "Erreur : Impossible d'enregistrer l'image." << std::endl;
        return;
    }

    std::cout << "L'image a été enregistrée avec succès sous le nom : " << filename << std::endl;
    */
    return;
}

void tagDetectionValue(int* tag){
    VideoCapture cap(1, CAP_V4L);
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
    //capture();
    
    int tags[50];
    tagDetectionValue(tags);
    int size = tags[0];
    /*if (size!=0){
        for (int i = 1; (i <= size && i <=50); ++i) {
            printf("tag : %i\n", tags[i]);
        }
    }
    else{
        printf("no tags detect\n");
    }*/
    /*
    int color = tagDetectionOrientation();
    if (color==0){
        printf("blue\n");
    } else if (color==1){
        printf("yellow\n");
    } else if (color==10){
        printf("blue and yellow\n");
    } else {
        printf("no tag\n");
    }*/
    
    return 0;
}