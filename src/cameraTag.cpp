#include <iostream>
#include <ctime>
#include <raspicam/raspicam_cv.h>
#include <opencv2/opencv.hpp>

int main() {
    // Initialiser la caméra
    raspicam::RaspiCam_Cv camera;
    if (!camera.open()) {
        std::cerr << "Erreur : impossible d'ouvrir la caméra." << std::endl;
        return -1;
    }

    // Attendre que la caméra se stabilise (temps de démarrage)
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

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
