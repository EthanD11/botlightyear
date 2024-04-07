#include "lidar.h"

#include "rplidar_sdk/rplidar.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>

//pour plot
#include <iostream>
#include <vector>

using std::string;


//pour lecture fichier
#include <fstream>
#include <sstream>


using namespace sl;
IChannel* channelTop; //diff pour les 2 lidars
ILidarDriver *lidarTop;


//idem pour bottom
IChannel* channelBottom; //diff pour les 2 lidars
ILidarDriver *lidarBottom;


//SLeep
#include <unistd.h>



void StartLidarTop(){
    ///  Create a communication channel instance
    Result<IChannel*> _channel = createSerialPortChannel("/dev/ttyUSB0", 115200); //port série spécifié est "/dev/ttyUSB0" avec un débit de 115200 bps.

    if (_channel.err) {
        fprintf(stderr, "Failed to create communication channel\r\n");
        return;
    }
    channelTop = _channel.value;

    Result<ILidarDriver *> _lidar = createLidarDriver();
    if (_lidar.err) {
        fprintf(stderr, "Failed to create Lidar driver\r\n");
        return;
    }
    lidarTop = _lidar.value;
    u_result res = lidarTop->connect(channelTop);
    if(SL_IS_OK(res)){
        sl_lidar_response_device_info_t deviceInfo;
        res = lidarTop->getDeviceInfo(deviceInfo);
        if(SL_IS_OK(res)){
            //TODO recup noms lidar ici pour faire la diff entre les 2
            printf("Model: %d, Firmware Version: %d.%d, Hardware Version: %d\n",
                   deviceInfo.model,
                   deviceInfo.firmware_version >> 8, deviceInfo.firmware_version & 0xffu,
                   deviceInfo.hardware_version);

            std::vector<LidarScanMode> scanModes;
            lidarTop->getAllSupportedScanModes(scanModes);

            LidarScanMode scanMode;
            lidarTop->startScan(false, true, 0, &scanMode);

        }else{
            fprintf(stderr, "Failed to get device information from LIDAR %08x\r\n", res);
        }
    }else{
        fprintf(stderr, "Failed to connect to LIDAR %08x\r\n", res);
    }
}

void StartLidarBottom(){

    ///  Create a communication channel instance
    Result<IChannel*> _channel = createSerialPortChannel("/dev/ttyUSB1", 256000); //port série spécifié est "/dev/ttyUSB0" avec un débit de 115200 bps.
    

    if (_channel.err) {
        fprintf(stderr, "Failed to create communication channel\r\n");
        return;
    }
    channelBottom = _channel.value;

    Result<ILidarDriver *> _lidar = createLidarDriver();
    if (_lidar.err) {
        fprintf(stderr, "Failed to create Lidar driver\r\n");
        return;
    }
    lidarBottom = _lidar.value;
    u_result res = lidarBottom->connect(channelBottom);
    if(SL_IS_OK(res)){
        sl_lidar_response_device_info_t deviceInfo;
        res = lidarBottom->getDeviceInfo(deviceInfo);
        if(SL_IS_OK(res)){
            //TODO recup noms lidar ici pour faire la diff entre les 2
            printf("Model: %d, Firmware Version: %d.%d, Hardware Version: %d\n",
                   deviceInfo.model,
                   deviceInfo.firmware_version >> 8, deviceInfo.firmware_version & 0xffu,
                   deviceInfo.hardware_version);

            std::vector<LidarScanMode> scanModes;
            lidarBottom->getAllSupportedScanModes(scanModes);

            LidarScanMode scanMode;
            lidarBottom->startScan(false, true, 0, &scanMode);

        }else{
            fprintf(stderr, "Failed to get device information from LIDAR %08x\r\n", res);
        }
    }else{
        fprintf(stderr, "Failed to connect to LIDAR %08x\r\n", res);
    }
}

void updateDataTop(double* angles, double* distances, double* quality, size_t* arraySize){
    sl_lidar_response_measurement_node_hq_t nodes[arraySize[1]];
    size_t nodeCount = sizeof(nodes)/sizeof(sl_lidar_response_measurement_node_hq_t);
    u_result res = lidarTop->grabScanDataHq(nodes, nodeCount);
    arraySize[0]=nodeCount;
    if (SL_IS_OK(res)){
        lidarTop->ascendScanData(nodes, nodeCount);//cfr sl_lidar_driver.cpp
        for(size_t i = 0; i < nodeCount; i++){
            double angle_in_degrees = nodes[i].angle_z_q14 * 90.f / (1 << 14);//cfr readme pour avoir les angles en degré
            double distance_in_meters = nodes[i].dist_mm_q2 / 1000.f / (1 << 2);

            angles[i] = angle_in_degrees/ 180 * M_PI; //angles in radian
            distances[i] = distance_in_meters;
            quality[i] = nodes[i].quality;
        }
    }
}

void updateDataBottom(double* angles, double* distances, double* quality, size_t* arraySize){
    sl_lidar_response_measurement_node_hq_t nodes[arraySize[1]];
    size_t nodeCount = sizeof(nodes)/sizeof(sl_lidar_response_measurement_node_hq_t);
    u_result res = lidarBottom->grabScanDataHq(nodes, nodeCount);
    arraySize[0]=nodeCount;
    if (SL_IS_OK(res)){
        lidarBottom->ascendScanData(nodes, nodeCount);//cfr sl_lidar_driver.cpp
        for(size_t i = 0; i < nodeCount; i++){
            double angle_in_degrees = nodes[i].angle_z_q14 * 90.f / (1 << 14);//cfr readme pour avoir les angles en degré
            double distance_in_meters = nodes[i].dist_mm_q2 / 1000.f / (1 << 2);

            angles[i] = angle_in_degrees/ 180 * M_PI; //angles in radian
            distances[i] = distance_in_meters;
            quality[i] = nodes[i].quality;
        }
    }
}

void updateDataFile(double* angles, double* distances, double* quality, string filename, size_t * arraySize){
    std::ifstream fichier(filename);
    if (!fichier.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
        return;
    }

    ///counts the number of lines in the file
    size_t sizeF = 0;
    std::string ligne;
    while (std::getline(fichier, ligne)) {
        ++sizeF;
    }

    arraySize[0] = sizeF;

    // Return to file start
    fichier.clear();
    fichier.seekg(0, std::ios::beg);

    size_t index = 0;
    while (std::getline(fichier, ligne)) {
        std::istringstream iss(ligne);
        double col1,col2;
        if (iss >> col1 >> col2) {
            angles[index] = col1 / 180 * M_PI;
            distances[index] = col2;
            quality[index] = 0.0; // TODO : to be modified if quality is recorded
            ++index;
        } else {
            std::cerr << "Error: Invalid line format in file" << filename << std::endl;
        }
    }
    fichier.close();
}

void DataToFileTop(string filename){
    sl_lidar_response_measurement_node_hq_t nodes[8192];//default on rplidar
    size_t nodeCount = sizeof(nodes)/sizeof(sl_lidar_response_measurement_node_hq_t);
    sl_result res = lidarTop->grabScanDataHq(nodes, nodeCount);

    if (SL_IS_OK(res)){
        lidarTop->ascendScanData(nodes, nodeCount);//cfr sl_lidar_driver.cpp
        FILE *file;

        ///open write + read; creates file if does not exist, overwrites if already exists
        file = fopen(filename.c_str(), "w+");
        if (file == NULL) {
            perror("Erreur lors de l'ouverture du fichier");
            return;
        }

        fprintf(file,"Angle, Distance\n");

        for(size_t i = 0; i < nodeCount; i++){
            double angle_in_degrees = nodes[i].angle_z_q14 * 90.f / (1 << 14);//cfr readme angles in degree
            double distance_in_meters = nodes[i].dist_mm_q2 / 1000.f / (1 << 2);

            ///quality is not taken into account in order to have information for each position in the file
            fprintf(file,"%f %f\n", angle_in_degrees, distance_in_meters);
        }
        fclose(file);
    }
}

void DataToFileBottom(string filename){
    sl_lidar_response_measurement_node_hq_t nodes[8192];//default on rplidar
    size_t nodeCount = sizeof(nodes)/sizeof(sl_lidar_response_measurement_node_hq_t);
    sl_result res = lidarBottom->grabScanDataHq(nodes, nodeCount);

    if (SL_IS_OK(res)){
        lidarBottom->ascendScanData(nodes, nodeCount);//cfr sl_lidar_driver.cpp
        FILE *file;

        ///open write + read; creates file if does not exist, overwrites if already exists
        file = fopen(filename.c_str(), "w+");
        if (file == NULL) {
            perror("Erreur lors de l'ouverture du fichier");
            return;
        }

        fprintf(file,"Angle, Distance\n");

        for(size_t i = 0; i < nodeCount; i++){
            double angle_in_degrees = nodes[i].angle_z_q14 * 90.f / (1 << 14);//cfr readme angles in degree
            double distance_in_meters = nodes[i].dist_mm_q2 / 1000.f / (1 << 2);

            ///quality is not taken into account in order to have information for each position in the file
            fprintf(file,"%f %f\n", angle_in_degrees, distance_in_meters);
        }
        fclose(file);
    }
}

void StopLidarTop(){
    lidarTop->stop();
    delete lidarTop;
    delete channelTop;
}


void StopLidarBottom(){
    lidarBottom->stop();
    delete lidarBottom;
    delete channelBottom;
}