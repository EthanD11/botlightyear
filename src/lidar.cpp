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
IChannel* channel;
ILidarDriver *lidar;



//SLeep
#include <unistd.h>

void StartLidar(){
    ///  Create a communication channel instance
    //TODO 2 lidar ici modif port chanel ?? 256000
    Result<IChannel*> _channel = createSerialPortChannel("/dev/ttyUSB0", 115200); //port série spécifié est "/dev/ttyUSB0" avec un débit de 115200 bps.
    if (_channel.err) {
        fprintf(stderr, "Failed to create communication channel\r\n");
        return;
    }
    channel = _channel.value;

    Result<ILidarDriver *> _lidar = createLidarDriver();
    if (_lidar.err) {
        fprintf(stderr, "Failed to create Lidar driver\r\n");
        return;
    }
    lidar = _lidar.value;
    u_result res = lidar->connect(channel);
    if(SL_IS_OK(res)){
        sl_lidar_response_device_info_t deviceInfo;
        res = lidar->getDeviceInfo(deviceInfo);
        if(SL_IS_OK(res)){
            //TODO recup noms lidar ici pour faire la diff entre les 2
            printf("Model: %d, Firmware Version: %d.%d, Hardware Version: %d\n",
                   deviceInfo.model,
                   deviceInfo.firmware_version >> 8, deviceInfo.firmware_version & 0xffu,
                   deviceInfo.hardware_version);

            std::vector<LidarScanMode> scanModes;
            lidar->getAllSupportedScanModes(scanModes);

            LidarScanMode scanMode;
            lidar->startScan(false, true, 0, &scanMode);

        }else{
            fprintf(stderr, "Failed to get device information from LIDAR %08x\r\n", res);
        }
    }else{
        fprintf(stderr, "Failed to connect to LIDAR %08x\r\n", res);
    }
}


void updateData(double* angles, double* distances, double* quality, int arraySize){
    sl_lidar_response_measurement_node_hq_t nodes[arraySize];
    size_t nodeCount = sizeof(nodes)/sizeof(sl_lidar_response_measurement_node_hq_t);
    u_result res = lidar->grabScanDataHq(nodes, nodeCount);

    if (SL_IS_OK(res)){
        lidar->ascendScanData(nodes, nodeCount);//cfr sl_lidar_driver.cpp
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

void DataToFile(string filename){
    sl_lidar_response_measurement_node_hq_t nodes[8192];//default on rplidar
    size_t nodeCount = sizeof(nodes)/sizeof(sl_lidar_response_measurement_node_hq_t);
    sl_result res = lidar->grabScanDataHq(nodes, nodeCount);

    if (SL_IS_OK(res)){
        lidar->ascendScanData(nodes, nodeCount);//cfr sl_lidar_driver.cpp
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

void StopLidar(){
    lidar->stop();
    delete lidar;
    delete channel;
}



void faussemain()
{      
    
    StartLidar();
    for (size_t i = 118; i < 119; i++)
    {
        //sleep(10);
        DataToFile("testBottom"+std::to_string(i)+".txt");
        printf("turn\n");
    }
    
    StopLidar();
    
 }

