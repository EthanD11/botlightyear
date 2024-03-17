#include "../headers/cameraTag.h"

int main(int argc, char const *argv[]){
    tagFromJpg("vin1.jpg");
    /*
        int tags[50];
        tagDetectionValue(tags);
        int size = tags[0];
        if (size >50){
            printf("size : %d \n", size);
            size=50;
        }
        if (size!=0){
            for (int i = 1; i <= size; ++i) {
                printf("tag : %i ", tags[i]);
            }
        }
        else{
            printf("no tags detect\n");
        }

        /*int color = tagDetectionOrientation();
        if (color==0){
            printf("blue\n");
        } else if (color==1){
            printf("yellow\n");
        } else if (color==10){
            printf("blue and yellow\n");
        } else {
            printf("no tag\n");
        }*/
}