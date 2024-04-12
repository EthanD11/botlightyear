#include "../headers/cameraTag.h"
#include <chrono>

int main(int argc, char const *argv[]){
    //for (int i = 0; i < 14; ++i) {
        //printf("%d\n",i);
        auto started = std::chrono::high_resolution_clock::now();
        double pri = tagFromJpg();
        
    auto done = std::chrono::high_resolution_clock::now();
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(done-started).count()<< "\n";
    
        printf("90 -- %f \n",pri);
    //}
}