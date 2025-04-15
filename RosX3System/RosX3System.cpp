#include "Networking/NetworkBase.h"
#include "ChasisController/Task_Executer.h"
#include "CommonData.h"
#include <iostream>

const int imageSize = 256 * 256 * 3;

std::string ip = "192.168.80.15";
int main(int argc, char** argv) {
    closeProgram.store(false);
    std::cout << "Please introduce server ip: ";
    std::cout << ip << '\n';
    setServersIP(ip);
    setLocalPort(8000);
    if (!initNetwork()) {
        return 0;
    }
    if(!initTaskExecuter(argc, argv)){
        closeProgram.store(true);
        closeNetwork();
        return 0;
    }
    std::string data;
    while (!closeProgram.load()){
        std::cin >> data;
        if (data == "stop"){
            closeProgram.store(true);
            closeNetwork();
        }
    }

}