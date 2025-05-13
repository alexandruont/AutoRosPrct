#include "Networking/NetworkBase.h"
#include "ChasisController/Task_Executer.h"
#include "CommonData.h"
#include <unistd.h>
#include <signal.h>
#include <iostream>
#include <cstdlib>

const int imageSize = 256 * 256 * 3;
std::string ip;

pid_t launchProcess(const std::string& launchFile) {
    pid_t pid = fork();
    if (pid == 0) {
        // Child process
        execlp("roslaunch", "roslaunch", launchFile.c_str(), (char*) nullptr);
        exit(1);  // Only if execlp fails
    }
    return pid;
}

bool isRoscoreRunning() {
    return system("rosnode list > /dev/null 2>&1") == 0;
}

bool startSubProcess() {
    // Simulate starting a subprocess
    std::cout << "Starting subprocesses\n";
    if (isRoscoreRunning()) {
        std::cout << "roscore is already running\n";
    } else {
        std::cout << "Starting roscore\n";
        system("gnome-terminal -- bash -c 'roscore'");
    }
    runningProcesses.push_back(launchProcess("yahboomcar_nav  laser_astrapro_bringup.launch"));
    runningProcesses.push_back(launchProcess("yahboomcar_nav yahboomcar_navigation.launch use_rviz:=false map:=rrt_map"));
    return true;
}

int main(int argc, char** argv) {
    closeProgram.store(false);
    if(!startSubProcess()){
        std::cerr << "Failed to start subprocesses.\n";
        return 1;
    }
    std::cout << "Please introduce server ip: ";
    std::cin >> ip;
    if (ip == "") {
        ip = "192.168.80.55";
    }    
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
            for (pid_t pid : runningProcesses) {
                kill(pid, SIGINT);
            }
            closeNetwork();
        }
    }
}