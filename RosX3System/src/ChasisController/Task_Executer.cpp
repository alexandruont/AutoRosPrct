// #include <CommonData.h>
// #include <Datatypes.h>
#include "Networking/NetworkBase.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "ChasisHandler.h"

const int imageSize = 256 * 256 * 3;
std::thread TaskExec;
void executeTasks();
bool initTaskExecuter(int argc, char** argv) {
    initChasisHandler(argc, argv);
    TaskExec = std::thread(executeTasks);
    return true;
}

void closeTaskExecuter() {
    closeChasisHandler();
    while(!TaskExec.joinable())
        sleep(1);
    TaskExec.join();
}

void getImageFromCamera(std::vector<uint8_t>& buffer) {
    buffer.resize(imageSize);
    // Fill the buffer with dummy data (e.g., all pixels set to 255)
    std::fill(buffer.begin(), buffer.end(), 255);
}

void handleGetRequest(Request& req) {
    // Setup dummy camera buffer
    std::vector<uint8_t> imageBuffer(imageSize);
    int32_t* buf = reinterpret_cast<int32_t*>(malloc(imageBuffer.size() + sizeof(int32_t)));
    *buf = 0;
    memcpy(buf + 1, imageBuffer.data(), imageBuffer.size());
    switch(req.infoType) {
    case InfoType::Movement: {
        std::cout << "Getting movement\n";
        RP::vec3 res;
        res.x = chMovement.x * 100;
        res.y = chMovement.y * 100;
        std::cout << "Movement: " << res.x << " , " << res.y << '\n';
        streamBuffer.write(reinterpret_cast<char*>(&res), sizeof(RP::vec3));
        break;
    }
    case InfoType::Camera: {
        std::cout << "Getting camera\n";
        // Simulate getting an image from the camera
        getImageFromCamera(imageBuffer);
        std::cout << "Camera image size: " << imageBuffer.size() << '\n';
        Task ts;
        ts.type = TaskType::Post;
        ts.infoType = InfoType::Camera;
        ts.data.data = reinterpret_cast<char*>(buf);
        ts.data.size = imageBuffer.size() + sizeof(int32_t);
        sendTask(ts);
        break;
    }
    case InfoType::Map: {
        std::cout << "Getting map\n";

        // Simulate getting a map
    }
    break;
    case InfoType::Arm: {
        std::cout << "Getting arm\n";
        // Simulate getting arm data
    }
    break;
    default:
        std::cerr << "Unknown Task" << std::endl;
        break;
    }
}

void executeTasks() {
    while(true) {
        Request req;
        streamBuffer.read(reinterpret_cast<char*>(&req), sizeof(Request));
        if(closeProgram.load()) {
            break;
        }
        switch(req.type) {
        case TaskType::Get:
            handleGetRequest(req);
            break;
        case TaskType::Post:
            break;
        case TaskType::Set:
            if(req.infoType == InfoType::Movement) {
                double movement[2];
                streamBuffer.read(reinterpret_cast<char*>(movement), req.size);
                RP::vec3 res;
                res.x = movement[0] / 100;
                res.y = movement[1] / 100;
                std::cout << "Setting movement to: " << res.x << " , " << res.y << '\n';
                chMovement = res;
            }
            break;
        default:
            std::cerr << "Unknown Task" << std::endl;
            break;
        }
        sleep(1);
    }
}