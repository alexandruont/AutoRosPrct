// #include <CommonData.h>
// #include <Datatypes.h>
#include "Networking/NetworkBase.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "ChasisHandler.h"

const int imageSize = 256 * 256 * 3;
std::thread TaskExec;
void executeTasks();
bool initTaskExecuter(int argc, char** argv){
    initChasisHandler(argc, argv);
    TaskExec = std::thread(executeTasks);
    return true;
}

void getImageFromCamera(std::vector<uint8_t>& buffer) {
    // Simulate an image of 640x480 with 3 channels (RGB)
    const int width = 256;
    const int height = 256;
    const int channels = 3;
    const int imageSize = width * height * channels;
    buffer.resize(imageSize);
    // Fill the buffer with dummy data (e.g., all pixels set to 255)
    std::fill(buffer.begin(), buffer.end(), 255);
}

void executeTasks(){
    // Setup dummy camera buffer
    std::vector<uint8_t> imageBuffer;
    getImageFromCamera(imageBuffer);
    int32_t* buf = reinterpret_cast<int32_t*>(malloc(imageBuffer.size() + sizeof(int32_t)));
    *buf = 0;
    memcpy(buf + 1, imageBuffer.data(), imageBuffer.size());

    while (true) {
        Request req;
        streamBuffer.read(reinterpret_cast<char*>(&req), sizeof(Request));
        if(closeProgram.load()){
            break;
        }
        switch (req.type) {
        case TaskType::Get:
            if (req.infoType == InfoType::Camera) {
                Task ts;
                ts.type = TaskType::Post;
                ts.infoType = InfoType::Camera;
                ts.data.data = reinterpret_cast<char*>(buf);
                ts.data.size = imageBuffer.size() + sizeof(int32_t);
                sendTask(ts);
            }
            break;
        case TaskType::Post:
            break;
        case TaskType::Set:
            if (req.infoType == InfoType::Movement) {
                double movement[2];
                streamBuffer.read(reinterpret_cast<char*>(movement), sizeof(double) * 2);
            }
            break;
        default:
            std::cerr << "Unknown Task" << std::endl;
            break;
        }
        sleep(1);
    }
    closeChasisHandler();
}