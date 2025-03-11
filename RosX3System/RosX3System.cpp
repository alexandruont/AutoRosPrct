#include "Networking/NetworkBase.h"
#include "CommonData.h"
#include <iostream>
const int imageSize = 640 * 480 * 3;

void getImageFromCamera(std::vector<uint8_t>& buffer) {
    // Simulate an image of 640x480 with 3 channels (RGB)
    const int width = 640;
    const int height = 480;
    const int channels = 3;
    const int imageSize = width * height * channels;
    buffer.resize(imageSize);
    // Fill the buffer with dummy data (e.g., all pixels set to 255)
    std::fill(buffer.begin(), buffer.end(), 255);
}


std::string ip = "192.168.80.92";
int main(){
	std::cout << "Please introduce server ip: ";
	//std::cin >> ip;
	std::vector<uint8_t> imageBuffer;
	int32_t* buf = reinterpret_cast<int32_t*>(malloc(imageBuffer.size() + sizeof(int32_t)));
	*buf = 0;
	memcpy(buf + 1, imageBuffer.data(), imageBuffer.size());
	getImageFromCamera(imageBuffer);
	setServersIP(ip);
	setLocalPort(8000);
	initNetwork();
	bool continueLoop = true;
	while (continueLoop){
		Request req;
		streamBuffer.read(reinterpret_cast<char*>(&req), sizeof(Request));
		switch (req.type)
		{
		case TaskType::Get:
			if(req.infoType == InfoType::Camera){
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
			break;
		default:
			std::cerr << "Unkown Task";
			break;
		}
		sleep(1);
	}
}