#include "Networking/NetworkBase.h"
#include "CommonData.h"

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


std::string ip = "192.168.80.68";
int main(){
	std::cout << "Please introduce server ip: ";
	//std::cin >> ip;
	std::vector<uint8_t> imageBuffer;
	getImageFromCamera(imageBuffer);
	setServersIP(ip);
	setLocalPort(8000);
	initNetwork();
	bool continueLoop = true;
	while (continueLoop){
		Task currentTask;	
		if(taskBuffer.pop(currentTask)){
			switch (currentTask.type)
			{
			case TaskType::Get:
				if(currentTask.infoType == InfoType::Camera){
					Task uploadTask;
					uploadTask.type = TaskType::Post;
					uploadTask.infoType = InfoType::Camera;
					uploadTask.data.size = imageSize;
					uploadTask.data.data = reinterpret_cast<char*>(malloc(imageSize + sizeof(int)));
					*reinterpret_cast<int*>(uploadTask.data.data) = 0;
					memcpy(uploadTask.data.data + sizeof(int), imageBuffer.data(), imageSize);
					sendTask(uploadTask);
					free(uploadTask.data.data);
				}
				break;
			default:
				std::cout << "Forgot to add other request handling\n";
				break;
			}
		}
		sleep(1);
	}
	
	std::cin.get();
}