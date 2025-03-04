#include "Networking/NetworkBase.h"
#include "CommonData.h"

std::string ip;

int main(){
	std::cout << "Please introduce server ip: ";
	std::cin >> ip;

	setServersIP(ip);
	setLocalPort(8000);
	initNetwork();
	bool continueLoop = true;
	std::string textToSend;
	while (continueLoop){
		std::cout << "Text to send\n";
		std::cin >> textToSend;
		if (textToSend == "stop") {
			break;
		}
		sendTextMessage(textToSend);
	}
}