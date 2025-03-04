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
	
	std::cin.get();
}