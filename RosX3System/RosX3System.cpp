#include "NetworkBase.h"
#include "NetworkHandler.h"
#include "CommonData.h"

std::string ip = "";

int main(){
	setServersIP(ip);
	std::thread networkThread(NetworkFunction);

}