#include "CommonData.h"

static std::string localIP;
static std::string serversIP;
static int port = 8000;

static bool isServer = false;
std::atomic<bool> closeProgram;
int camera1Size[2];
int camera2Size[2];

std::string getLocalIP() {
	return localIP;
}

void setLocalIP(const std::string& ip) {
	localIP = ip;
}

std::string getServersIP() {
	return serversIP;
}

void setServersIP(const std::string& ip) {
	serversIP = ip;
}

int getLocalPort() {
	return port;
}

void setLocalPort(int port) {
	port = port;
}

bool getIsServer() {
	return isServer;
}

void setServerStatus(bool isServer) {
	isServer = isServer;
}
void setProgramClose(bool close) {
	closeProgram = close;
}