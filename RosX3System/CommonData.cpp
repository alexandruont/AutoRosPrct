#include "CommonData.h"

static std::string localIP;
static std::string serversIP;
static int port;

static bool isServer = false;
static bool closeProgram = false;

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

bool programClose() {
	return closeProgram;
}

void setProgramClose(bool close) {
	closeProgram = close;
}