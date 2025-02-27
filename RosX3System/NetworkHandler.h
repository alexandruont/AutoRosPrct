#pragma once
#include "Framework.h"
class NetworkHandler
{
    int sock;
    struct sockaddr_in serverAddr;
#ifdef _WIN32
    WSADATA wsa;
#endif
public:
    NetworkHandler() : sock(-1) {}
    bool connectToServer(const std::string& ip, int port);
    bool sendData(const std::string& data);
    std::string receiveData();
    void closeConnection();
    ~NetworkHandler();
};