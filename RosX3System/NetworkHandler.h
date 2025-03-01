#pragma once
#include "Datatypes.h"

class NetworkHandler
{
    int sock = -1;
    struct sockaddr_in serverAddr;
#ifdef _WIN32
    WSADATA wsa;
#endif
public:
    bool connectToServer(const std::string& ip, int port);
    bool sendData(ArrayData&);
    bool dataAvailable();
	size_t availableDataSize();
    ArrayData receiveData();
	ArrayData receiveData(size_t);
    void closeConnection();
    ~NetworkHandler();
};