#ifndef NETWORKBASE_H
#define NETWORKBASE_H
#include "Datatypes.h"
#include "tcpClient/tcp_client.h"
#include "vendor/AESBuffer.h"
#define Sleep(x) sleep(x)

bool initNetwork();
void closeNetwork();

void sendTextMessage(std::string&);
void sendTask(const Task& request);
extern RDA::AESBuffer streamBuffer;

#endif // !NETWORKBASE_H