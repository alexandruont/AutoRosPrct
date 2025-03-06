#ifndef NETWORKBASE_H
#define NETWORKBASE_H
#include "Datatypes.h"
#include "tcpClient/tcp_client.h"
#include "vendor/concurrent_buffers.h"
#define Sleep(x) sleep(x)

void initNetwork();
void closeNetwork();

void sendTextMessage(std::string&);
void sendTask(const Task& request);
extern SPSCBuffer<Task, 50> taskBuffer;

#endif // !NETWORKBASE_H