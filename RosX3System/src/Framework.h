#ifndef FRAMEWORK_H
#define FRAMEWORK_H

#include <iostream>
#include <cstring>
#include <thread>
#include <queue>
#include <chrono>
#include <string>
#include <vector>
// Syncronous communication
#include <atomic>
#include <condition_variable>
#include <mutex>
#ifdef _WIN32
#include <winsock2.h>
#include <Ws2tcpip.h>  // Required for InetPton
#pragma comment(lib, "ws2_32.lib")
#define inet_pton(family, addr, dst) InetPtonA(family, addr, dst)  // Use InetPtonA for narrow strings
#else
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

#endif // !FRAMEWORK_H
