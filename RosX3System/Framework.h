#pragma once
#include <iostream>
#include <cstring>
#ifdef _WIN32
#include <winsock2.h>
#include <Ws2tcpip.h>  // Required for InetPton
#pragma comment(lib, "ws2_32.lib")
#define inet_pton(family, addr, dst) InetPtonA(family, addr, dst)  // Use InetPtonA for narrow strings
#else
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif