#ifndef COMMONDATA_H
#define COMMONDATA_H

#include "Framework.h"

#define NUMBER_OF_CAMERAS 2
#define NUMBER_OF_ARM_JOINTS 6

extern int camera1Size[2];
extern int camera2Size[2];
extern std::vector<pid_t> runningProcesses;
extern std::atomic<bool> closeProgram;
std::string getLocalIP();
void setLocalIP(const std::string& ip);

std::string getServersIP();
void setServersIP(const std::string& ip);

int getLocalPort();
void setLocalPort(int port);

bool getIsServer();
void setServerStatus(bool isServer);
void setProgramClose(bool close);

#endif // !COMMONDATA_H