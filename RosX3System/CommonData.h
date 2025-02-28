#pragma once
#include "Framework.h"

std::string getLocalIP();
void setLocalIP(const std::string& ip);

std::string getServersIP();
void setServersIP(const std::string& ip);

int getLocalPort();
void setLocalPort(int port);

bool getIsServer();
void setServerStatus(bool isServer);
bool programClose();
void setProgramClose(bool close);