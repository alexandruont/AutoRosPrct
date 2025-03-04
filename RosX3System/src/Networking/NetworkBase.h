#pragma once
#include "Datatypes.h"
#include "tcpClient/tcp_client.h"

void onIncomingMsg(const char* msg, size_t size);
void onDisconnection(const pipe_ret_t& ret);