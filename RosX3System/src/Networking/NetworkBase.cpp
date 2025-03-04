#include "NetworkBase.h"
#include "CommonData.h"

#include "vendor/concurrent_buffers.h"


SPSCBuffer<Task, 200> syncronizer;
bool requestResolved = true;
Request currentRequest;

void onIncomingMsg(const char* msg, size_t size) {
	std::cout << "Got msg from server: " << msg << "\n";
}

// observer callback. will be called when server disconnects
void onDisconnection(const pipe_ret_t& ret) {
	std::cout << "Server disconnected: " << ret.message() << "\n";
}