#include "NetworkHandler.h"
#include "NetworkBase.h"
#include "CommonData.h"
#include "concurrent_buffers.h"

NetworkHandler handler;
SPSCBuffer<Task, 200> syncronizer;
bool requestResolved = true;
Request currentRequest;

void NetworkFunction() {
	handler = NetworkHandler();
	handler.connectToServer(getServersIP(), getLocalPort());

	while (!programClose()){
		// Checking for enought data to read the request body
		if (requestResolved == false) {
			if (currentRequest.size < handler.availableDataSize()) {
				continue;
			}
			requestResolved = true;
			goto PUSH_TASK;
		}
		// Checking for enought data to read the request header
		if (handler.availableDataSize() < sizeof(Request))
			continue;
		// Load the request header
		currentRequest = *reinterpret_cast<Request*>(handler.receiveData(sizeof(Request)).data);
		if (currentRequest.size > handler.availableDataSize()){
			requestResolved = false;
			continue;
		}
		// Start processing the request
		PUSH_TASK:
		Task task;
		task.type = currentRequest.type;
		task.infoType = currentRequest.infoType;
		if (currentRequest.size){
			task.data = handler.receiveData(currentRequest.size);
		}
		task.data.size = currentRequest.size;
		task.callback = nullptr;
		syncronizer.push(task);
	}
}