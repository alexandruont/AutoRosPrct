#include "NetworkHandler.h"
#include "NetworkBase.h"
#include "CommonData.h"
#include "Syncronizer.h"

NetworkHandler handler;
Syncronizer syncronizer;
bool requestResolved = true;
Request currentRequest;

void NetworkFunction() {
	handler = NetworkHandler();
	handler.connectToServer(getServersIP(), getLocalPort());

	while (!programClose()){
		if (handler.dataAvailable()) {
			handler.receiveData();
			if (!requestResolved) {
				ArrayData data = handler.receiveData();
				if (data.size == currentRequest.size) {
					// Process data
					// TODO: Implement data processing
					requestResolved = true;
				}
			}
		}
	}
}