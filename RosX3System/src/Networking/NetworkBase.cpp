#include "NetworkBase.h"
#include "CommonData.h"

TcpClient client;

SPSCBuffer<Task, 50> taskBuffer;
void onIncomingMsg(const char* msg, size_t size);
void onDisconnection(const pipe_ret_t& ret);

void initNetwork() {
	client_observer_t observer;
	observer.wantedIP = getServersIP();
	observer.incomingPacketHandler = onIncomingMsg;
	observer.disconnectionHandler = onDisconnection;
	client.subscribe(observer);

    bool connected = false;
    while (!connected) {
        pipe_ret_t connectRet = client.connectTo(getServersIP(), getLocalPort());
        connected = connectRet.isSuccessful();
        if (connected) {
            std::cout << "Client connected successfully\n";
        }
        else {
            std::cout << "Client failed to connect: " << connectRet.message() << "\n"
                << "Make sure the server is open and listening\n\n";
            Sleep(2);
            std::cout << "Retrying to connect...\n";
        }
    };
}

void closeNetwork() {
    const pipe_ret_t closeResult = client.close();
    if (!closeResult.isSuccessful()) {
        std::cout << "closing client failed: " << closeResult.message() << "\n";
    }
    else {
        std::cout << "closed client successfully\n";
    }
}

void sendTextMessage(std::string& message) {
    client.sendMsg(message.c_str(), message.size());
}

void sendTask(const Task& task) {
    std::string message = "Hello From Client";
    client.sendMsg(message.c_str(), message.size());
}

void onIncomingMsg(const char* msg, size_t size) {
    char* buffer = new char[size + 1];
    memcpy(buffer, msg, size);
    buffer[size] = '\0';
	std::cout << "Got msg from server: " << buffer << "\n";
    delete[] buffer;
}

// observer callback. will be called when server disconnects
void onDisconnection(const pipe_ret_t& ret) {
	std::cout << "Server disconnected: " << ret.message() << "\n";
}