#include "NetworkBase.h"
#include "CommonData.h"
TcpClient client;

Task currentTask;
size_t currentLoaded = 0;

RDA::AESBuffer streamBuffer(1024);
void onIncomingMsg(const char* msg, size_t size);
void onDisconnection(const pipe_ret_t& ret);
void sendSpecs();

void initNetwork() {
    currentTask.type = TaskType::NULL_TASK;
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
    sendSpecs();
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
    Request req(task);
    client.sendMsg(reinterpret_cast<char*>(&req), sizeof(Request));
    client.sendMsg(reinterpret_cast<char*>(task.data.data), task.data.size);
}

void sendSpecs(){
    Request specReq;
    specReq.type = TaskType::Specs;
    specReq.infoType = InfoType::None;
    specReq.size = 0;
    client.sendMsg(reinterpret_cast<char*>(&specReq), sizeof(Request));
    SpecStruct st = SpecStruct();
    client.sendMsg(reinterpret_cast<char*>(&st), sizeof(SpecStruct));
}

void onIncomingMsg(const char* msg, size_t size) {
    std::cout<< "Incoming bytes: " << size << std::endl;
    streamBuffer.write(msg, size);
}

// observer callback. will be called when server disconnects
void onDisconnection(const pipe_ret_t& ret) {
	std::cout << "Server disconnected: " << ret.message() << "\n";
}