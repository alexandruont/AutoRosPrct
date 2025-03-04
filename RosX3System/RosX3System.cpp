#include "Networking/NetworkBase.h"
#include "CommonData.h"

std::string ip = "127.0.0.1";

int main(){
	setServersIP(ip);
	TcpClient client;

	client_observer_t observer;
	observer.wantedIP = "127.0.0.1";
	observer.incomingPacketHandler = onIncomingMsg;
	observer.disconnectionHandler = onDisconnection;
	client.subscribe(observer);

    bool connected = false;
    while (!connected) {
        pipe_ret_t connectRet = client.connectTo("127.0.0.1", 8000);
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

    bool shouldTerminate = false;
    std::string message = "Hello From Client";
    client.sendMsg(message.c_str(), message.size());
    Sleep(2);
    const pipe_ret_t closeResult = client.close();
    if (!closeResult.isSuccessful()) {
        std::cout << "closing client failed: " << closeResult.message() << "\n";
    }
    else {
        std::cout << "closed client successfully\n";
    }

}