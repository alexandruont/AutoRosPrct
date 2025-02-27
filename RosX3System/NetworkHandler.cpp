#include "NetworkHandler.h"
bool NetworkHandler::connectToServer(const std::string& ip, int port) {
#ifdef _WIN32
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        std::cerr << "WSAStartup failed\n";
        return false;
    }
#endif
    // Create socket
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == -1) {
        std::cerr << "Socket creation failed\n";
        return false;
    }

    // Configure server address
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    if (inet_pton(AF_INET, ip.c_str(), &serverAddr.sin_addr) <= 0) {
        std::cerr << "Invalid address\n";
        return false;
    }

    // Connect to server
    if (connect(sock, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        std::cerr << "Connection failed\n";
        return false;
    }

    std::cout << "Connected to server\n";
    return true;
}

bool NetworkHandler::sendData(const std::string& data) {
    if (send(sock, data.c_str(), data.length(), 0) < 0) {
        std::cerr << "Send failed\n";
        return false;
    }
    return true;
}

std::string NetworkHandler::receiveData() {
    char buffer[1024] = { 0 };
    int bytesReceived = recv(sock, buffer, sizeof(buffer) - 1, 0);
    if (bytesReceived <= 0) {
        std::cerr << "Receive failed or connection closed\n";
        return "";
    }
    return std::string(buffer, bytesReceived);
}

void NetworkHandler::closeConnection() {
#ifdef _WIN32
    closesocket(sock);
    WSACleanup();
#else
    close(sock);
#endif
    std::cout << "Connection closed\n";
}

NetworkHandler::~NetworkHandler() {
    if (sock != -1) closeConnection();
}