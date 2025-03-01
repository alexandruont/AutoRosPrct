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

bool NetworkHandler::sendData(ArrayData& data) {
    if (send(sock, reinterpret_cast<char*>(data.data), data.size, 0) < 0) {
        std::cerr << "Send failed\n";
        return false;
    }
    return true;
}

ArrayData NetworkHandler::receiveData() {
	ArrayData data;
	data.size = availableDataSize();
	data.data = new char[data.size];
    if (!recv(sock, data.data, data.size, 0)) {
        std::cerr << "Receive failed or connection closed\n";
		delete[] data.data;
        return ArrayData();
    }
    return data;
}

ArrayData NetworkHandler::receiveData(size_t size) {
	ArrayData data;
	data.size = size;
	data.data = new char[size];
	if (!recv(sock, data.data, size, 0)) {
		std::cerr << "Receive failed or connection closed\n";
		delete[] data.data;
		return ArrayData();
	}
	return data;
}

size_t NetworkHandler::availableDataSize() {
#ifdef _WIN32
    u_long bytesAvailable = 0;
    ioctlsocket(sock, FIONREAD, &bytesAvailable);
    return static_cast<size_t>(bytesAvailable);
#else
    size_t bytesAvailable = 0;
    ioctl(sock, FIONREAD, &bytesAvailable);
    return bytesAvailable;
#endif // _WIN32
}

bool NetworkHandler::dataAvailable() {
	return availableDataSize() > 0;
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