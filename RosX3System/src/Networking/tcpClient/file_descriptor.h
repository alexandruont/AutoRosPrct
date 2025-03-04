#ifndef FILE_DESCRIPTOR_H
#define FILE_DESCRIPTOR_H

class FileDescriptor {
private:
    int _sockfd = 0;

public:
    void set(int fd) { _sockfd = fd; }
    int get() const { return _sockfd; }
};
#endif // !FILE_DESCRIPTOR_H