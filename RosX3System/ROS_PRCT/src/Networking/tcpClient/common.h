#ifndef COMMON_H
#define COMMON_H

#include <cstdio>

#define MAX_PACKET_SIZE 4096

namespace fd_wait {
    enum Result {
        FAILURE,
        TIMEOUT,
        SUCCESS
    };

    Result waitFor(const FileDescriptor &fileDescriptor, uint32_t timeoutSeconds = 1);
};

#endif // !COMMON_H