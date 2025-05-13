#pragma once
#include <cstdlib> // For malloc and free
#include <iostream>
#include <mutex>

namespace RDA {
struct sh_all {
    struct sha_header {
        uint16_t counter;
    };
    sh_all() = default;
    sh_all(size_t pSize) : _size(pSize) {
        std::lock_guard<std::mutex> lock(_mutex);
        _all = malloc(pSize + sizeof(sha_header));
        if(!_all)
            throw std::bad_alloc();
        reinterpret_cast<sha_header*>(_all)->counter = 1;
    }
    sh_all(const sh_all& other) {
        std::lock_guard<std::mutex> lock(other._mutex);
        this->_all = other._all;
        this->_size = other._size;
        if(_all) {
            sha_header* t = reinterpret_cast<sha_header*>(_all);
            t->counter++;
        }
    }
    ~sh_all() {
        std::lock_guard<std::mutex> lock(_mutex);
        if(!_all) {
            return;
        }
        sha_header* t = reinterpret_cast<sha_header*>(_all);
        t->counter--;
        if(t->counter == 0) {
            free(_all);
        }
        _all = nullptr;
        _size = 0;
    }
    sh_all& operator=(const sh_all& other) {
        if(this == &other) return *this;

        std::lock_guard<std::mutex> lock(_mutex);
        std::lock_guard<std::mutex> other_lock(other._mutex);

        if(_all) {
            sha_header* t = reinterpret_cast<sha_header*>(_all);
            t->counter--;
            if(t->counter == 0) {
                free(_all);
            }
        }
        if(other._all) {
            auto other_header = other.getHeader();
            std::lock_guard<std::mutex> other_lock(other_header->header_mutex);
            other_header->counter++;
        }
        _all = other._all;
        _size = other._size;
        return *this;
    }

    void* operator->() const {
        std::lock_guard<std::mutex> lock(_mutex);
        return reinterpret_cast<sha_header*>(_all) + 1;
    }
    void* getRawPointer() {
        std::lock_guard<std::mutex> lock(_mutex);
        return reinterpret_cast<sha_header*>(_all) + 1;
    }

  private:
    void* _all = nullptr;
    size_t _size = 0;
    mutable std::mutex _mutex; // Mutex to protect shared resources
};
}