#pragma once
#include "shared_allocation.h"
#include <iostream>
#include <atomic>
#include <cstring>

namespace RDA {
    struct AESBuffer {
        AESBuffer(size_t startSize) : _size(startSize), _available(startSize), readI(0), writeI(0) {
            _all = sh_all(startSize);
        }

        void write(const char* buffer, size_t pAmount) {
            size_t cA = _available.load(std::memory_order_relaxed);
            size_t readC = readI.load(std::memory_order_acquire);
            size_t writeC = writeI.load(std::memory_order_acquire);
            if (cA < pAmount) {
                cA = _size + pAmount * 2;
                sh_all temp = sh_all(cA);
                if (readC <= writeC) {
                    memcpy(temp.getRawPointer(), _all.getRawPointer(), writeC);
                    size_t tmp = _size - readC;
                    memcpy(reinterpret_cast<char*>(temp.getRawPointer()) + cA - tmp, reinterpret_cast<char*>(_all.getRawPointer()) + readC, tmp);
                }
                else {
                    memcpy(temp.getRawPointer(), _all.getRawPointer(), writeC);
                    memcpy(reinterpret_cast<char*>(temp.getRawPointer()) + cA - (_size - readC), reinterpret_cast<char*>(_all.getRawPointer()) + readC, _size - readC);
                }
                _all = temp;
                _size = cA;
            }
            char* bf = reinterpret_cast<char*>(_all.getRawPointer());
            cA -= pAmount;
            if (pAmount > _size - writeC) {
                memcpy(bf + writeC, buffer, _size - writeC);
                buffer += _size - writeC;
                pAmount -= _size - writeC;
                writeC = 0;
            }
            memcpy(bf + writeC, buffer, pAmount);
            writeC += pAmount;
            writeI.store(writeC, std::memory_order_release);
            _available.store(cA, std::memory_order_release);
            cv.notify_all();
        }

        void read(char* buffer, size_t pAmount) {
            std::unique_lock<std::mutex> lock(mtx);
            cv.wait(lock, [&] { 
                size_t available = _size - _available.load(std::memory_order_acquire);
                std::cout << "We got info: " << available << " >= " << pAmount << '\n';
                return available >= pAmount; 
            });

            sh_all tAll(_all);
            size_t cA = _available.load(std::memory_order_relaxed);
            char* bf = reinterpret_cast<char*>(_all.getRawPointer());
            size_t readC = readI.load(std::memory_order_acquire);
            cA -= pAmount;
            if (pAmount > _size - readC) {
                memcpy(buffer, bf + readC, _size - readC);
                buffer += _size - readC;
                pAmount -= _size - readC;
                readC = 0;
            }
            memcpy(buffer, bf + readC, pAmount);
            readC += pAmount;
            readI.store(readC, std::memory_order_release);
            _available.store(cA, std::memory_order_release);
        }

    private:
        sh_all _all;
        std::atomic<size_t> _available;
        size_t _size;
        std::atomic<size_t> readI;
        std::atomic<size_t> writeI;
        std::mutex mtx;
        std::condition_variable cv;
    };
}
