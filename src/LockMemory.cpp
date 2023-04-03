//
// Created by bismarck on 23-4-3.
//

#include <cstdlib>
#include <iostream>
#include "md_camera/LockFrame.h"

bool LockFrame::lock() {
    bool _t = false;
    return used.compare_exchange_strong(_t, true);
}

void LockFrame::release() {
    used = false;
}

uint8_t *LockFrame::operator()() {
    return memory;
}

uint8_t *LockFrame::data() {
    return memory;
}

uint8_t **LockFrame::ptr() {
    return &memory;
}

tSdkFrameHead *LockFrame::headPtr() {
    return &head;
}


LockFramePool::LockFramePool(int _size) {
    pool = new LockFrame[_size];
    size = _size;
}

LockFramePool::LockFramePool(LockFramePool &&other) noexcept {
    if (&other == this) {
        return;
    }
    pool = other.pool;
    other.pool = nullptr;
    size = other.size;
    other.size = 0;
}

LockFramePool& LockFramePool::operator=(LockFramePool&& other) noexcept {
    if (&other == this) {
        return *this;
    }
    pool = other.pool;
    other.pool = nullptr;
    size = other.size;
    other.size = 0;
    return *this;
}

LockFramePool::~LockFramePool() {
    free();
    delete[] pool;
}

LockFrame *LockFramePool::getSlot() {
    for (int i = 0; i < size; i++) {
        if (pool[i].lock()) {
            return pool + i;
        }
    }
    std::cerr << "Memory Pool Used Out !!!" << std::endl;
    return nullptr;
}

void LockFramePool::free() {
    for (int i = 0; i < size; i++) {
        if (pool[i]() != nullptr) {
            ::free(pool[i]());
        }
    }
}

void LockFramePool::resetSlotSize(int _size) {
    free();
    for (int i = 0; i < size; i++) {
        *pool[i].ptr() = (uint8_t*)malloc(_size);
    }
}
