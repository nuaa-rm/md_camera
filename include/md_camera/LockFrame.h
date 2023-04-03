//
// Created by bismarck on 23-4-3.
//

#ifndef SRC_LOCKFRAME_H
#define SRC_LOCKFRAME_H

#include <atomic>
#include <cstdint>
#include <CameraDefine.h>

class LockFrame {
private:
    uint8_t *memory = nullptr;
    tSdkFrameHead head;
    std::atomic<bool> used = false;
public:
    bool lock();
    void release();

    uint8_t *operator()();
    uint8_t *data();
    uint8_t **ptr();
    tSdkFrameHead* headPtr();
};

class LockFramePool {
private:
    LockFrame* pool = nullptr;
    int size = 0;

    void free();
public:
    explicit LockFramePool(int _size);
    LockFramePool() = delete;
    LockFramePool(LockFramePool&) = delete;
    LockFramePool(LockFramePool&&) noexcept;
    LockFramePool& operator=(LockFramePool&&) noexcept;
    ~LockFramePool();

    LockFrame *getSlot();
    void resetSlotSize(int _size);
};

#endif //SRC_LOCKFRAME_H
