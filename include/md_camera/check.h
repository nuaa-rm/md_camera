//
// Created by bismarck on 23-4-1.
//

#ifndef SRC_CHECK_H
#define SRC_CHECK_H

struct md_camera_exception : public exception {
    int status;

    explicit md_camera_exception(int _status) {
        status = _status;
    }

    const char *what() const noexcept override {
        return "MDCamera failure";
    }
};

#define INIT_CHECK_RETRY()                                                                      \
    static int __COUNT;

#define CHECK(status)                                                                           \
    do {                                                                                        \
        auto ret = (status);                                                                    \
        if (ret != 0) {                                                                         \
            std::cerr << "MDCamera failure: " << ret << std::endl;                              \
            throw md_camera_exception(ret);                                                     \
        }                                                                                       \
    } while (false)

#define CHECK_RETURN(status)                                                                    \
    do {                                                                                        \
        auto ret = (status);                                                                    \
        if (ret != 0) {                                                                         \
            std::cerr << "MDCamera failure: " << ret << std::endl;                              \
            return ret;                                                                         \
        }                                                                                       \
    } while (false)

#define CHECK_RETURN_RETRY(count, status)                                                       \
    __COUNT = 1;                                                                                \
    do {                                                                                        \
        auto ret = (status);                                                                    \
        if (ret != 0) {                                                                         \
            std::cerr << "MDCamera failure: " << ret<< " , retry " << __COUNT << std::endl;                              \
            if (__COUNT++ >= count) return ret;                                                  \
        } else {                                                                                \
            break;                                                                              \
        }                                                                                       \
    } while (true)

#define CHECK_ABORT(status)                                                                     \
    do {                                                                                        \
        auto ret = (status);                                                                    \
        if (ret != 0) {                                                                         \
            std::cerr << "MDCamera failure: " << ret << std::endl;                              \
            abort();                                                                            \
        }                                                                                       \
    } while (false)

#define CHECK_ABORT_RETRY(count, status)                                                        \
    __COUNT = 1;                                                                                \
    do {                                                                                        \
        auto ret = (status);                                                                    \
        if (ret != 0) {                                                                         \
            std::cerr << "MDCamera failure: " << ret << " , retry " << __COUNT << std::endl;    \
            if (__COUNT++ >= count) abort();                                                     \
        } else {                                                                                \
            break;                                                                              \
        }                                                                                       \
    } while (true)

#endif //SRC_CHECK_H
