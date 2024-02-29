#pragma once
#include <chrono>
#include <string>
#include <mylog.hpp>
namespace SerialConection {
struct framerate_t {
    std::string name;
    uint32_t count;
    std::chrono::high_resolution_clock::time_point last;
    double fps;
    framerate_t(const std::string& name_) : name(name_), last(std::chrono::high_resolution_clock::now()){}
    void update() {
        count ++;
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - last);
        if (duration.count() > 1000) {
            fps = count * 1000.0 / duration.count();
            count = 0;
            last = currentTime;
            mylog("the frequency of", name, "is", fps);
        }
    }
};
}