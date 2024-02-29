#pragma once
#include <iostream>


namespace SerialConection {

void mylog() {
    std::cout << std::endl;
}

template <typename T, typename... Args>
void mylog(T f, Args... args) {
    std::cout << f << " ";
    mylog(args...);
}

}