#pragma once
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cstdint>
#include <string>

namespace SerialConection {

using file_descriptor_t = int;

class basic_connector {
   public:
    basic_connector() : fd(-1), has_opened(false) {}
    ~basic_connector() { close();}

    bool open(const char* file_path) {
        fd = ::open(file_path, O_RDWR | O_NOCTTY | O_NDELAY);
        has_opened = true;
        if (fd == -1) {
            return false;
        }
        termios newtio{};
        newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
        newtio.c_cc[VTIME] = 0;
        newtio.c_cc[VMIN] = 1;
        tcflush(fd, TCIOFLUSH);
        tcsetattr(fd, TCSANOW, &newtio);
        // 设置为非阻塞模式，这个在读串口的时候会用到
        //  fcntl(fd, F_SETFL, O_NONBLOCK);

        return true;
    }

    void close() {
        std::cerr << "basic connector close entering" << std::endl;
        if (fd >= 0) ::close(fd);
        fd = -1;
        has_opened = false;
    }

    bool write(const uint8_t* data, size_t len) {
        if (fd == -1) return false;
        if (writer::write(fd, data, len)) {
            return true;
        } else {
            fd = -1;
            return false;
        }
    }

   private:
    file_descriptor_t fd;
    bool has_opened;

    class writer {
       public:
        static bool write(file_descriptor_t fd, const uint8_t* data, size_t len) {
            if (fd != -1) {
                return (::write(fd, data, len) != -1);
            }
            return false;
        }
    };
};

}  // namespace SerialConection