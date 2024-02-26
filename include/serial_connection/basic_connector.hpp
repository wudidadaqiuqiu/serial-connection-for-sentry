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
    ~basic_connector() { close(); }

    bool open(const char* file_path) {
        fd = ::open(file_path, O_RDWR | O_NOCTTY | O_NDELAY);
        fcntl(fd, F_SETFL, 0);
        has_opened = true;
        if (fd == -1) {
            return false;
        }
        termios newtio{};
        newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
        newtio.c_iflag = 0;
        newtio.c_oflag = 0;
        newtio.c_lflag = 0;
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

    int read() {
        if (fd == -1) return -1;
        // std::cout << "basic connector read\n";
        int res = reader_.read(fd);
        if (res == -1) {
            std::cout << "errno is: " << errno << std::endl;
        }
        return res;
    }

    const uint8_t* get() {
        return reader_.buf;
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

    class reader {
       public:
        uint8_t buf[127];
        int read(file_descriptor_t fd) {
            int res = ::read(fd, buf, sizeof(buf));
            return res;
        }
    };

    reader reader_;
};

}  // namespace SerialConection