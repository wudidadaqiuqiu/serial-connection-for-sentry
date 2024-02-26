#pragma once

#include <functional>
#include <string>
#include <thread>

#include "container/multithread_stream.hpp"
#include "easy_robot_commands/shared_member/concepts.hpp"
#include "exec_cmd.hpp"
#include "serial_connection/basic_connector.hpp"
namespace SerialConection {
class Connector {
   public:
    Connector(const std::string& modalias,
              const std::string& bash_absolute_path,
              const std::function<void(const uint8_t*, size_t)>& recv_notify = nullptr)
        : key(modalias),
          path(bash_absolute_path),
          inner_connector(),
          recv_callback(recv_notify),
          recv_flag(true) { }
    ~Connector() {
        recv_flag = false;
        recv_thread.join();
    }

    void Connect() {
        reconnect();
        ReceiveStart();
    }
    void Transmite(const uint8_t* data, size_t len) {
        std::cout << "Connector Transmite " << len << "bytes" << std::endl;
        if (!inner_connector.write(data, len))
            reconnect();
    }

    void receive_loop() {
        while (recv_flag) {
            int read_len = inner_connector.read();
            if (read_len < 0) {
                std::cout << "read len < 0 enter reconnect, read_len:" << read_len << std::endl;
                reconnect();
            } else {
                // std::cout << "read success!!" << std::endl;
                if (recv_callback != nullptr)
                    recv_callback(inner_connector.get(), read_len);
            }
        }
    }

    void change_recv_callbaclk(const std::function<void(const uint8_t*, size_t)>& c) {
        recv_callback = c;
    }

    void blocked_consume(const uint8_t* data, size_t len) {
        Transmite(data, len);
    }

    template <EasyRobotCommands::can_trigger T>
    Connector& operator<<(T& trigger) {
        auto lambda = [this](T& trigger_) {
            // stream container >> Connector blocked_consume
            trigger_ >> (*this);
        };
        EasyRobotCommands::trigger_operation<T> ope(lambda);
        trigger.register_trigger_operation(ope);
        return (*this);
    }

    void reconnect() {
        if (reconnect_mutex.try_lock() == false) return;
        static int cnt;
        cnt++;
        // 防止不断 reconnect
        if (cnt > 20 && (cnt % 20)) {
            reconnect_mutex.unlock();
            return;
        }
        if (cnt > 100 && (cnt % 101)) {
            reconnect_mutex.unlock();
            return;
        }
        inner_connector.close();
        std::string output;
        try {
            output = exec((". " + path).c_str());
        } catch (const std::exception& e) {
            std::cerr << "Exception: " << e.what() << std::endl;
            reconnect_mutex.unlock();
            return;
        }
        std::string temp;
        std::istringstream ss(output);
        std::vector<std::string> keys;
        std::vector<std::string> values;

        while (std::getline(ss, temp)) {
            if (temp.size() < 10)
                values.push_back("/dev/" + temp);
            else
                keys.push_back(temp);
        }

        std::map<std::string, std::string> dict;
        if (keys.size() != values.size()) {
            reconnect_mutex.unlock();
            return;
        }
        for (size_t i = 0; i < keys.size(); ++i) {
            dict[keys.at(i)] = values.at(i);
        }

        std::cout << "Connector connecting ..." << std::endl;
        if (dict.find(key) != dict.end()) {
            std::cout << "Value for key '" << key << "': " << dict[key] << "found" << std::endl;
            inner_connector.open(dict[key].c_str());
        } else {
            std::cout << "Key '" << key << "' not found in the map." << std::endl;
        }

        // end:
        reconnect_mutex.unlock();
    }

    // basic_connector& get_inner_connector() {
    //     return inner_connector;
    // }

   private:
    const std::string key;
    const std::string path;
    basic_connector inner_connector;
    EasyRobotCommands::multithread_stream<256> inner_stream;

    mutable std::mutex reconnect_mutex;
    std::function<void(const uint8_t*, size_t)> recv_callback;
    bool recv_flag;
    std::thread recv_thread;

    void ReceiveStart() {
        recv_thread = std::thread(&Connector::receive_loop, this);
    }
};

}  // namespace SerialConection