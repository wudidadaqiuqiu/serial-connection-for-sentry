#pragma once

#include <functional>
#include <string>
#include <thread>
#include <condition_variable>

#include "container/multithread_stream.hpp"
#include "easy_robot_commands/shared_member/concepts.hpp"
#include "exec_cmd.hpp"
#include "serial_connection/basic_connector.hpp"
namespace SerialConection {
class Connector {
   public:
   using receive_stream = EasyRobotCommands::multithread_stream<256>;
    Connector(const std::string& modalias,
              const std::string& bash_absolute_path,
              const std::function<void(const uint8_t*, size_t)>& recv_notify = nullptr)
        : key(modalias),
          path(bash_absolute_path),
          inner_connector(),
          data_in_stream(0),
          recv_callback(recv_notify),
          recv_flag(true) { }
    ~Connector() {
        inner_connector.close_prepare();
        recv_flag = false;
        process_thread_end = true;
        working_condition.notify_all();
        condition.notify_all();
        recv_thread.join();
        process_thread.join();
    }

    bool is_working() {
        return inner_connector.is_working();
    }
    void Connect() {
        reconnect();
        ReceiveStart();
    }

    bool ReConnect() {
        reconnect();
        working_condition.notify_all();
        return is_working();
    }
    void Transmite(const uint8_t* data, size_t len) {
        // std::cout << "Connector Transmite " << len << "bytes" << std::endl;
        if (!inner_connector.write(data, len))
            reconnect();
    }

    void receive_loop() {
        while (recv_flag) {
            if (inner_connector.is_working() == false){
                std::cout << "inner connector not working" << std::endl;
                std::unique_lock<std::mutex> l(working_mutex);
                working_condition.wait(l, [&]{return (this->inner_connector.is_working() || this->process_thread_end);});
                if (process_thread_end) {
                    std::cout << "receive loop break" << std::endl;
                    break;
                }
            }
            int read_len = inner_connector.read();
            if (read_len <= 0) {
                std::cout << "read len <= 0, read_len:" << read_len << std::endl;
                inner_connector.set_to_not_working();
                // reconnect();
            } else {
                if (inner_stream.add_with_caution(inner_connector.get(), read_len) == receive_stream::stream_normal){
                    {std::lock_guard<std::mutex> l(pc_mutex);
                        data_in_stream++;
                    }
                    // std::cout << "notify" << std::endl;
                    condition.notify_one();
                } else {
                    std::cerr << "connector inner stream overrun or (read_len == 0), data in stream is " << data_in_stream << std::endl;
                }
                // std::cout << "read success!!" << std::endl;
                // if (recv_callback != nullptr)
                    // recv_callback(inner_connector.get(), read_len);
            }
        }
        std::cout << "receive loop end" << std::endl;
    }
    
    void receive_stream_consume_loop() {
        while (recv_flag) {
            {std::unique_lock<std::mutex> l(pc_mutex);
                condition.wait(l, [&]{return (this->data_in_stream > 0 || (this->process_thread_end));});
                if (process_thread_end) {
                    std::cout << "receive_stream_consume_loop break" << std::endl; 
                    break;
                }
                --data_in_stream;
            }
            // std::cout << "data in stream triggered"<< std::endl;
            if (recv_callback == nullptr) {
                inner_stream.clear();
                continue;
            }
            while (inner_stream.consume_with_best_effort(recv_callback, true) != receive_stream::stream_empty) {
                /* code */
                // static uint32_t ii;
                // std::cout << "consume once " << ii++ << std::endl;
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
        // static int cnt;
        // cnt++;
        // 防止不断 reconnect
        // if (cnt > 20 && (cnt % 20)) {
        //     reconnect_mutex.unlock();
        //     return;
        // }
        // if (cnt > 100 && (cnt % 101)) {
        //     reconnect_mutex.unlock();
        //     return;
        // }
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
    receive_stream inner_stream;
    std::condition_variable condition;
    std::condition_variable working_condition;
    std::mutex working_mutex;
    bool process_thread_end;
    std::mutex pc_mutex;
    int data_in_stream;
    mutable std::mutex reconnect_mutex;
    std::function<void(const uint8_t*, size_t)> recv_callback;
    bool recv_flag;
    std::thread recv_thread;
    std::thread process_thread;
    void ReceiveStart() {
        recv_thread = std::thread(&Connector::receive_loop, this);
        process_thread_end = false;
        process_thread = std::thread(&Connector::receive_stream_consume_loop, this);
    }
};

}  // namespace SerialConection