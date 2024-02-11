#pragma once

#include <string>

#include "easy_robot_commands/shared_member/concepts.hpp"
#include "exec_cmd.hpp"
#include "serial_connection/basic_connector.hpp"
namespace SerialConection {


class Connector {
   public:
    Connector(const std::string& modalias, const std::string& bash_absolute_path) : key(modalias),
                                                                                    path(bash_absolute_path),
                                                                                    inner_connector() {}
    ~Connector() {}

    void Connect() {
        reconnect();
    }
    void Transmite(const uint8_t* data, size_t len) {
        std::cout << "Connector Transmite " << len << "bytes" << std::endl;
        if (!inner_connector.write(data, len))
            reconnect();
    }

    void Receive() {
    }

    void blocked_consume(const uint8_t* data, size_t len) {
        Transmite(data, len);
    }

    template <EasyRobotCommands::can_trigger T>
    Connector& operator<<(T& trigger) {
        auto lambda = [this](T& trigger_) {
            // stream container >> Connector Transmite
            trigger_ >> (*this);
        };
        EasyRobotCommands::trigger_operation<T> ope(lambda);
        trigger.register_trigger_operation(ope);
        return (*this);
    }

   private:
    const std::string key;
    const std::string path;
    basic_connector inner_connector;

    void reconnect() {
        static int cnt;
        cnt++;
        // 防止不断 reconnect
        if (cnt > 20 && (cnt % 20)) {
            return;
        }
        if (cnt > 100 && (cnt % 101)) {
            return;
        }
        inner_connector.close();
        std::string output;
        try {
            output = exec((". " + path).c_str());
        } catch (const std::exception& e) {
            std::cerr << "Exception: " << e.what() << std::endl;
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
        if (keys.size() != values.size()) return;

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
    }
};

}  // namespace SerialConection