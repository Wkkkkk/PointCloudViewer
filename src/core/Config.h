/*
 * Copyright (c) 2018 Ally of Intelligence Technology Co., Ltd. All rights reserved.
 *
 * Created by WuKun on 5/14/19.
 * Contact with:wk707060335@gmail.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http: *www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/

#ifndef PROTYPE_CONFIG_H
#define PROTYPE_CONFIG_H

#include <iostream>
#include <memory>
#include <yaml-cpp/yaml.h>

class Config {
private:
    YAML::Node node_;
    std::string filepath_;

    Config() = default;
public:
    static Config &getInstance() {
        static Config instance;
        return instance;
    }

    static void setParameterFile(const std::string &filename) {
        auto &config = Config::getInstance();
        config.node_ = YAML::LoadFile(filename);
        config.filepath_ = filename;
        if (config.node_.IsNull()) {
            std::cerr << "parameter file " << filename << " does not exist." << std::endl;
            return;
        }
    }

    template<typename T>
    static T get(const std::string &key) {
        auto &config = Config::getInstance();
        auto result = config.node_[key];
        if (result.IsDefined()) {
            auto value = result.as<T>();
            std::cout << key << " is set to: " << value << std::endl;
            return value;
        } else {
            std::cerr << key << " is lost." << std::endl;
            return T();
        }
    }

    template<typename T>
    static T get(const std::string &key, const T defaultVal) {
        auto &config = Config::getInstance();
        auto result = config.node_[key];
        if (result.IsDefined()) {
            auto value = result.as<T>();
            return value;
        } else {
            return defaultVal;
        }
    }

    template<typename T>
    static std::vector<T> getlist(const std::string &key) {
        auto &config = Config::getInstance();
        auto node = config.node_[key];
        static std::vector<T> collect;
        if (node.IsDefined()) {
            for (auto t : node) {
                collect.push_back(t.as<T>());
            }
        }
        return collect;
    }

    static std::string getConfigFilePath() {
        auto &config = Config::getInstance();
        return config.filepath_;
    }

};


#endif //PROTYPE_CONFIG_H
