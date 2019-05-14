/*
 * Copyright (c) 2018 Ally of Intelligence Technology Co., Ltd. All rights reserved.
 *
 * Created by WuKun on 2/15/19.
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

#ifndef SPACECLOUD_SINGLETON_H
#define SPACECLOUD_SINGLETON_H

#include <map>
#include <mutex>
#include <memory>
#include <thread>
#include <vector>
#include <functional>
#include <algorithm>
#include <iostream>

namespace core {

/**
 * @brief A simple Singleton Implement
 * */
template<class T>
class Singleton {
public:
    Singleton(const Singleton &) = delete;

    Singleton &operator=(const Singleton &) = delete;

    static Singleton &getInstance() {
        static Singleton<T> instance;
        return instance;
    }

    void update(T obj) {
        std::swap(value, obj);
    }

    T getValue() {
        return value;
    }

private:
    Singleton() = default;

    T value;
};

}
#endif //SPACECLOUD_SINGLETON_H
