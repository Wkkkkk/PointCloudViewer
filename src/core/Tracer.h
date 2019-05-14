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

#ifndef PROTYPE_TRACER_H
#define PROTYPE_TRACER_H

#include <iostream>

#define TRACER_DEBUG
#ifdef  TRACER_DEBUG
#define TRACER \
   Tracer tracer(__func__)
#else
#define TRACER
#endif

class Tracer {
public:
    explicit Tracer(const char msg[]) :
            m_msg(msg) {
        std::cout << ">>>Enter: " << m_msg << std::endl;
    }

    ~Tracer() {
        std::cout << "<<<Leave: " << m_msg << std::endl;
    }

private:
    const char *m_msg;
};

#endif //PROTYPE_TRACER_H
