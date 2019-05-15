/*
 * Copyright (c) 2018 Ally of Intelligence Technology Co., Ltd. All rights reserved.
 *
 * Created by WuKun on 12/17/18.
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

#ifndef PROTYPE_FILECONVERT_H
#define PROTYPE_FILECONVERT_H

#include <QtCore/QThread>

class FileConvertThread : public QThread {
Q_OBJECT
public:
    explicit FileConvertThread(const QString &file_dir_path);

    void run() override;

private:
    QString file_dir_path_;
signals:

    void progress_value(int);
};

#endif //PROTYPE_FILECONVERT_H