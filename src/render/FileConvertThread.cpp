#include <iostream>
#include <functional>

#include "FileConvertThread.h"
#include "PointcloudConvert.h"

FileConvertThread::FileConvertThread(QString file_dir_path) :
        file_dir_path_(file_dir_path) {

}

void FileConvertThread::run() {
    emit progress_value(0);

    //sleep(5);
    auto dir_path_str = file_dir_path_.toStdString();
    std::cout << "get bag dir: " << dir_path_str << std::endl;

    ConvertToLas(dir_path_str, []() {});

    emit progress_value(100);
}