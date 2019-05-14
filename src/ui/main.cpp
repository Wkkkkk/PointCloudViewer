#include <QtWidgets/QApplication>
#include <QtCore/QTranslator>

#include "MainWindow.h"
#include "Config.h"

int main(int argc, char* argv[]) {
    qputenv("QT_STYLE_OVERRIDE",""); // suppress the qt style warning
    QApplication app(argc, argv);

    auto config_file_path = "./config/setting.yaml";
    Config::setParameterFile(config_file_path);

    qRegisterMetaType<Point>("Point");

    QTranslator translator;
    if(!app.arguments().contains("en"))
        if (translator.load(QLocale("zh"), QLatin1String("lpd"), QLatin1String("_"), app.applicationDirPath()+"/tr"))
            app.installTranslator(&translator);

    MainWindow mainwindow;
    mainwindow.setMinimumSize(800, 600);  //avoid graphic context bugs!
    mainwindow.show();

    return app.exec();
}