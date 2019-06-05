#include <QtWidgets/QApplication>
#include <QtCore/QTranslator>
#include <QtCore/QTextStream>
#include <QtCore/QMutex>
#include <QtCore/QFile>

#include "MainWindow.h"
#include "Config.h"

void MessageOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
	// 加锁
	static QMutex mutex;
	mutex.lock();

	// 输出信息至文件中（读写、追加形式）
	QFile file("log.txt");
	file.open(QIODevice::ReadWrite | QIODevice::Append);
	QTextStream stream(&file);
	stream << msg << "\r\n";
	file.flush();
	file.close();

	// 解锁
	mutex.unlock();
}

int main(int argc, char* argv[]) {
	qInstallMessageHandler(MessageOutput); // message redirect to file
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