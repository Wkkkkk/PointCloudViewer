#include <QtCore/QThread>


class FileConvertThread : public QThread {
Q_OBJECT
public:
    FileConvertThread(QString file_dir_path);

    void run() override;

private:
    QString file_dir_path_;
signals:

    void progress_value(int);
};