#include <QProcess>

class Process
{
private slots:
void start()
{
process = new QProcess(this);
    process->startDetached("E:/Program Files (x86)/netease/MailMaster/Application/mailmaster.exe",QStringList());
}
};

