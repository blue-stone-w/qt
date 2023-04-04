#include <QDir>
#include <QFileInfoList>

class ReadFiles
{
public:
  ReadFiles(){}
  ~ReadFiles(){}
  void displayFilesName();
  QFileInfoList GetFileList(QString path);
  
};
