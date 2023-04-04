#include "read_files.h"


void ReadFiles::displayFilesName()
  {
  std::cout << "Files' names" << std::endl;
  QString path = "";
  QDir dir(path);
  dir.setFilter(QDir::Files | QDir::Hidden | QDir::NoSymLinks);
  dir.setSorting(QDir::Size | QDir::Reversed);
   QFileInfoLIst list = dir.entryInfoList();
   for(int i = 0; i < list.size(); ++i)
   {
    QFileInfo fileInfo = list.at(i);
    std::cout << qPrintable(QString("%1 %2").arg(fileInfo.size(), 10)
                                            .arg(fileInfo.fileName()));
    std::cout << std::endl;
   }
  }
QFileInfoList ReadFiles::GetFileList(QString path)
{
  QDir dir(path);
  QFileInfoList file_list = dir.entryInfoList(QDir::Files | QDir::Hidden | QDir::NoSymLinks);
  QFileInfoList folder_list = dir.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot);

  for(int i = 0; i != folder_list.size(); i++)
  {
    QString name = folder_list.at(i).absoluteFilePath();
    QFileInfoList child_file_list = GetFileList(name);
    file_list.append(child_file_list);
  }
  return file_list;
}
