#include "parsedataform.h"
#include "ui_parsedataform.h"

ParseDataForm::ParseDataForm(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::ParseDataForm)
{
  ui->setupUi(this);

  init();
  this->setAttribute(Qt::WA_QuitOnClose, false);
}

ParseDataForm::~ParseDataForm()
{
  delete ui;
}

bool ParseDataForm::init()
{
  initTruckParam();
  ui->time2->setText("3");

  return true;
}

bool ParseDataForm::initTruckParam()
{
  config = YAML::LoadFile("./remote/config.yaml");
  std::vector<std::string> trucks_name;
  if(config["truck_params"]["truck_name"])
  {
    trucks_name = config["truck_params"]["truck_name"].as<std::vector<std::string>>();
  }
  for(size_t i = 0; i < trucks_name.size(); i++)
  {
    bool isValid = config["truck_params"][trucks_name[i]]["ip"] && config["truck_params"][trucks_name[i]]["name"] &&
                   config["truck_params"][trucks_name[i]]["lf"] && config["truck_params"][trucks_name[i]]["lb"];
    if(!isValid)
    {
      QMessageBox::warning(this,"警告",QString("车辆参数不全"));
      continue;
    }

    TruckParam temp;
    temp.ip = config["truck_params"][trucks_name[i]]["ip"].as<std::vector<int>>();
    temp.lf = config["truck_params"][trucks_name[i]]["lf"].as<std::vector<double>>();
    temp.lb = config["truck_params"][trucks_name[i]]["lb"].as<std::vector<double>>();
    temp.name = config["truck_params"][trucks_name[i]]["name"].as<std::string>();

    if(temp.ip.size() != 4)
    {
      QMessageBox::warning(this,"警告", QString::fromStdString(temp.name) + QString("IP地址不正确"));
      continue;
    }

    if(temp.lf.size() != 6)
    {
      QMessageBox::warning(this,"警告", QString::fromStdString(temp.name) + QString("前雷达参数不正确"));
      continue;
    }

    if(temp.lf.size() != 6)
    {
      QMessageBox::warning(this,"警告", QString::fromStdString(temp.name) + QString("后雷达参数不正确"));
      continue;
    }

    ui->comboBox->addItem(QString::fromStdString(temp.name));
    truck_paramv.push_back(temp);
  }
  on_comboBox_currentIndexChanged(0);
  return true;
}



void ParseDataForm::on_comboBox_currentIndexChanged(int index)
{
  if (ui->comboBox->count() > truck_paramv.size())
  {
    return;
  }
  QString tStr;
  tStr = QString::number(truck_paramv[index].ip[0]);
  for(size_t i = 1; i < truck_paramv[index].ip.size(); i++)
  {
    tStr += "." + QString::number(truck_paramv[index].ip[i]);
  }
  ui->ip2->setText(tStr);

  tStr = QString::number(truck_paramv[index].lf[0]);
  for(size_t i = 1; i < truck_paramv[index].lf.size(); i++)
  {
    tStr += ", " + QString::number(truck_paramv[index].lf[i]);
  }
  ui->lf2->setText(tStr);

  tStr = QString::number(truck_paramv[index].lb[0]);
  for(size_t i = 1; i < truck_paramv[index].lb.size(); i++)
  {
    tStr += ", " + QString::number(truck_paramv[index].lb[i]);
  }
  ui->lb2->setText(tStr);
  return;
}

//bool ParseDataForm::updateIndex(int index)
//{
//}

void ParseDataForm::on_saveButton_clicked()
{
  int index = ui->comboBox->currentIndex();
/* Store edited configuation.
  truck_paramv[index].ip = ui->ip2;
  truck_paramv[index].lf = ui->lf2;
  truck_paramv[index].lb = ui->lb2;
  */

/* Save current configuation into file named "save.yaml".
  std::ofstream fout("save.yaml");
  fout << config;
  fout.close();
*/
}
