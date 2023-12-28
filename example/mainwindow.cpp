#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  azimuth_panel= new AzimuthPanel(this);
  azimuth_panel->move(10,20);
  azimuth_panel->setCurrentValue(50);

  bar=new Bar(this);

  battery=new Battery(this);
  battery->move(10,450);

  editable_label=new EditableLabel(this);
  editable_label->move(10,550);
  editable_label->stack->resize(100,25);

//  info_label=new InfoLabel(this);
//  info_label->move(10,600);

  panel=new Panel(this);
  panel->move(500,25);

  switch_dual=new SwitcherDual(this);
  switch_dual->move(500,300);
}


MainWindow::~MainWindow()
{
  delete ui;
}
