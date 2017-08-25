#include "mainwindow.h"
#include "ui_mainwindow.h"

using namespace std;

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui_(new Ui::MainWindow)
{
  my_ros_ = new ROSThread(this, &mutex);
  ui_->setupUi(this);
  my_ros_->start();
  connect(my_ros_, SIGNAL(StampShow(quint64)), this, SLOT(SetStamp(quint64)));

  connect(ui_->quitButton, SIGNAL(pressed()), this, SLOT(TryClose()));
  connect(ui_->pushButton, SIGNAL(pressed()), this, SLOT(FilePathSet()));
  connect(ui_->pushButton_2, SIGNAL(pressed()), this, SLOT(Play()));
  connect(ui_->pushButton_3, SIGNAL(pressed()), this, SLOT(Pause()));

  connect(ui_->doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(PlaySpeedChange(double)));
  ui_->doubleSpinBox->setRange(0.1,10.0);
  ui_->doubleSpinBox->setValue(1.0);
  ui_->doubleSpinBox->setSingleStep(0.1);
  connect(ui_->checkBox, SIGNAL(stateChanged(int)), this, SLOT (LoopFlagChange(int)));
  ui_->checkBox->setCheckState(Qt::Unchecked);

  play_flag_ = false;
  pause_flag_ = false;
  loop_flag_ = false;
}

MainWindow::~MainWindow()
{
  emit setThreadFinished(true); //Tell the thread to finish
  delete ui_;
  my_ros_->quit();
  if(!my_ros_->wait(500)) //Wait until it actually has terminated (max. 3 sec)
  {
      my_ros_->terminate(); //Thread didn't exit in time, probably deadlocked, terminate it!
      my_ros_->wait(); //We have to wait again here!
  }
}


void MainWindow::RosInit(ros::NodeHandle &n)
{
  my_ros_->ros_initialize(n);
}


void MainWindow::TryClose()
{
  close();
}


void MainWindow::FilePathSet()
{
  QFileDialog dialog;
  data_folder_path_ = dialog.getExistingDirectory();

  my_ros_->data_folder_path_ = data_folder_path_.toUtf8().constData();
  this->ui_->label->setText("Data is loading.....");

  my_ros_->Ready();

  this->ui_->label->setText(data_folder_path_);

}

void MainWindow::SetStamp(quint64 stamp)
{
  this->ui_->label_2->setText(QString::number(stamp));
}

void MainWindow::Play()
{
  if(play_flag_ == false){
    play_flag_ = true;
    my_ros_->play_flag_ = true;
    this->ui_->pushButton_2->setText(QString::fromStdString("End"));
  }else{
    play_flag_ = false;
    my_ros_->play_flag_ = false;
    this->ui_->pushButton_2->setText(QString::fromStdString("Play"));
  }
}

void MainWindow::Pause()
{
  if(pause_flag_ == false){
    pause_flag_ = true;
    my_ros_->pause_flag_ = true;
    this->ui_->pushButton_3->setText(QString::fromStdString("Resume"));
  }else{
    pause_flag_ = false;
    my_ros_->pause_flag_ = false;
    this->ui_->pushButton_3->setText(QString::fromStdString("Pause"));
  }
}


void MainWindow::PlaySpeedChange(double value)
{
  my_ros_->play_rate_ = value;
}

void MainWindow::LoopFlagChange(int value)
{
  if(value == 2){
    loop_flag_ = true;
  }else if(value == 0){
    loop_flag_ = false;
  }
}
