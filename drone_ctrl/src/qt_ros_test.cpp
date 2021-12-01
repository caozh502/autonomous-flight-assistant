#include "qt_ros_test.h"
#include "drone_ctrl/ui_qt_ros_test.h"
#include <QString>
#include <QtGui>
#include <QMessageBox>
#include <std_msgs/String.h>
#include "std_msgs/Float32MultiArray.h"
#include <sensor_msgs/NavSatFix.h>

Qt_Ros_Test::Qt_Ros_Test(int argc, char** argv,QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Qt_Ros_Test),
    qnode(argc,argv)
{
    ui->setupUi(this);

    qnode.init();
    QObject::connect(ui->actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ui->lineEdit->setText("8.4075");
    ui->lineEdit_2->setText("49.014");
    realTime = new QTimer(this);
    connect(realTime,SIGNAL(timeout()),this,SLOT(getData()));
    realTime->setTimerType(Qt::PreciseTimer);
    realTime->start(100);
//    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    ros::init(argc, argv, "gps_info");
    if (!ros::master::check())
    {
        ROS_INFO("No master started!");
        this->close();
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    chatter_publisher = n.advertise<std_msgs::Float32MultiArray>("target_point_gps", 1);

    if( ui->checkBox->isChecked() )
    {
        on_pushButton_clicked(true);
    }

//    realTime = new QTimer(this);
//    connect(realTime,SIGNAL(timeout()),this,SLOT(getData()));
//    realTime->setTimerType(Qt::PreciseTimer);
//    realTime->start(1000);

}

Qt_Ros_Test::~Qt_Ros_Test()
{
    delete ui;
}

//void Qt_Ros_Test::on_slider_value_change(int value)
//{
//    ui->lineEdit->setText(QString::number(value));

//    std_msgs::String msg;
//    std::stringstream ss;
//    ss << "data: " << ui->verticalSlider->value();
//    msg.data = ss.str();
//    chatter_publisher.publish(msg);

//}


void Qt_Ros_Test::on_pushButton_clicked(bool checked)
{
//  if ( ui->checkBox->isChecked()){
    QString longitude_ = ui->lineEdit->text();
    QString latitude_ = ui->lineEdit_2->text();
    longitude = longitude_.toFloat();
    latitude = latitude_.toFloat();

    std_msgs::Float32MultiArray msg;
    msg.data.push_back(longitude);
    msg.data.push_back(latitude);
//    flight_state=3; // assistant active
//    msg.data.push_back(flight_state);
    chatter_publisher.publish(msg);
    std::cout<<"successful push"<<std::endl;


//  }
}
void Qt_Ros_Test::getData(){
    qsrand(QTime(0,0,0).secsTo(QTime::currentTime()));
    QString lo = QString::number(qnode.current_lo);
    QString la = QString::number(qnode.current_la);
    QString dis = QString::number(qnode.distance);
    QString ang = QString::number(qnode.angle);
    QString state = QString::fromStdString(qnode.flight_state);
    QString speed = QString::number(qnode.speed);
    QString minDis = QString::number(qnode.minDis);
    QString assistant = QString::number(qnode.control_state);
    ui->label_3->setText(lo);
    ui->label_5->setText(la);
    ui->label_9->setText(dis);
    ui->label_11->setText(ang);
    ui->label_14->setText(state);
    ui->label_16->setText(speed);
    ui->label_18->setText(minDis);
    ui->label_20->setText(assistant);
}

void Qt_Ros_Test::on_pushButton_2_clicked()
{
    QString longitude_ = ui->lineEdit->text();
    QString latitude_ = ui->lineEdit_2->text();
    longitude = longitude_.toFloat();
    latitude = latitude_.toFloat();

    std_msgs::Float32MultiArray msg;
    msg.data.push_back(longitude);
    msg.data.push_back(latitude);
    flight_state=0; // landing
    msg.data.push_back(flight_state);
    chatter_publisher.publish(msg);
}
