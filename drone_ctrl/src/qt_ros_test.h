#ifndef QT_ROS_TEST_H
#define QT_ROS_TEST_H

#include <QWidget>
#include <ros/ros.h>
#include  "qnode.h"
#include <QTime>
#include <QTimer>

namespace Ui {
class Qt_Ros_Test;
}

class Qt_Ros_Test : public QWidget
{
    Q_OBJECT

public:
    explicit Qt_Ros_Test(int argc, char** argv,QWidget *parent = 0);
    ~Qt_Ros_Test();
  float longitude;
  float latitude;
  int flight_state = 0;
  void gpsCallback();
private slots:
//    void on_slider_value_change(int value);

    void on_pushButton_clicked(bool checked);
    void getData();

    void on_pushButton_2_clicked();

private:
    Ui::Qt_Ros_Test *ui;
    ros::Publisher chatter_publisher;

    QNode qnode;

    QTimer *realTime;
};

#endif // QT_ROS_TEST_H
