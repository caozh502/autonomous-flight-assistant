#ifndef KEYBOARD_RC_GUI_H
#define KEYBOARD_RC_GUI_H

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/UInt8.h>

#include <qt5/QtWidgets/QDialog>
#include <qt5/QtWidgets/QtWidgets>

const float MODE_P = 10000.0;  // Value of MODE_P normally transmitted by the remote
const float mode_a = 0.0;  // Value of mode_a normally transmitted by the remote
const float mode_f = -10000.0;  // Value of mode_f normally transmitted by the remote

namespace Ui {
    class KeyboardRcGUI;
}

class KeyboardRcGUI : public QDialog
{
    Q_OBJECT

public:
    explicit KeyboardRcGUI(QWidget *parent = 0);
    ~KeyboardRcGUI();

    ros::Publisher pub_rc;
    float mode_state{0}; // start with "A"

    void setPublisher(const ros::Publisher& pub);
    void publish_rc(float right_stick_right, float right_stick_up, float left_stick_right, float left_stick_up, float mode_switch) const;

    void keyPressEvent(QKeyEvent *event);
private:
    Ui::KeyboardRcGUI *ui;

};

#endif // KEYBOARD_RC_GUI_H
