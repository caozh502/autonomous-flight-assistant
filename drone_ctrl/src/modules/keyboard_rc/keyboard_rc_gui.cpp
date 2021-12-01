#include "keyboard_rc_gui.h"
#include <QtWidgets>
#include "ui_keyboard_rc_gui.h" //this will be autogenerated during compilation, normally not present



KeyboardRcGUI::KeyboardRcGUI(QWidget *parent) :
        QDialog(parent),
        ui(new Ui::KeyboardRcGUI) //this will be autogenerated during compilation, normally not present

{
    ui->setupUi(this);
}

KeyboardRcGUI::~KeyboardRcGUI()
{
    delete ui;
}

void KeyboardRcGUI::keyPressEvent(QKeyEvent *event){

    char key = event->key();
    std::cout << "key:" << key << std::endl;
    switch(key){

        case 'W': //going up
            publish_rc(0,0,0,1,this->mode_state);
            break;

        case 'S': //going down
            publish_rc(0,0,0,-1,this->mode_state);
            break;

        case 'A': //turn left
            publish_rc(0,0,-1,0,this->mode_state);
            break;

        case 'D': //turn right
            publish_rc(0,0,1,0,this->mode_state);
            break;

        case 'I': // forward
            publish_rc(0,1,0,0,this->mode_state);
            break;

        case 'K': //backward
            publish_rc(0,-1,0,0,this->mode_state);
            break;

        case 'J': // strafe left
            publish_rc(-1,0,0,0,this->mode_state);
            break;

        case 'L': // strafe right
            publish_rc(1,0,0,0,this->mode_state);
            break;

        case '1': // set to F
            this->mode_state = mode_f;
            publish_rc(0,0,0,0,this->mode_state);
            break;

        case '2': // set to A
            this->mode_state = mode_a;
            publish_rc(0,0,0,0,this->mode_state);
            break;

        case '3': // set to P
            this->mode_state = MODE_P;
            publish_rc(0,0,0,0,this->mode_state);
            break;

        case 'T': //takeoff
            publish_rc(-1,-1,1,-1,this->mode_state);
            break;

        default:
            break;
    }
    event->accept();
}


void KeyboardRcGUI::setPublisher(const ros::Publisher& pub) {
    this->pub_rc = pub;
}

void KeyboardRcGUI::publish_rc(float right_stick_right, float right_stick_up, float left_stick_right, float left_stick_up, float mode_switch) const {
    sensor_msgs::Joy joy_msg;
    joy_msg.axes.push_back(right_stick_right); // Roll Channel
    joy_msg.axes.push_back(right_stick_up); // Pitch Channel
    joy_msg.axes.push_back(left_stick_right); // Yaw Channel
    joy_msg.axes.push_back(left_stick_up); // Throttle Channel
    joy_msg.axes.push_back(mode_switch); // Mode switch
    joy_msg.axes.push_back(-5000); // Landing Gear Switch (not implemented)
    this->pub_rc.publish(joy_msg);
}