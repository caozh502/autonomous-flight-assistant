#include "../include/DialogKeyboard.h"
#include <QtWidgets>
#include "ui_DialogKeyboard.h"

const int s = 1;  // speedMultiplier, set as desired
const int m600_rc_v_horizontal = 10 ; // this is what is equivalent to what the RC sends on max
const int m600_rc_v_vertical = 5;
const int m600_rc_yawrate = 3;


DialogKeyboard::DialogKeyboard(QWidget *parent) :
        QDialog(parent),
        ui(new Ui::DialogKeyboard)
{
    ui->setupUi(this);
}

DialogKeyboard::~DialogKeyboard()
{
    delete ui;
}

void DialogKeyboard::keyPressEvent(QKeyEvent *event){
    if(!drone)
        return ;
    char key = event->key();
    std::cout << "key:" << key << std::endl;
    switch(key){
        case 'Z':
            //take off
            std::cout << "take off !" << std::endl;
            drone->monitoredTakeoff();
            drone->takeOff();
            break;
        case 'X':
            //land
            drone->land();
            break;
        case 'H':
            //normally automatically activated (p mode)
            drone->hover();
            break;

        case 'W':
            //going up
            drone->rise(s * m600_rc_v_vertical);
            break;

        case 'S':
            //going down
            drone->rise(-s * m600_rc_v_vertical);
            break;

        case 'A':
            //turn left
            drone->yaw(s * m600_rc_yawrate);
            break;

        case 'D':
            //turn right
            drone->yaw(-s * m600_rc_yawrate);
            break;


        case 'I':
            // forward
            drone->pitch(s*m600_rc_v_horizontal);
            break;

        case 'K':
            //backward
            drone->pitch(-s*m600_rc_v_horizontal);
            break;

        case 'J':
            // strafe left
            drone->roll(s*m600_rc_v_horizontal);
            break;

        case 'L':
            // strafe right
            drone->roll(-s*m600_rc_v_horizontal);
            break;

        default:
            //break;
            if(!drone->isPosctrl)
                drone->hover();
    }
    event->accept();
}

void DialogKeyboard::keyReleaseEvent(QKeyEvent *event){
    if(!drone)
        return ;
    char key = event->key();
    if (!event->isAutoRepeat()){
        std::cout << "key:" << key << " has been released !" << std::endl;
        if( !drone->isPosctrl)
            drone->hover();
        event->accept();
    }else{
        event->ignore();
    }
}