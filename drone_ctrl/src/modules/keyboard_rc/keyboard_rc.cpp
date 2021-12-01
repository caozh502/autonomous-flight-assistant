#include "keyboard_rc_gui.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "keyboard_rc_gui");
    QApplication app(argc, argv);

    ros::NodeHandle node;
    KeyboardRcGUI dlg_keyboard;
    ros::Publisher pub_rc;
    pub_rc = node.advertise < sensor_msgs::Joy > ("/dji_sdk/rc", 1024);

    dlg_keyboard.setPublisher(pub_rc);
    dlg_keyboard.show();

    return QApplication::exec();
}
