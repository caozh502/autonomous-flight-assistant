//
// Created by caleb on 17.05.21.
//

#include "direction_decision.h"

#define C_EARTH (float)6378137.0
#define C_PI (float)3.141592653589793

// TODO: cloudClusters in message rewrite (reference drone_object_ros.cpp)
// global variables for subscribed topics
std::vector<PtCdtr> cloudClusters;
pcl::visualization::PCLVisualizer::Ptr viewer;

direction_decision::direction_decision(DroneObjectROS &drone,int timeRate_in)
    : nh(), my_drone(drone),assistant_active(false)
{
    flightStatePub = nh.advertise<std_msgs::String>("flight_state", 1);
    attiSub = nh.subscribe("dji_sdk/attitude", 1, &direction_decision::attitude_callback,this);
    tarGpsSub = nh.subscribe("target_point_gps", 1, &direction_decision::target_gpsCallback,this);
    disAngPub = nh.advertise<std_msgs::Float32MultiArray>("distance_angle2ro", 1);
    gpsSub = nh.subscribe("dji_sdk/gps_position", 1, &direction_decision::current_gpsCallback,this);
    directPub = nh.advertise<std_msgs::Float32MultiArray>("direct_info", 1);
    rcSub = nh.subscribe("/dji_sdk/rc", 1, &direction_decision::receive_rc_info, this);
    
    rc_info={NAN,NAN,NAN,NAN,NAN};
    //initialize
    timeRate = timeRate_in;

    ///droneBox for collision detection
    Eigen::Vector3f uav_point(-0.85, -0.3, -0.8);
    float length =  10 - 2 * uav_point(2); // detection box's length
    float width = -2 * uav_point(0);
    float height = -2 * uav_point(1);

    droneBox = createBox(uav_point, width, length, height, cameraPosition);

    ///droneBox_true for calculating the distance to obstacle

    float length_t = -2 * uav_point(2);
    droneBox_t = createBox(uav_point, width, length_t, height,cameraPosition);

    d_preBegin = length + 2* uav_point(2);
    d_preEnd = ((d_safe + length_t/2.+cameraPosition[2]) * sqrt(3)) * sqrt(3) - (length_t/2.+cameraPosition[2]);

}


direction_decision::~direction_decision(){}

void direction_decision::motionStep() {

    std_msgs::Float32MultiArray pubMsg;
    std_msgs::Float32MultiArray directMsg;
    int clusterId = 0;
    int nearstId = 0;
    int fs_state = 0;
    float minDistance = 0;
    int clusterSize = (!cloudClusters.empty())? cloudClusters.size(): 0;

    UAVBoxVisualization(viewer, droneBox, colors[1], clusterId);

    std_msgs::String state;
    std::stringstream ss;

    ////get Angle between drone heading vector and drone2Target vector
    setDronePos(current_gps.longitude,current_gps.latitude,current_gps.altitude,
                toEulerAngle(current_atti).z);
    float angle2ro=getAimAngle();//angle2ro: angle for drone to aim to target,
    ////calculate included angle for GPS navigation
    float a = DEG2RAD(drone_y) - DEG2RAD(target_y);
    float b = DEG2RAD(drone_x) - DEG2RAD(target_x);
    float s = 2 * asin(sqrt(pow(sin(a/2), 2) + cos(DEG2RAD(drone_y))*cos(DEG2RAD(target_y))*pow(sin(b/2),2)));
    float dis2tar = s * C_EARTH;

    ///publish the information of distance and included angle
    pubMsg.data.push_back(dis2tar);
    pubMsg.data.push_back(angle2ro*180/C_PI);
    
    bool want_to_be_active = (this->rc_info[4]==MODE_P); //if rc is in Mode P, assistant wants to be active

    if(assistant_active && want_to_be_active) { //if it is active and still wants to be active, it adjusts the distances
        ////assistant airpilot
        /// if drone is close to target, then hovering...
        if (dis2tar<2){

          ss<<"REACH target point, wating for next command..."<<endl;
          drone_stop(1);
        //takeoff_state = 0;
        //my_drone.land();
          state.data = ss.str();
          flightStatePub.publish(state);
          pubMsg.data.push_back(speed);
          pubMsg.data.push_back(minDistance);
          pubMsg.data.push_back(assistant_active);
          disAngPub.publish(pubMsg);
          return;
        }
        /// turn to target
//        if (turn_state == 0){ // after avoidance or at beginning
//        if (fabs(angle2ro)>DEG2RAD(5) && fabs(angle2ro)<DEG2RAD(30)) {
//            collision_detection CD_Nav(droneBox);
////            CD_Nav.nav_detector(-angle2ro);
//            CD_Nav.nav_detector(-DEG2RAD(15)*angle2ro/fabs(angle2ro));
//            speed_ro = 0.5*v_min*angle2ro/fabs(angle2ro);
//            if (!CD_Nav.isCollision) {
//                ss<< "turn to target slowly... "<<endl;
//                my_drone.move(0,0,0,speed_ro);
                
//                state.data = ss.str();
//                flightStatePub.publish(state);
//                pubMsg.data.push_back(speed);
//                pubMsg.data.push_back(minDistance);
//                pubMsg.data.push_back(assistant_active);
//                disAngPub.publish(pubMsg);
//                return;
//            }
//            }else if (fabs(angle2ro) > DEG2RAD(30)) {
////                speed_ro = 0.3 * angle2ro;
//                collision_detection CD_Nav(droneBox);
//                CD_Nav.nav_detector(-DEG2RAD(30)*angle2ro/fabs(angle2ro));
//                speed_ro = 0.5*v_max*angle2ro/fabs(angle2ro);
//                if (!CD_Nav.isCollision) {
//                    ss<< "turn to target faster... "<<endl;
//                    my_drone.move(0,0,0,speed_ro);

//                    state.data = ss.str();
//                    flightStatePub.publish(state);
//                    pubMsg.data.push_back(speed);
//                    pubMsg.data.push_back(minDistance);
//                    pubMsg.data.push_back(assistant_active);
//                    disAngPub.publish(pubMsg);
//                    return;
//                }
//            }else if (fabs(angle2ro) < DEG2RAD(5) && !rotate_counter==2){
//                collision_detection CD_Nav(droneBox);
//                CD_Nav.nav_detector(-angle2ro);
//                speed_ro = speed_emer*angle2ro/fabs(angle2ro);
//                if (!CD_Nav.isCollision) {
//                    my_drone.move(0,0,0,speed_ro);
//                    speed = v_min;
//                    rotate_counter ++;

//                    state.data = ss.str();
//                    flightStatePub.publish(state);
//                    pubMsg.data.push_back(speed);
//                    pubMsg.data.push_back(minDistance);
//                    pubMsg.data.push_back(assistant_active);
//                    disAngPub.publish(pubMsg);
//                    return;
//                }
//            }
//        }

    collision_detection CD_Rough(droneBox);
    float DistanceList[clusterSize];
    float objCenterList[clusterSize];
    for (const PtCdtr &cluster : cloudClusters) {
    /// 4th: Find bounding boxes for each obstacle cluster
    viewer->removeShape("box" + std::to_string(clusterId));
    Box box = BoundingBox(cluster);
    renderBox(viewer, box, clusterId);

    /// 5th: Collision Detection ---> find nearst object
    /// Rough detection
    CD_Rough.update_box(box);
    CD_Rough.update_droneBox(droneBox);
    CD_Rough.rough_detector();
    if (CD_Rough.isCollision){
        viewer->removeShape("uav" + std::to_string(clusterId-1));
        CollisionVisualization(viewer, droneBox, colors[0], clusterId);

        obs_state = 1;
        turn_state = 1;
        rotate_counter = 0;
        
        CD_Rough.update_droneBox(droneBox_t);
        CD_Rough.rough_detector();
//            float minDistance_now = box.z_min - droneBox_t.z_max;

        DistanceList[clusterId] = CD_Rough.min_distance;
    } else {
       DistanceList[clusterId] = 100.0;
    }
    float center_tmp;
    center_tmp = (box.x_max - box.x_min)/2.0;
    objCenterList[clusterId] = center_tmp;
    clusterId++;
    }
    if (obs_state ==1){
    noObs_counter = 0;
    nearstId = std::min_element(DistanceList, DistanceList+sizeof(DistanceList)/sizeof(DistanceList[0]))-DistanceList;
    minDistance = DistanceList[nearstId];
    // reset vote
    float minDis_delta = minDistance-minDistance_pre;
    if (minDis_delta>=d_safe){//new obstacle appears, reset some value
        fs_counter = 0;
        memset(vote, 0, sizeof(vote));
    }
    minDistance_pre = minDistance;
    Box box_nearst = BoundingBox(cloudClusters[nearstId]);

    // pre-detector
    collision_detection CD_1(box_nearst,droneBox_t);
    collision_detection CD_1_1(box_nearst,droneBox_t);
    collision_detection CD_1_2(box_nearst,droneBox_t);
    collision_detection CD_1_3(box_nearst,droneBox_t);
    // post- and mid-detector
    collision_detection CD_2(box_nearst,droneBox_t);
    collision_detection CD_3(box_nearst,droneBox_t);

    ss << "======= Obstacle ahead ======="<<endl;

    //statistic clusters distribution, predict fs_state
    int n = 0;
    int fs_pre = 2;
    for (int i = 0; i < clusterSize;i++){
        if (objCenterList[i]<=0)n++;
    }
    if ((2*n-clusterSize) >=5){
        fs_pre = 1; //right
        direct_state = 1;
    } else if ((clusterSize-2*n)>=5){
        fs_pre = 0; //left
        direct_state = 1;
    }

    if (minDistance>d_preEnd && minDistance<d_preBegin){
        /// pre detection
        /// 0/4 direction detector
        float area_lr[2];
        area_lr[0] = (box_nearst.x_min-x_min) * (box_nearst.y_max-box_nearst.y_min); // region 4
        area_lr[1] = (x_max-box_nearst.x_max) * (box_nearst.y_max-box_nearst.y_min); // region 0
        fs_state = std::max_element(area_lr,area_lr+sizeof(area_lr)/sizeof(area_lr[0]))-area_lr;
        if (direct_state ==1 && fs_state != fs_pre){
            fs_state = fs_pre;
        }


        CD_1.lr_preDetector(fs_state,d_safe,minDistance);
        if (!CD_1.isCollision){
                info.direct = CD_1.direct;
                info.angleH = CD_1.getFlightAngle_H();
                info.angleV = 0;
                info.detecLength = CD_1.getFlightDistance();
                vote_state = 1;

        } else {
            fs_state = (fs_state==0)? 1:0;
            CD_1_1.lr_preDetector(fs_state,d_safe,minDistance);
            if (!CD_1_1.isCollision){
                    info.direct = CD_1_1.direct;
                    info.angleH = CD_1_1.getFlightAngle_H();
                    info.angleV = 0;
                    info.detecLength = CD_1_1.getFlightDistance();
                    vote_state = 1;
                    fs_counter--;
            } else {

                /// 2/6 direction detector
                float area_ud[2];
                area_ud[0] = (box_nearst.x_max-box_nearst.x_min) * (box_nearst.y_min-y_min); // region 2
                area_ud[1] = (box_nearst.x_max-box_nearst.x_min) * (y_max-box_nearst.y_max); // region 6
                fs_state = std::max_element(area_ud,area_ud+sizeof(area_ud)/sizeof(area_ud[0]))-area_ud;
                CD_1_2.ud_preDetector(fs_state,d_safe,minDistance);
                if (!CD_1_2.isCollision){
                        info.direct = CD_1_2.direct;
                        info.angleH = 0;
                        info.angleV = CD_1_2.getFlightAngle_V();
                        info.detecLength = CD_1_2.getFlightDistance();
                        vote_state = 1;
                        fs_counter--;
                } else {
                    fs_state = (fs_state==0)? 1:0;
                    CD_1_3.ud_preDetector(fs_state,d_safe,minDistance);
                    if (!CD_1_3.isCollision){
                            info.direct = CD_1_3.direct;
                            info.angleH = 0;
                            info.angleV = CD_1_3.getFlightAngle_V();
                            info.detecLength = CD_1_3.getFlightDistance();
                            vote_state = 1;
                            fs_counter--;
                    } else{
                        fs_state = ((2*n-clusterSize) >-3)? 1 : 0;
                        ss<< "No Freespace in FOV"<<endl;
                        flightStatePub.publish(state);
                        fs_counter++;
                        if(fs_counter>noFS_thres){
                            ss<<">>>> Searching new path"<<endl;
                            (fs_state==0)? my_drone.move(0,0,0,speed_emer):my_drone.move(0,0,0,-speed_emer);
                            memset(vote, 0, sizeof(vote));
                        }
                    }
                }
            }
        }
        if (vote_state == 1){
            switch (info.direct){
            case 0:
                vote[r60]++;
                break;
            case 4:
                vote[l60]++;
                break;
            case 2:
                vote[u60]++;
                break;
            case 6:
                vote[d60]++;
                break;
            }
            std::string pre_str;
            switch (info.direct){
            case 0:
            {
                pre_str = "right " +std::to_string(info.angleH);
                break;
            }
            case 4:
            {
                pre_str = "left " +std::to_string(-info.angleH);
                break;
            }
            case 2:
            {
                pre_str = "up " +std::to_string(info.angleV);
                break;
            }
            case 6:
            {
                pre_str = "down " +std::to_string(-info.angleV);
                break;
            }
            }
            ss<< "Predetector: go "+pre_str<<endl;
        }
        if (fs_counter<0){
            if (speed<v_max) {
                speed = speed + 0.1*v_min;
                ss << "boosting..."<<endl;
            }
            my_drone.move(speed,0,0,0);
        }

        } else if (minDistance<d_preEnd && minDistance>(d_safe+1.0)) {
        /// post Detector
            float area_lr[2];
            area_lr[0] = (box_nearst.x_min-x_min) * (box_nearst.y_max-box_nearst.y_min); // region 4
            area_lr[1] = (x_max-box_nearst.x_max) * (box_nearst.y_max-box_nearst.y_min); // region 0
            fs_state = std::max_element(area_lr,area_lr+sizeof(area_lr)/sizeof(area_lr[0]))-area_lr;
            if (direct_state ==1 && fs_state != fs_pre){
                fs_state = fs_pre;
            }
            CD_1.lr_postDetector(fs_state,d_safe,minDistance);
            if (!CD_1.isCollision){
                CD_1.midDetector(d_safe,d_safe+1,minDistance);
                if (!CD_1.isCollision)
                {
                    info.direct = CD_1.direct;
                    info.angleH = CD_1.getFlightAngle_H();
                    info.angleV = 0;
                    info.detecLength = CD_1.getFlightDistance();
                    vote_state = 1;
                    fs_counter--;
                    cout<<"y y"<<endl;
                } else {
                    ROS_INFO_STREAM("Failed in direction "<< CD_1.direct<<" and angle "<< CD_1.getFlightAngle_H());
                    fs_state = (fs_state==0)? 1:0;
                    CD_2.lr_postDetector(fs_state,d_safe,minDistance);
                    if (!CD_2.isCollision){
                        CD_2.midDetector(d_safe,d_safe+1,minDistance);
                        if (!CD_2.isCollision)
                        {
                            ROS_INFO("Postdetector redirection... ");
                            info.direct = CD_2.direct;
                            info.angleH = CD_2.getFlightAngle_H();
                            info.angleV = 0;
                            info.detecLength = CD_2.getFlightDistance();
                            vote_state = 1;
                            fs_counter--;
                            cout<<"y n y y"<<endl;
                        } else {
                            fs_state = ((2*n-clusterSize) >-3)? 1 : 0;
                            ss<< "No Freespace in FOV"<<endl;
                            flightStatePub.publish(state);
                            fs_counter++;
                            if(fs_counter>noFS_thres && (vote[5]+vote[7]<vote_thres)){
                                ss<<">>>> Searching new path"<<endl;
                                (fs_state==0)? my_drone.move(0,0,0,speed_emer):my_drone.move(0,0,0,-speed_emer);
                                memset(vote, 0, sizeof(vote));
                            }
                        }
                    } else {
                        fs_state = ((2*n-clusterSize) >-3)? 1 : 0;
                        ss<< "No Freespace in FOV"<<endl;
                        flightStatePub.publish(state);
                        fs_counter++;
                        if(fs_counter>noFS_thres && (vote[5]+vote[7]<vote_thres)){
                            ss<<">>>> Searching new path"<<endl;
                            (fs_state==0)? my_drone.move(0,0,0,speed_emer):my_drone.move(0,0,0,-speed_emer);
                            memset(vote, 0, sizeof(vote));
                        }
                    }
                }
            } else {
                ROS_INFO_STREAM("Failed in direction "<< CD_1.direct<<" and angle "<< CD_1.getFlightAngle_H());
                fs_state = (fs_state==0)? 1:0;
                CD_3.lr_postDetector(fs_state,d_safe,minDistance);
                if (!CD_3.isCollision){
                    CD_3.midDetector(d_safe,d_safe+1,minDistance);
                    if (!CD_3.isCollision)
                    {
                        ROS_INFO("Postdetector redirection... ");
                        info.direct = CD_3.direct;
                        info.angleH = CD_3.getFlightAngle_H();
                        info.angleV = 0;
                        info.detecLength = CD_3.getFlightDistance();
                        vote_state = 1;
                        fs_counter--;
                        cout<<"n y y"<<endl;
                    } else {
                        fs_state = ((2*n-clusterSize) >-3)? 1 : 0;
                        ss<< "No Freespace in FOV"<<endl;
                        flightStatePub.publish(state);
                        fs_counter++;
                        if(fs_counter>noFS_thres && (vote[5]+vote[7]<vote_thres)){
                            ss<<">>>> Searching new path"<<endl;
                            (fs_state==0)? my_drone.move(0,0,0,speed_emer):my_drone.move(0,0,0,-speed_emer);
                            memset(vote, 0, sizeof(vote));
                        }
                    }
                } else {
                    fs_state = ((2*n-clusterSize) >-3)? 1 : 0;
                    ss<< "No Freespace in FOV"<<endl;
                    flightStatePub.publish(state);
                    fs_counter++;
                    if(fs_counter>noFS_thres && (vote[5]+vote[7]<vote_thres)){
                        ss<<">>>> Searching new path"<<endl;
                        (fs_state==0)? my_drone.move(0,0,0,speed_emer):my_drone.move(0,0,0,-speed_emer);
                        memset(vote, 0, sizeof(vote));
                    }
                }
            }
                //vote valid
            if (vote_state == 1){
                switch (info.direct){
                case 0:
                    vote[r30] = vote[r30]+2;
                    break;
                case 4:
                    vote[l30] = vote[l30]+2;
                    break;
                }
                std::string pre_str;
                switch (info.direct){
                case 0:
                {
                    pre_str = "right " +std::to_string(info.angleH);
                    break;
                }
                case 4:
                {
                    pre_str = "left " +std::to_string(-info.angleH);
                    break;
                }
                }
                ss<< "Post- & Mid-: go "+pre_str<<endl;
            }
            if (fs_counter<0){
                speed=(minDistance-d_safe)/d_preEnd*v_max+0.25*v_min;
                ss << "slow down ..."<<endl;
                my_drone.move(speed,0,0,0);
            }
        } else if (minDistance<(d_safe+1.0)) {
            int voteId = std::max_element(vote,vote+sizeof(vote)/sizeof(vote[0]))-vote;
            switch (voteId){
            case 0:
                info.direct = 4;
                info.angleH = -30;
                info.angleV = 0;
                break;
            case 1:
                info.direct = 4;
                info.angleH = -60;
                info.angleV = 0;
                break;
            case 2:
                info.direct = 0;
                info.angleH = 30;
                info.angleV = 0;
                break;
            case 3:
                info.direct = 0;
                info.angleH = 60;
                info.angleV = 0;
                break;
            case 4:
                info.direct = 2;
                info.angleH = 0;
                info.angleV = 30;
                break;
            case 5:
                info.direct = 2;
                info.angleH = 0;
                info.angleV = 60;
                break;
            case 6:
                info.direct = 6;
                info.angleH = 0;
                info.angleV = -30;
                break;
            case 7:
                info.direct = 6;
                info.angleH = 0;
                info.angleV = -60;
                break;
            }
            if (vote[voteId]<vote_thres){// for emergency situation
                fs_state = ((2*n-clusterSize) >-3)? 1 : 0;
                ss<< "No Freespace for vote not enough"<<endl;
                flightStatePub.publish(state);
//                fs_counter++;
//                if(fs_counter>noFS_thres){
                    ss<<">>>> Searching new path"<<endl;
                    (fs_state==0)? my_drone.move(0,0,0,speed_emer):my_drone.move(0,0,0,-speed_emer);
                    memset(vote, 0, sizeof(vote));
//                }
            } else{
                std::string mis_str;
                switch (info.direct){
                case 0:
                {
                    mis_str = "right " +std::to_string(info.angleH);
                    break;
                }
                case 4:
                {
                    mis_str = "left " +std::to_string(-info.angleH);
                    break;
                }
                case 2:
                {
                    mis_str = "up " +std::to_string(info.angleV);
                    break;
                }
                case 6:
                {
                    mis_str = "down " +std::to_string(-info.angleV);
                    break;
                }
                }
                speed = 0.5*v_max;
                ss<< "Go "<<mis_str<<endl;
                mission(info.direct,info.angleH,info.angleV);
            }

        }

    } else if (noObs_counter==noObs_thres && obs_state==0){
        if (speed<v_max) {
            speed = speed + 0.1*v_min;
            ss << "boosting..."<<endl;
            my_drone.move(speed,0,0,0);
        } else {
            ss<< "======= SAFE FLYING ======="<<endl;
            turn_state = 0; // allow to turn to target
            speed = v_max;
            my_drone.move(speed,0,0,0);
        }
    } else if (noObs_counter<noObs_thres && obs_state==0){
            noObs_counter++;
    }

    }
    if(!assistant_active && want_to_be_active) {
        my_drone.obtain_control(this->nh);
        this->assistant_active = true;
    }
    if(assistant_active && !want_to_be_active) {
        my_drone.give_up_control(this->nh);
        this->assistant_active = false;
    }

    ss<<"vote state:"+std::to_string(vote[0])
                     +" "+std::to_string(vote[1])
                     +" "+std::to_string(vote[2])
                     +" "+std::to_string(vote[3])
                     +" "+std::to_string(vote[5])
                     +" "+std::to_string(vote[7])<<endl;
    state.data = ss.str();
    flightStatePub.publish(state);

    pubMsg.data.push_back(speed);
    pubMsg.data.push_back(minDistance);
    pubMsg.data.push_back(assistant_active);
    disAngPub.publish(pubMsg);

    //decision result shown in rviz
    if (noObs_counter!=noObs_thres && vote_state == 1){
        int voteMaxId = std::max_element(vote,vote+sizeof(vote)/sizeof(vote[0]))-vote;
        switch (voteMaxId){
        case 0:
            info_viewer.direct = 4;
            info_viewer.angleH = -30;
            info_viewer.angleV = 0;
            break;
        case 1:
            info_viewer.direct = 4;
            info_viewer.angleH = -60;
            info_viewer.angleV = 0;
            break;
        case 2:
            info_viewer.direct = 0;
            info_viewer.angleH = 30;
            info_viewer.angleV = 0;
            break;
        case 3:
            info_viewer.direct = 0;
            info_viewer.angleH = 60;
            info_viewer.angleV = 0;
            break;
        case 4:
            info_viewer.direct = 2;
            info_viewer.angleH = 0;
            info_viewer.angleV = 30;
            break;
        case 5:
            info_viewer.direct = 2;
            info_viewer.angleH = 0;
            info_viewer.angleV = 60;
            break;
        case 6:
            info_viewer.direct = 6;
            info_viewer.angleH = 0;
            info_viewer.angleV = -30;
            break;
        case 7:
            info_viewer.direct = 6;
            info_viewer.angleH = 0;
            info_viewer.angleV = -60;
            break;
        }
    } else {
        info_viewer.direct = 0;
        info_viewer.angleH = 0;
    }
    directMsg.data.push_back(minDistance);
    directMsg.data.push_back(info_viewer.detecLength);
    directMsg.data.push_back(info_viewer.direct);
    directMsg.data.push_back(info_viewer.angleH);
    directPub.publish(directMsg);

//    ROS_INFO_STREAM(fs_counter);
    direct_state = 0;
    obs_state = 0;
    vote_state = 0;
}

void direction_decision::mission(int direct, int angleH, int angleV){
    switch (direct) {
        case 0: {
//            if (angleH == 30) {
//                my_drone.move(0.606 * speed, -speed, 0, 0);
//                //ros::Duration(distance * sin(fabs(angleH)) / (speed * 15)).sleep();
//            } else {
//                my_drone.move(0.202 * speed, -speed, 0, 0);
//               //ros::Duration(distance * sin(fabs(angleH)) / (speed * 15)).sleep();
//            }
            my_drone.move(speed, -speed*tan(DEG2RAD(angleH))*C_speed, 0, 0);
            break;
        }
        case 2: {
            my_drone.move(speed, 0, speed*tan(DEG2RAD(angleV))*C_speed, 0);
            break;
        }
        case 4: {
            my_drone.move(speed, -speed*tan(DEG2RAD(angleH))*C_speed, 0, 0);
            break;
        }
        case 6: {
            my_drone.move(speed, 0, speed*tan(DEG2RAD(angleV))*C_speed, 0);
            break;
        }
/*        case 1: {
//            float cos_angle = (cos(angleH * C_PI / 180) + cos(angleV * C_PI / 180)) /
//                              (2 + 2 * cos(angleH * C_PI / 180) * cos(angleV * C_PI / 180));
//            float angle = acos(cos_angle);
//            if (angleH == 30) {
//                if (angleV == 30) {
//                    my_drone.move(0.606 * speed, -speed, speed, 0);
//                    //ros::Duration(distance * sin(angle) / (speed * 15)).sleep();
//                } else {
//                    my_drone.move(0.606 * speed, -speed, 3*speed, 0);
//                    //ros::Duration(distance * sin(angle) / (speed * 15)).sleep();
//                }
//            } else {
//                if (angleV == 60) {
//                    my_drone.move(0.202 * speed, -speed, speed, 0);
//                    //ros::Duration(distance * sin(angle) / (speed * 15)).sleep();
//                } else {
//                    my_drone.move(0.202 * speed, -speed, 0.333*speed, 0);
//                    //ros::Duration(distance * sin(angle) / (speed * 15)).sleep();
//                }

//            }
//            break;
//        }

//        case 3: {
//            float cos_angle = (cos(angleH * C_PI / 180) + cos(angleV * C_PI / 180)) /
//                              (2 + 2 * cos(angleH * C_PI / 180) * cos(angleV * C_PI / 180));
//            float angle = acos(cos_angle);
//            if (angleH == 30) {
//                if (angleV == 30) {
//                    my_drone.move(0.606 * speed, speed, speed, 0);
//                    //ros::Duration(distance * sin(angle) / (speed * 15)).sleep();
//                } else {
//                    my_drone.move(0.606 * speed, speed, 3*speed, 0);
//                    //ros::Duration(distance * sin(angle) / (speed * 15)).sleep();
//                }
//            } else {
//                if (angleV == 60) {
//                    my_drone.move(0.202 * speed, speed, speed, 0);
//                    //ros::Duration(distance * sin(angle) / (speed * 15)).sleep();
//                } else {
//                    my_drone.move(0.202 * speed, speed, 0.333*speed, 0);
//                    //ros::Duration(distance * sin(angle) / (speed * 15)).sleep();
//                }
//            }
//            break;
//        }


//        case 5: {
//            float cos_angle = (cos(angleH * C_PI / 180) + cos(angleV * C_PI / 180)) /
//                              (2 + 2 * cos(angleH * C_PI / 180) * cos(angleV * C_PI / 180));
//            float angle = acos(cos_angle);
//            if (angleH == 30) {
//                if (angleV == 30) {
//                    my_drone.move(0.606 * speed, speed, -speed, 0);
//                    //ros::Duration(distance * sin(angle) / (speed * 15)).sleep();
//                } else {
//                    my_drone.move(0.606 * speed, speed, -3*speed, 0);
//                    //ros::Duration(distance * sin(angle) / (speed * 15)).sleep();
//                }
//            } else {
//                if (angleV == 60) {
//                    my_drone.move(0.202 * speed, speed, -speed, 0);
//                    //ros::Duration(distance * sin(angle) / (speed * 15)).sleep();
//                } else {
//                    my_drone.move(0.202 * speed, speed, -0.333*speed, 0);
//                    //ros::Duration(distance * sin(angle) / (speed * 15)).sleep();
//                }
//            }
//            break;
//        }

//        case 7: {
//            float cos_angle = (cos(angleH * C_PI / 180) + cos(angleV * C_PI / 180)) /
//                              (2 + 2 * cos(angleH * C_PI / 180) * cos(angleV * C_PI / 180));
//            float angle = acos(cos_angle);
//            if (angleH == 30) {
//                if (angleV == 30) {
//                    my_drone.move(0.606 * speed, -speed, -speed, 0);
//                    //ros::Duration(distance * sin(angle) / (speed * 15)).sleep();
//                } else {
//                    my_drone.move(0.606 * speed, -speed, -3*speed, 0);
//                    //ros::Duration(distance * sin(angle) / (speed * 15)).sleep();
//                }
//            } else {
//                if (angleV == 60) {
//                    my_drone.move(0.202 * speed, -speed, -speed, 0);
//                    //ros::Duration(distance * sin(angle) / (speed * 15)).sleep();
//                } else {
//                    my_drone.move(0.202 * speed, -speed, -0.333*speed, 0);
//                    //ros::Duration(distance * sin(angle) / (speed * 15)).sleep();
//                }

//            }
//            break;
//        }
*/
    }
}

void direction_decision::receive_rc_info(const sensor_msgs::JoyConstPtr &joy_msg){

    this->rc_info = joy_msg->axes;


//    if(!received_rc_info.empty()){
//        this->processed_rc_info = this->received_rc_info;

//        if (this->received_rc_info[4] == MODE_P && this->last_rc_flight_mode == MODE_A){
//            this->processed_rc_info[4] = 1.0;
//        }

//        else if (this->received_rc_info[4] == MODE_A && this->last_rc_flight_mode == MODE_P){
//            this->processed_rc_info[4] = -1.0;
//        }

//        else {
//            this->processed_rc_info[4] = 0;
//        }
//        this->received_rc_info.clear();
//    }
}
/*
void direction_decision::motionCtrl(Box box, float width, float length, float height) {
    if (box.y_max >= -0.5 && box.y_max <= 0.5) {
        switch (optDown) {
            case 0:
                ROS_INFO( "***** turn down *****" );
                drone_stop(0.5);
                optDown = 1;
                optRise = 0;
                break;
            case 1:
                drone_updown(-0.02);
                break;
        }
        return;
    } else {optDown = 0;}
    float cita, C1, C2 ,x;
    int ang;
    if ((box.x_min + box.x_max) / 2 >= 0) {
        //safe mode
//            C1 = box.x_min;
//            C2 = width/2.;
//            x = (2*length*pow(C1,2)
//                       +sqrt(4*pow(length,2)*pow(C1,4)+4*(pow(C1,2)-pow(C2,2))*pow(C1,2)*pow(C2,2)))
//                      /(2*(pow(C1,2)-pow(C2,2)));
            cita = atan((width/2.-box.x_min)/(box.z_min-length/2.)) ;
            ROS_INFO_STREAM("cita = "<<cita*180/C_PI);
            ang = (cita<(30/C_PI)) ? 45 : ((cita<(60/C_PI))? 60 : 90);
            switch (optSide) {
                case 0:
                    ROS_INFO( "***** go ahead forward left *****" );
                    drone_stop(1.0);
                    optSide = 1;
                    break;
                case 1:
                    switch (ang) {
                        case 45:
                            my_drone.pitch_l45(speed);
                            ros::Duration(0.05).sleep();
                            break;
                        case 60:
                            my_drone.pitch_l30(speed);
                            ros::Duration(0.05).sleep();
                            break;
                        case 90:
                            my_drone.roll(speed);
                            ros::Duration(0.05).sleep();
                            break;
                    }
                    break;
            }

    }
    else {
//            C1 = box.x_max;
//            C2 = width/2.;
//            x = (2*length*pow(C1,2)
//                 +sqrt(4*pow(length,2)*pow(C1,4)+4*(pow(C1,2)-pow(C2,2))*pow(C1,2)*pow(C2,2)))
//                /(2*(pow(C1,2)-pow(C2,2)));
            cita = atan((width/2.+box.x_max)/(box.z_min-length/2.)) ;
            ROS_INFO_STREAM("cita = "<<cita*180/C_PI);
            ang = (cita<(30/C_PI)) ? 45 : ((cita<(60/C_PI))? 60 : 90);
        switch (optSide) {
            case 0:
                ROS_INFO( "***** go ahead forward right *****" );
                drone_stop(1.0);
                optSide = 1;
                break;
            case 1:
                switch (ang) {
                    case 45:
                        my_drone.pitch_r45(speed);
                        ros::Duration(0.05).sleep();
                        break;
                    case 60:
                        my_drone.pitch_r30(speed);
                        ros::Duration(0.05).sleep();
                        break;
                    case 90:
                        my_drone.roll(-speed);
                        ros::Duration(0.05).sleep();
                        break;
                }
                break;
        }

    }
}
*/
void direction_decision::drone_forward(){
    my_drone.pitch(speed);
//    ros::Duration(time).sleep();
}

void direction_decision::drone_stop(float time){
    my_drone.hover();
    ros::Duration(time).sleep();
}

void direction_decision::drone_rotation(float time){
    //plus: turn left; minus: turn right
    my_drone.yaw(time/fabs(time)*speed_ro);
//    ros::Duration(fabs(time)).sleep();
}

void direction_decision::drone_updown(float time){
    //plus: turn up; minus: turn down
    my_drone.rise(time/fabs(time)*speed);
    ros::Duration(fabs(time)).sleep();
}

void direction_decision::setTarget(float x, float y, float z){
    target_x=x; //longtitude
    target_y=y; //latitude
    target_z=z;
//    std::cout<<"set target successfully!"<<std::endl;
}

void direction_decision::setDronePos(float x, float y, float z, float yaw){
    drone_x=x;
    drone_y=y;
    drone_z=z;
    drone_yaw=yaw;
//    std::cout<<"set drone position successfully!"<<std::endl;
}

float direction_decision::getAimAngle(){

    float yaw_ture = C_PI /2 + drone_yaw;
    //drone_v: drone heading vector
    float drone_v[2] = {cos(yaw_ture), sin(yaw_ture)};
    //drone2tar_v: drone -> target point vector
    float drone2tar_v[2] = {target_x - drone_x, target_y - drone_y};
    float target_v[2] = {target_x,target_y};
    float drone_v2[2] = {drone_x+cos(yaw_ture),drone_y+sin(yaw_ture)} ;
    float angle_diff= getAngle(drone_v,drone2tar_v);
    float turn_sign = judgeTurnDirection(drone_v, drone2tar_v);

    return turn_sign*angle_diff;

}

float direction_decision::judgeTurnDirection(const float *pos1, const float *pos2){
    float c=pos1[0]*pos2[1]-pos1[1]*pos2[0];
    if (c>0) return 1.0; // turn left
    else return -1.0; // turn right
}

float direction_decision::getAngle(const float *v1, const float *v2){
    float x1=v1[0]; float y1=v1[1];
    float x2=v2[0]; float y2=v2[1];
    float v1_length= getLength(v1);
    float v2_length= getLength(v2);
    float cos_angle = (x1 * x2 + y1 * y2) / (v1_length * v2_length);
    return acos(cos_angle);
}

float direction_decision::getLength(const float *v) {
    return (sqrt(pow(v[0],2)+pow(v[1],2)));
}

geometry_msgs::Vector3 direction_decision::toEulerAngle(geometry_msgs::Quaternion quat)
{
    geometry_msgs::Vector3 ans;

    tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
    R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
    return ans;
}

void direction_decision::attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
    current_atti = msg->quaternion;
}
void direction_decision::target_gpsCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    target_x=msg->data.at(0);
    target_y=msg->data.at(1);
//    takeoff_state=msg->data.at(2);
}
void direction_decision::current_gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    current_gps = *msg;
}
void direction_decision::reset(){
    speed = v_min;
    speed_ro = 0;
    obs_state = 0;
    turn_state = 0;
    vote_state = 0;
    direct_state = 0;
    memset(vote, 0, sizeof(vote)/sizeof(vote[0]));
    minDistance_pre = 0;
}
Box BoundingBox(PtCdtr cluster) {

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, Box box, int id){
    std::string cube = "box"+std::to_string(id);
    viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, 1, 1, 1, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
}
