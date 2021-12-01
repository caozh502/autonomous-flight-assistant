//
// Created by caleb on 10.05.21.
//

#include "CollisionDetection.h"
#define C_PI (double)3.141592653589793
PtCdtr cp_filtered;

Box createBox(Eigen::Vector3f uav_point,
              float width, float length, float height,
              const float offset[]){
    Box uav;
    uav.x_min=uav_point(0)+offset[0];
    uav.x_max=uav_point(0)+width+offset[0];
    uav.y_min=uav_point(1)+offset[1];
    uav.y_max=uav_point(1)+height+offset[1];
    uav.z_min=uav_point(2)+offset[2];
    uav.z_max=uav_point(2)+length+offset[2];
    return uav;
}

Eigen::Matrix3f collision_detection::setRPY(Eigen::Vector3f rot){
    Eigen::Matrix3f ret;
    ret = Eigen::AngleAxisf(rot.x(), Eigen::Vector3f::UnitX())
          * Eigen::AngleAxisf(rot.y(), Eigen::Vector3f::UnitY())
          * Eigen::AngleAxisf(rot.z(), Eigen::Vector3f::UnitZ());
    return ret;
}

fcl::CollisionObjectf collision_detection::createCollisionObject(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ptr){
    // octomap octree settings
    const float resolution = 0.1;
    const float prob_hit = 0.9;
    const float prob_miss = 0.1;
    const float clamping_thres_min = 0.12;
    const float clamping_thres_max = 0.98;
    octomap::point3d sensor_origin_3d (0.,0.,0.);

    std::shared_ptr<octomap::OcTree> octomap_octree = std::make_shared<octomap::OcTree>(resolution);
    octomap_octree->setProbHit(prob_hit);
    octomap_octree->setProbMiss(prob_miss);
    octomap_octree->setClampingThresMin(clamping_thres_min);
    octomap_octree->setClampingThresMax(clamping_thres_max);
    octomap::Pointcloud octoCloud;
    for(pcl::PointCloud<pcl::PointXYZ>::iterator it = pointcloud_ptr->begin(); it < pointcloud_ptr->end(); ++it)
    {
        octoCloud.push_back(octomap::point3d(it->x, it->y,it->z));
    }
    octomap_octree->insertPointCloud(octoCloud, sensor_origin_3d, -1, true, true);
    octomap_octree->updateInnerOccupancy();

    auto fcl_octree = std::make_shared<fcl::OcTreef> (octomap_octree);
    return fcl::CollisionObjectf (fcl_octree);
}


void CollisionVisualization(pcl::visualization::PCLVisualizer::Ptr& viewer, Box box, Color color,int id){
    std::string cube = "uav"+std::to_string(id);
    viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.r, color.g, color.b, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cube);
}

void UAVBoxVisualization(pcl::visualization::PCLVisualizer::Ptr& viewer, Box box, Color color,int id){
    std::string cube = "uavBox"+std::to_string(id);
    viewer->removeShape("uavBox"+std::to_string(id));
    viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.r, color.g, color.b, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
}

collision_detection::collision_detection(const PtCdtr &cluster, float width, float length, float height){
    auto startTime = std::chrono::steady_clock::now();

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudCluster(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cluster, *cloudCluster);
    auto octree_point = createCollisionObject(cloudCluster);
    std::shared_ptr<fcl::CollisionGeometryf> box_geometry1(new fcl::Boxf(width,height,length));
    fcl::CollisionObjectf box1(box_geometry1);

    fcl::Vector3f trans1(0.,0.,0.);
    fcl::Vector3f rot1(0.,0.,0.);
    fcl::Vector3f trans2(0.,0.,0.);
    fcl::Vector3f rot2(0.,0.,0.);
    box1.setTranslation(trans1);
    box1.setRotation(setRPY(rot1));
    octree_point.setTranslation(trans2);
    octree_point.setRotation(setRPY(rot2));
    // DynamicAABBTree BroadPhase Managers
    std::shared_ptr<fcl::BroadPhaseCollisionManager<float>> group1 = std::make_shared<fcl::DynamicAABBTreeCollisionManager<float>>();

    // Set Objects
    group1->registerObject(&octree_point);
    group1->setup();

    // Contact Number between env and que
    group1->collide(&box1, &c_data1, fcl::DefaultCollisionFunction);
    group1->distance(&box1, &d_data1, fcl::DefaultDistanceFunction);
    isCollision = c_data1.result.isCollision();
    min_distance = d_data1.result.min_distance;
    //std::cout << res << std::endl;
//    request.enable_nearest_points = true;
//    request.enable_signed_distance = true;
//    request.gjk_solver_type = fcl::GJKSolverType::GST_INDEP;
//    //request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
//
//    // Calculate distance
//    result.clear();
//    //fcl::collide(&box1, &octree_point,C_request,C_result);
//    fcl::distance(&box1, &octree_point, request, result);
//    min_distance = result.min_distance;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    ROS_INFO_STREAM( "Collision Detection took " << elapsedTime.count() << " ms");
}


collision_detection::collision_detection(Box obj_in,Box drone_in){
    obj = obj_in;
    drone = drone_in;
    drone_w = drone.x_max-drone.x_min;
    drone_h = drone.y_max-drone.y_min;
    drone_l = drone.z_max-drone.z_min;

}
collision_detection::collision_detection(Box drone_in){
    drone = drone_in;
    drone_w = drone.x_max-drone.x_min;
    drone_h = drone.y_max-drone.y_min;
    drone_l = drone.z_max-drone.z_min;

}

void collision_detection::update_droneBox(Box drone_in){
    drone = drone_in;
    drone_w = drone.x_max-drone.x_min;
    drone_h = drone.y_max-drone.y_min;
    drone_l = drone.z_max-drone.z_min;
}
void collision_detection::update_box(Box obj_in){
    obj = obj_in;
}
void collision_detection::rough_detector(){

    float obj_w = obj.x_max-obj.x_min;
    float obj_h = obj.y_max-obj.y_min;
    float obj_l = obj.z_max-obj.z_min;
//    std::cout<<obj_w<<" "<<obj_h <<" "<<obj_l<<" "<<std::endl;
    //center of object
    float objc_x = (obj.x_max+obj.x_min)/2.;
    float objc_y = (obj.y_max+obj.y_min)/2.;
    float objc_z = (obj.z_max+obj.z_min)/2.;
//    std::cout<<objc_x<<" "<<objc_y <<" "<<objc_z<<" "<<std::endl;

    std::shared_ptr<fcl::CollisionGeometryf> drone_geometry(new fcl::Boxf(drone_w,drone_h,drone_l));
    fcl::CollisionObjectf box1(drone_geometry);
    std::shared_ptr<fcl::CollisionGeometryf> obj_geometry(new fcl::Boxf(obj_w,obj_h,obj_l));
    fcl::CollisionObjectf box2(obj_geometry);

    fcl::Vector3f trans1(cameraPosition[0],cameraPosition[1],cameraPosition[2]+drone_l/2.-drone_l_t/2.);
    fcl::Vector3f rot1(0.,0.,0.);
    fcl::Vector3f trans2(objc_x,objc_y,objc_z);
    fcl::Vector3f rot2(0.,0.,0.);
    box1.setTranslation(trans1);
    box1.setRotation(setRPY(rot1));
    box2.setTranslation(trans2);
    box2.setRotation(setRPY(rot2));

    request.enable_nearest_points = true;
    request.enable_signed_distance = true;
    request.gjk_solver_type = fcl::GJKSolverType::GST_INDEP;
    //request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;

    // Calculate distance
    result.clear();
    C_result.clear();
    fcl::collide(&box1, &box2,C_request,C_result);
    fcl::distance(&box1, &box2, request, result);
    isCollision = C_result.isCollision();
    min_distance = result.min_distance;
//    min_distance = obj.z_min - drone.z_max;
}
void collision_detection::lr_detector(int fs_state){


    switch (fs_state) {
        case 0: {
            /// region 4, left
            cita_h = atan((drone_w / 2. - obj.x_min) / (obj.z_min - drone_l / 2.));
            //ROS_INFO_STREAM("cita_h = " << cita_h * 180 / C_PI);
            ang_h = (cita_h < (45 * C_PI / 180)) ? -30 : -60;
            direct = 4;
            break;
        }
        case 1: {
            /// region 0, right
            cita_h = atan((drone_w/2.+obj.x_max)/(obj.z_min-drone_l/2.)) ;
            //ROS_INFO_STREAM("cita_h = " << cita_h * 180 / C_PI);
            ang_h = (cita_h < (45*C_PI/180)) ? 30 : 60 ;
            direct = 0;
            break;
        }
    }

    detec_l = obj.z_min/cos(ang_h*C_PI/180);
    //cout<<detec_l<<" "<< cos(fabs(ang_h)*C_PI/180)<<" "<<obj.z_min<<endl;
    std::shared_ptr<fcl::CollisionGeometryf> drone_geometry(new fcl::Boxf(drone_w,drone_h,detec_l));
    fcl::CollisionObjectf box1(drone_geometry);
    // DynamicAABBTree BroadPhase Managers
    std::shared_ptr<fcl::BroadPhaseCollisionManager<float>> group1 = std::make_shared<fcl::DynamicAABBTreeCollisionManager<float>>();

    fcl::Vector3f trans1(0.,0.,detec_l/2.);
    fcl::Vector3f rot1(0.,ang_h*C_PI/180,0.);
    box1.setTranslation(trans1);
    box1.setRotation(setRPY(rot1));

//    for (const PtCdtr &cluster : cloudClusters) {
//        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudCluster(new pcl::PointCloud<pcl::PointXYZ>);
//        pcl::copyPointCloud(*cluster, *cloudCluster);
//        auto octree_point = createCollisionObject(cloudCluster);
//        fcl::Vector3f trans2(0.,0.,0.);
//        fcl::Vector3f rot2(0.,0.,0.);
//        octree_point.setTranslation(trans2);
//        octree_point.setRotation(setRPY(rot2));
//        // Set Objects
//        group1->registerObject(&octree_point);
//    }
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cp_filtered, *cloud);
    auto octree_point = createCollisionObject(cloud);

    fcl::Vector3f trans2(0,0,0);
    fcl::Vector3f rot2(0.,0.,0.);
    octree_point.setTranslation(trans2);
    octree_point.setRotation(setRPY(rot2));

    group1->registerObject(&octree_point);
    group1->setup();


    // Contact Number between env and que
    c_data1.request.gjk_solver_type = fcl::GJKSolverType::GST_INDEP;
    c_data1.result.clear();
    group1->collide(&box1, &c_data1, fcl::DefaultCollisionFunction);
    isCollision = c_data1.result.isCollision(); // 1:collision; 0: no collision

    if (isCollision){
        ROS_INFO("redirection... ");
        switch (ang_h) {
            case -30: {
                ang_h = -60;
                break;
            }
            case 30:{
                ang_h = 60;
                break;
            }
        }

        detec_l = obj.z_min/cos(ang_h*C_PI/180);
        std::shared_ptr<fcl::CollisionGeometryf> drone_geometry(new fcl::Boxf(drone_w,drone_h,detec_l));
        fcl::CollisionObjectf box1(drone_geometry);
        trans1<<0.,0.,detec_l/2.;
        rot1<<0.,ang_h * C_PI / 180, 0.;
        box1.setRotation(setRPY(rot1));
        box1.setTranslation(trans1);
        c_data1.result.clear();
        group1->collide(&box1, &c_data1, fcl::DefaultCollisionFunction);
        isCollision = c_data1.result.isCollision();
    }

}
void collision_detection::lr_preDetector(int fs_state, float d_s, float minDis){


    switch (fs_state) {
        case 0: {
            /// region 4, left
//            cita_h = atan((drone_w / 2. - obj.x_min) / (obj.z_min - (drone_l/2. +cameraPosition[2]) - (minDis-d_s)));
//            ang_h = (cita_h < (45 * C_PI / 180)) ? -30 : -60;
            ang_h = -60;
            direct = 4;
            break;
        }
        case 1: {
            /// region 0, right
//            cita_h = atan((drone_w/2.+obj.x_max)/(obj.z_min-(drone_l/2. +cameraPosition[2])- (minDis-d_s))) ;
//            ang_h = (cita_h < (45*C_PI/180)) ? 30 : 60 ;
            ang_h = 60;
            direct = 0;
            break;
        }
    }

    detec_l = (obj.z_min-(minDis-d_s)-cameraPosition[2])/cos(ang_h*C_PI/180);
//    cout<<detec_l<<" "<< minDis<<" "<<obj.z_min<<endl;
    true_w = sqrt(pow(drone_w,2)+pow(drone_l,2));
    std::shared_ptr<fcl::CollisionGeometryf> drone_geometry(new fcl::Boxf(true_w,drone_h,detec_l+0.5));
    fcl::CollisionObjectf box1(drone_geometry);
    // DynamicAABBTree BroadPhase Managers
    std::shared_ptr<fcl::BroadPhaseCollisionManager<float>> group1 = std::make_shared<fcl::DynamicAABBTreeCollisionManager<float>>();

    fcl::Vector3f trans1(cameraPosition[0]+detec_l/2.*sin(ang_h*C_PI/180),
                         cameraPosition[1],
                         cameraPosition[2]+detec_l/2.*cos(ang_h*C_PI/180)+ (minDis-d_s));
    fcl::Vector3f rot1(0.,ang_h*C_PI/180,0.);
    box1.setTranslation(trans1);
    box1.setRotation(setRPY(rot1));

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (!cp_filtered->empty()){
        pcl::copyPointCloud(*cp_filtered, *cloud);
        auto octree_point = createCollisionObject(cloud);

        fcl::Vector3f trans2(0,0,0);
        fcl::Vector3f rot2(0.,0.,0.);
        octree_point.setTranslation(trans2);
        octree_point.setRotation(setRPY(rot2));

        group1->registerObject(&octree_point);
        group1->setup();


        // Contact Number between env and que
        c_data1.request.gjk_solver_type = fcl::GJKSolverType::GST_INDEP;
        c_data1.result.clear();
        group1->collide(&box1, &c_data1, fcl::DefaultCollisionFunction);
        isCollision = c_data1.result.isCollision(); // 1:collision; 0: no collision
    }

//    if (isCollision){
//        ROS_INFO("Predetector redirection... ");
//        ang_h = -ang_h;
//        direct = (direct==0)?4:0;
//        detec_l = (obj.z_min-(minDis-d_s)-cameraPosition[2])/cos(ang_h*C_PI/180);
//        std::shared_ptr<fcl::CollisionGeometryf> drone_geometry(new fcl::Boxf(true_w,drone_h,detec_l));
//        fcl::CollisionObjectf box1(drone_geometry);
//        trans1<<cameraPosition[0]+detec_l/2.*sin(ang_h*C_PI/180) ,
//                cameraPosition[1],
//                cameraPosition[2]+detec_l/2.*cos(ang_h*C_PI/180)+ (minDis-d_s);
//        rot1<<0.,ang_h * C_PI / 180, 0.;
//        box1.setRotation(setRPY(rot1));
//        box1.setTranslation(trans1);
//        c_data1.result.clear();
//        group1->collide(&box1, &c_data1, fcl::DefaultCollisionFunction);
//        isCollision = c_data1.result.isCollision();
//    }

}
void collision_detection::midDetector(float d_s1, float d_s2, float minDis){

    int ang;
    if (direct == 0 || direct == 4){
        ang = ang_h;
        ang_v = 0;
    } else {
        ang = ang_v;
        ang_h = 0;
    }

    std::shared_ptr<fcl::CollisionGeometryf> drone_geometry(new fcl::Boxf(drone_w,drone_h,drone_l+d_s2+0.5));
    fcl::CollisionObjectf box1(drone_geometry);
    // DynamicAABBTree BroadPhase Managers
    std::shared_ptr<fcl::BroadPhaseCollisionManager<float>> group1 = std::make_shared<fcl::DynamicAABBTreeCollisionManager<float>>();

    float x_trans = cameraPosition[0] + detec_l*sin(ang_h*C_PI/180);
    float y_trans = cameraPosition[1] - detec_l*sin(ang_v*C_PI/180);
    float z_trans = cameraPosition[2] + d_s2/2.+ (minDis-d_s1)+detec_l*cos(ang*C_PI/180);
//    ROS_INFO_STREAM(x_trans<<" "<<y_trans<<" "<<z_trans<<" "<<(drone_l+d_s2)<<" "<<drone_w<<" "<<drone_h);

    fcl::Vector3f trans1(x_trans,y_trans, z_trans);
//    fcl::Vector3f trans1(0.,0.,0.);
    fcl::Vector3f rot1(0.,0.,0.);
    box1.setTranslation(trans1);
    box1.setRotation(setRPY(rot1));

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (!cp_filtered->empty()){
        pcl::copyPointCloud(*cp_filtered, *cloud);
        auto octree_point = createCollisionObject(cloud);

        fcl::Vector3f trans2(0,0,0);
        fcl::Vector3f rot2(0.,0.,0.);
        octree_point.setTranslation(trans2);
        octree_point.setRotation(setRPY(rot2));

        group1->registerObject(&octree_point);
        group1->setup();


        // Contact Number between env and que
        c_data1.request.gjk_solver_type = fcl::GJKSolverType::GST_INDEP;
        c_data1.result.clear();
        group1->collide(&box1, &c_data1, fcl::DefaultCollisionFunction);
        isCollision = c_data1.result.isCollision(); // 1:collision; 0: no collision
    }

}
void collision_detection::lrud_preDetector(int fs_state, float d_s){

    switch (fs_state) {
        case 0:{
            /// region 1, upper right
            cita_h = atan((drone_w / 2. + obj.x_max) / (obj.z_min - drone_l / 2. - (min_distance-d_s)));
//            ROS_INFO_STREAM("cita_h = " << cita_h * 180 / C_PI);
            ang_h = (cita_h < (45 * C_PI / 180)) ? 30 : 60;
            cita_v = atan((drone_h / 2. - obj.y_min) / (obj.z_min - drone_l / 2. - (min_distance-d_s)));
//            ROS_INFO_STREAM("cita_v = " << cita_v * 180 / C_PI);
            ang_v = (cita_v < (45 * C_PI / 180)) ? 30 : 60 ;
            direct = 1;
            break;
        }
        case 1:{
            /// region 3, upper left
            cita_h = atan((drone_w / 2. - obj.x_min) / (obj.z_min - drone_l / 2. - (min_distance-d_s)));
            ang_h = (cita_h < (45 * C_PI / 180)) ? -30 : -60;
            cita_v = atan((drone_h / 2. - obj.y_min) / (obj.z_min - drone_l / 2. - (min_distance-d_s)));
            ang_v = (cita_v < (45 * C_PI / 180)) ? 30 : 60 ;
            direct = 3;
            break;
        }
        case 2:{
            /// region 5, lower left
            cita_h = atan((drone_w / 2. - obj.x_min) / (obj.z_min - drone_l / 2. - (min_distance-d_s)));
            ang_h = (cita_h < (45 * C_PI / 180)) ? -30 : -60;
            cita_v = atan((drone_h / 2. + obj.y_max) / (obj.z_min - drone_l / 2. - (min_distance-d_s)));
            ang_v = (cita_v < (45 * C_PI / 180)) ? -30 : -60 ;
            direct = 5;
            break;
        }
        case 3:{
            /// region 7, lower right
            cita_h = atan((drone_w / 2. + obj.x_max) / (obj.z_min - drone_l / 2. - (min_distance-d_s)));
            ang_h = (cita_h < (45 * C_PI / 180)) ? 30 : 60;
            cita_v = atan((drone_h / 2. + obj.y_max) / (obj.z_min - drone_l / 2. - (min_distance-d_s)));
            ang_v = (cita_v < (45 * C_PI / 180)) ? -30 : -60 ;
            direct = 7;
            break;
        }
    }

    float cos_angle = (cos(ang_h*C_PI/180)+cos(ang_v*C_PI/180))/(2+2*cos(ang_h*C_PI/180)*cos(ang_v*C_PI/180));
    detec_l = (obj.z_min-(min_distance-d_s))/cos_angle;
    true_w = sqrt(pow(drone_w,2)+pow(drone_l,2)+pow(drone_h,2));
    std::shared_ptr<fcl::CollisionGeometryf> drone_geometry(new fcl::Boxf(drone_w,drone_h,detec_l));
    fcl::CollisionObjectf box1(drone_geometry);
    // DynamicAABBTree BroadPhase Managers
    std::shared_ptr<fcl::BroadPhaseCollisionManager<float>> group1 = std::make_shared<fcl::DynamicAABBTreeCollisionManager<float>>();

    fcl::Vector3f rot1(ang_v*C_PI/180,ang_h*C_PI/180,0.);
    fcl::Vector3f trans1(0.,0.,detec_l/2. +(min_distance-d_s));
    box1.setTranslation(trans1);
    box1.setRotation(setRPY(rot1));

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cp_filtered, *cloud);
    auto octree_point = createCollisionObject(cloud);

    fcl::Vector3f trans2(0,0,0);
    fcl::Vector3f rot2(0.,0.,0.);
    octree_point.setTranslation(trans2);
    octree_point.setRotation(setRPY(rot2));

    group1->registerObject(&octree_point);
    group1->setup();

    // Contact Number between env and que
    c_data1.request.gjk_solver_type = fcl::GJKSolverType::GST_INDEP;
    c_data1.result.clear();
    group1->collide(&box1, &c_data1, fcl::DefaultCollisionFunction);
    isCollision = c_data1.result.isCollision();
}
void collision_detection::lrud_detector(int fs_state){

    switch (fs_state) {
        case 0:{
            /// region 1, upper right
            cita_h = atan((drone_w / 2. + obj.x_max) / (obj.z_min - drone_l / 2.));
//            ROS_INFO_STREAM("cita_h = " << cita_h * 180 / C_PI);
            ang_h = (cita_h < (45 * C_PI / 180)) ? 30 : 60;
            cita_v = atan((drone_h / 2. - obj.y_min) / (obj.z_min - drone_l / 2.));
//            ROS_INFO_STREAM("cita_v = " << cita_v * 180 / C_PI);
            ang_v = (cita_v < (45 * C_PI / 180)) ? 30 : 60 ;
            direct = 1;
            break;
        }
        case 1:{
            /// region 3, upper left
            cita_h = atan((drone_w / 2. - obj.x_min) / (obj.z_min - drone_l / 2.));
            ang_h = (cita_h < (45 * C_PI / 180)) ? -30 : -60;
            cita_v = atan((drone_h / 2. - obj.y_min) / (obj.z_min - drone_l / 2.));
            ang_v = (cita_v < (45 * C_PI / 180)) ? 30 : 60 ;
            direct = 3;
            break;
        }
        case 2:{
            /// region 5, lower left
            cita_h = atan((drone_w / 2. - obj.x_min) / (obj.z_min - drone_l / 2.));
            ang_h = (cita_h < (45 * C_PI / 180)) ? -30 : -60;
            cita_v = atan((drone_h / 2. + obj.y_max) / (obj.z_min - drone_l / 2.));
            ang_v = (cita_v < (45 * C_PI / 180)) ? -30 : -60 ;
            direct = 5;
            break;
        }
        case 3:{
            /// region 7, lower right
            cita_h = atan((drone_w / 2. + obj.x_max) / (obj.z_min - drone_l / 2.));
            ang_h = (cita_h < (45 * C_PI / 180)) ? 30 : 60;
            cita_v = atan((drone_h / 2. + obj.y_max) / (obj.z_min - drone_l / 2.));
            ang_v = (cita_v < (45 * C_PI / 180)) ? -30 : -60 ;
            direct = 7;
            break;
        }
    }

    float cos_angle = (cos(ang_h*C_PI/180)+cos(ang_v*C_PI/180))/(2+2*cos(ang_h*C_PI/180)*cos(ang_v*C_PI/180));
    detec_l = obj.z_min/cos_angle;
    std::shared_ptr<fcl::CollisionGeometryf> drone_geometry(new fcl::Boxf(drone_w,drone_h,detec_l));
    fcl::CollisionObjectf box1(drone_geometry);
    // DynamicAABBTree BroadPhase Managers
    std::shared_ptr<fcl::BroadPhaseCollisionManager<float>> group1 = std::make_shared<fcl::DynamicAABBTreeCollisionManager<float>>();

    fcl::Vector3f rot1(ang_v*C_PI/180,ang_h*C_PI/180,0.);
    fcl::Vector3f trans1(0.,0.,detec_l/2.);
    box1.setTranslation(trans1);
    box1.setRotation(setRPY(rot1));

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cp_filtered, *cloud);
    auto octree_point = createCollisionObject(cloud);

    fcl::Vector3f trans2(0,0,0);
    fcl::Vector3f rot2(0.,0.,0.);
    octree_point.setTranslation(trans2);
    octree_point.setRotation(setRPY(rot2));

    group1->registerObject(&octree_point);
    group1->setup();

    // Contact Number between env and que
    c_data1.request.gjk_solver_type = fcl::GJKSolverType::GST_INDEP;
    c_data1.result.clear();
    group1->collide(&box1, &c_data1, fcl::DefaultCollisionFunction);
    isCollision = c_data1.result.isCollision();
}

void collision_detection::ud_detector(int fs_state){

    switch (fs_state) {
        case 0:{
            /// region 2, upper
            cita_v = atan((drone_h / 2. - obj.y_min) / (obj.z_min - drone_l / 2.));
            ang_v = (cita_v < (45 * C_PI / 180)) ? 30 : 60 ;
            direct = 2;
            break;
        }
        case 1:{
            /// region 6, lower
            cita_v = atan((drone_h / 2. + obj.y_max) / (obj.z_min - drone_l / 2.));
            ang_v = (cita_v < (45 * C_PI / 180)) ? -30 : -60 ;
            direct = 6;
            break;
        }
    }

    detec_l = obj.z_min/cos(ang_v*C_PI/180);
    std::shared_ptr<fcl::CollisionGeometryf> drone_geometry(new fcl::Boxf(drone_w,drone_h,detec_l));
    fcl::CollisionObjectf box1(drone_geometry);
    // DynamicAABBTree BroadPhase Managers
    std::shared_ptr<fcl::BroadPhaseCollisionManager<float>> group1 = std::make_shared<fcl::DynamicAABBTreeCollisionManager<float>>();

    fcl::Vector3f trans1(0.,0.,detec_l/2.);
    fcl::Vector3f rot1(ang_v*C_PI/180,0.,0.);
    box1.setTranslation(trans1);
    box1.setRotation(setRPY(rot1));

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cp_filtered, *cloud);
    auto octree_point = createCollisionObject(cloud);

    fcl::Vector3f trans2(0,0,0);
    fcl::Vector3f rot2(0.,0.,0.);
    octree_point.setTranslation(trans2);
    octree_point.setRotation(setRPY(rot2));

    group1->registerObject(&octree_point);
    group1->setup();

    // Contact Number between env and que
    c_data1.request.gjk_solver_type = fcl::GJKSolverType::GST_INDEP;
    c_data1.result.clear();
    group1->collide(&box1, &c_data1, fcl::DefaultCollisionFunction);
    isCollision = c_data1.result.isCollision();

    if (isCollision){
        ROS_INFO("redirection... ");
        switch (ang_v) {
            case -30: {
                ang_v = -60;
                break;
            }
            case 30:{
                ang_v = 60;
                break;
            }
        }
        detec_l = obj.z_min/cos(ang_v*C_PI/180);
        std::shared_ptr<fcl::CollisionGeometryf> drone_geometry(new fcl::Boxf(drone_w,drone_h,detec_l));
        fcl::CollisionObjectf box1(drone_geometry);
        fcl::Vector3f trans1(0.,0.,detec_l/2.);
        fcl::Vector3f rot1(ang_v*C_PI/180,0.,0.);
        box1.setTranslation(trans1);
        box1.setRotation(setRPY(rot1));
        c_data1.result.clear();
        group1->collide(&box1, &c_data1, fcl::DefaultCollisionFunction);
        isCollision = c_data1.result.isCollision();
    }
}
void collision_detection::ud_preDetector(int fs_state, float d_s, float minDis){

    switch (fs_state) {
        case 0:{
            /// region 2, upper
//            cita_v = atan((drone_h / 2. - obj.y_min) / (obj.z_min - (drone_l/2. +cameraPosition[2])- (minDis-d_s)));
//            ang_v = (cita_v < (45 * C_PI / 180)) ? 30 : 60 ;
            ang_v = 60;
            direct = 2;
            break;
        }
        case 1:{
            /// region 6, lower
//            cita_v = atan((drone_h / 2. + obj.y_max) / (obj.z_min - (drone_l/2. +cameraPosition[2])- (minDis-d_s)));
//            ang_v = (cita_v < (45 * C_PI / 180)) ? -30 : -60 ;
            ang_v = -60;
            direct = 6;
            break;
        }
    }

    detec_l = (obj.z_min- (minDis-d_s)-cameraPosition[2])/cos(ang_v*C_PI/180);
    true_w = sqrt(pow(drone_l,2)+pow(drone_h,2));
    std::shared_ptr<fcl::CollisionGeometryf> drone_geometry(new fcl::Boxf(true_w,drone_h,detec_l));
    fcl::CollisionObjectf box1(drone_geometry);
    // DynamicAABBTree BroadPhase Managers
    std::shared_ptr<fcl::BroadPhaseCollisionManager<float>> group1 = std::make_shared<fcl::DynamicAABBTreeCollisionManager<float>>();

    fcl::Vector3f trans1(cameraPosition[0] ,
                         cameraPosition[1]-detec_l/2.*sin(ang_v*C_PI/180),
                         cameraPosition[2]+detec_l/2.*cos(ang_v*C_PI/180)+ (minDis-d_s));
    fcl::Vector3f rot1(ang_v*C_PI/180,0.,0.);
    box1.setTranslation(trans1);
    box1.setRotation(setRPY(rot1));

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (!cp_filtered->empty()){
        pcl::copyPointCloud(*cp_filtered, *cloud);
        auto octree_point = createCollisionObject(cloud);

        fcl::Vector3f trans2(0,0,0);
        fcl::Vector3f rot2(0.,0.,0.);
        octree_point.setTranslation(trans2);
        octree_point.setRotation(setRPY(rot2));

        group1->registerObject(&octree_point);
        group1->setup();

        // Contact Number between env and que
        c_data1.request.gjk_solver_type = fcl::GJKSolverType::GST_INDEP;
        c_data1.result.clear();
        group1->collide(&box1, &c_data1, fcl::DefaultCollisionFunction);
        isCollision = c_data1.result.isCollision();
    }

//    if (isCollision){
//        ROS_INFO("redirection... ");
//        ang_v = -ang_v;
//        direct = (direct==2)?6:2;
//        detec_l = (obj.z_min- (minDis-d_s)-cameraPosition[2])/cos(ang_v*C_PI/180);
//        std::shared_ptr<fcl::CollisionGeometryf> drone_geometry(new fcl::Boxf(true_w,drone_h,detec_l));
//        fcl::CollisionObjectf box1(drone_geometry);
//        fcl::Vector3f trans1(cameraPosition[0] ,
//                cameraPosition[1]-detec_l/2.*sin(ang_v*C_PI/180),
//                cameraPosition[2]+detec_l/2.*cos(ang_v*C_PI/180)+ (minDis-d_s));
//        fcl::Vector3f rot1(ang_v*C_PI/180,0.,0.);
//        box1.setTranslation(trans1);
//        box1.setRotation(setRPY(rot1));
//        c_data1.result.clear();
//        group1->collide(&box1, &c_data1, fcl::DefaultCollisionFunction);
//        isCollision = c_data1.result.isCollision();
//    }
}
void collision_detection::lr_postDetector(int fs_state, float d_s, float minDis){

    switch (fs_state) {
        case 0: {
            /// region 4, left
            direct = 4;
            ang_h = -30;
            break;
        }
        case 1: {
            /// region 0, right
            direct = 0;
            ang_h = 30;
            break;
        }
    }

    detec_l = (obj.z_min- (minDis-d_s)-cameraPosition[2])/cos(ang_h*C_PI/180);
    true_w = sqrt(pow(drone_w,2)+pow(drone_l,2));
    std::shared_ptr<fcl::CollisionGeometryf> drone_geometry(new fcl::Boxf(true_w,drone_h,detec_l+1));
    fcl::CollisionObjectf box1(drone_geometry);
    // DynamicAABBTree BroadPhase Managers
    std::shared_ptr<fcl::BroadPhaseCollisionManager<float>> group1 = std::make_shared<fcl::DynamicAABBTreeCollisionManager<float>>();
    float x_trans = cameraPosition[0] + detec_l/2.*sin(ang_h*C_PI/180);
    float y_trans = cameraPosition[1] ;
    float z_trans = cameraPosition[2] + (minDis-d_s)+detec_l/2.*cos(ang_h*C_PI/180);
    float ang_y = ang_h*C_PI/180;
//    ROS_INFO_STREAM(ang_h<<" "<<x_trans<<" "<<y_trans<<" "<<z_trans<<" "<<true_w<<" "<<drone_h<<" "<<detec_l);
//    cout<<drone_w<<" "<<drone_h<<" "<<(drone_l+d_s2)<<endl;
    fcl::Vector3f trans1(x_trans,y_trans, z_trans);
    fcl::Vector3f rot1(0.,ang_y,0.);
    box1.setTranslation(trans1);
    box1.setRotation(setRPY(rot1));

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (!cp_filtered->empty())
    {
        pcl::copyPointCloud(*cp_filtered, *cloud);
        auto octree_point = createCollisionObject(cloud);

        fcl::Vector3f trans2(0,0,0);
        fcl::Vector3f rot2(0.,0.,0.);
        octree_point.setTranslation(trans2);
        octree_point.setRotation(setRPY(rot2));

        group1->registerObject(&octree_point);
        group1->setup();


        // Contact Number between env and que
        c_data1.request.gjk_solver_type = fcl::GJKSolverType::GST_INDEP;
        c_data1.result.clear();
        group1->collide(&box1, &c_data1, fcl::DefaultCollisionFunction);
        isCollision = c_data1.result.isCollision(); // 1:collision; 0: no collision
    }

}
/*void collision_detection::emergency_detector(int fs_state){
//    switch (fs_state) {
//        case 0: {
//            /// sideways to left
//            detec_l = (drone_w / 2 - obj.x_min);
//            ang_h = -90;
//            break;
//        }
//        case 1: {
//            /// sideways to right
//            detec_l = (obj.x_max + drone_w / 2);
//            ang_h = 90;
//            break;
//        }
//    }

//    direct = 8;

//    std::shared_ptr<fcl::CollisionGeometryf> drone_geometry(new fcl::Boxf(drone_w,drone_h,detec_l));
//    fcl::CollisionObjectf box1(drone_geometry);
//    // DynamicAABBTree BroadPhase Managers
//    std::shared_ptr<fcl::BroadPhaseCollisionManager<float>> group1 = std::make_shared<fcl::DynamicAABBTreeCollisionManager<float>>();

//    fcl::Vector3f trans1(0.,0.,detec_l/2.);
//    fcl::Vector3f rot1(0,ang_h*C_PI/180,0.);
//    box1.setTranslation(trans1);
//    box1.setRotation(setRPY(rot1));

//    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::copyPointCloud(*cp_filtered, *cloud);
//    auto octree_point = createCollisionObject(cloud);

//    fcl::Vector3f trans2(0,0,0);
//    fcl::Vector3f rot2(0.,0.,0.);
//    octree_point.setTranslation(trans2);
//    octree_point.setRotation(setRPY(rot2));

//    group1->registerObject(&octree_point);
//    group1->setup();

//    // Contact Number between env and que
//    group1->collide(&box1, &c_data1, fcl::DefaultCollisionFunction);
//    isCollision = c_data1.result.isCollision();
//}
*/
void collision_detection::nav_detector(float angle_in){
    std::shared_ptr<fcl::CollisionGeometryf> drone_geometry(new fcl::Boxf(drone_w,drone_h,drone_l/2.));
    fcl::CollisionObjectf box1(drone_geometry);
    // DynamicAABBTree BroadPhase Managers
    std::shared_ptr<fcl::BroadPhaseCollisionManager<float>> group1 = std::make_shared<fcl::DynamicAABBTreeCollisionManager<float>>();

    fcl::Vector3f trans1(cameraPosition[0]+(drone_l/2.-drone_l_t)/2.*sin(angle_in),
                         cameraPosition[1],
                         cameraPosition[2]+(drone_l/2.-drone_l_t)/2.*cos(angle_in));
    fcl::Vector3f rot1(0,angle_in,0.);
    box1.setTranslation(trans1);
    box1.setRotation(setRPY(rot1));

//    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudCluster(new pcl::PointCloud<pcl::PointXYZ>);
//    for (const PtCdtr &cluster : cloudClusters) {
//        pcl::copyPointCloud(*cluster, *cloudCluster);
//        auto octree_point = createCollisionObject(cloudCluster);
//        octree_point.setTranslation(trans2);
//        octree_point.setRotation(setRPY(rot2));
//        // Set Objects
//        group1->registerObject(&octree_point);
//    }
//    group1->setup();
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (!cp_filtered->empty()){
        pcl::copyPointCloud(*cp_filtered, *cloud);
        auto octree_point = createCollisionObject(cloud);

        fcl::Vector3f trans2(0,0,0);
        fcl::Vector3f rot2(0.,0.,0.);
        octree_point.setTranslation(trans2);
        octree_point.setRotation(setRPY(rot2));

        group1->registerObject(&octree_point);
        group1->setup();

        // Contact Number between env and que
        group1->collide(&box1, &c_data1, fcl::DefaultCollisionFunction);
        isCollision = c_data1.result.isCollision();
    }
}
void collision_detection::customize_detector(float angle){

    std::shared_ptr<fcl::CollisionGeometryf> drone_geometry(new fcl::Boxf(drone_w,drone_h,drone_l));
    fcl::CollisionObjectf box1(drone_geometry);
    // DynamicAABBTree BroadPhase Managers
    std::shared_ptr<fcl::BroadPhaseCollisionManager<float>> group1 = std::make_shared<fcl::DynamicAABBTreeCollisionManager<float>>();

    fcl::Vector3f trans1(cameraPosition[0],cameraPosition[1],cameraPosition[2]+drone_l/2.-drone_l_t/2.);
    fcl::Vector3f rot1(0.,angle*C_PI/180,0.);
    box1.setTranslation(trans1);
    box1.setRotation(setRPY(rot1));

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cp_filtered, *cloud);
    auto octree_point = createCollisionObject(cloud);

    fcl::Vector3f trans2(0,0,0);
    fcl::Vector3f rot2(0.,0.,0.);
    octree_point.setTranslation(trans2);
    octree_point.setRotation(setRPY(rot2));

    group1->registerObject(&octree_point);
    group1->setup();


    // Contact Number between env and que
    c_data1.request.gjk_solver_type = fcl::GJKSolverType::GST_INDEP;
    c_data1.result.clear();
    group1->collide(&box1, &c_data1, fcl::DefaultCollisionFunction);
    isCollision = c_data1.result.isCollision(); // 1:collision; 0: no collision


}
collision_detection::~collision_detection(){}

Box collision_detection::BoundingBox(PtCdtr cluster) {

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

int collision_detection::getFlightAngle_H(){
    return ang_h;
}

int collision_detection::getFlightAngle_V(){
    return ang_v;
}

float collision_detection::getFlightDistance(){
    return detec_l;
}
