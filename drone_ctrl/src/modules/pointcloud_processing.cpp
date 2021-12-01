//
// Created by caleb on 12.05.21.
//

#include "pointcloud_processing.h"

#define PI (double)3.141592653589793
static const std::string FRAME_ID = "camera_depth_optical_frame";
//static const std::string FRAME_ID = "drone_rotated";
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT>::Ptr PtCdtr;
const int color_bar[][3] =
        {
                { 192,192,192},
                { 225,225,225 },
                { 255,69,0 },
                { 30,144,255 },
                { 255,255,0 },
                { 127,255,212 },
                { 160,32,240 }
        };

pointcloud_processing::pointcloud_processing(int timeRate_in,boost::shared_ptr<pcl::visualization::PCLVisualizer>& pc_viewer)
        : nh(), point_sub(), gps_sub()
{
    viewer = pc_viewer;
     imuPub = nh.advertise<sensor_msgs::Imu>("imu", 10);
     imu_sub = nh.subscribe("dji_sdk/imu", 10, &pointcloud_processing::imuCallback,this);
     modelPub = nh.advertise<visualization_msgs::MarkerArray>("model_marker", 10);
     boxPub = nh.advertise<visualization_msgs::MarkerArray>("box_marker", 10);
     direct_sub = nh.subscribe("direct_info",10, &pointcloud_processing::directCallback,this);
     directPub = nh.advertise<visualization_msgs::MarkerArray>("direct_marker", 10);
     velocity_sub = nh.subscribe("dji_sdk/velocity", 10, &pointcloud_processing::velocityCallback,this);
     locPub = nh.advertise<visualization_msgs::MarkerArray>("loc_marker", 10);
     pcPub = nh.advertise<sensor_msgs::PointCloud2> ("point_cloud", 1);
    // gps_sub = nh.subscribe("dji_sdk/gps_position", 1, &pointcloud_processing::gpsCallback,this);

//    point_sub = nh.subscribe("/drone/front_camera/depth/points", 10,
//                             &pointcloud_processing::pointCloudCallback, this);//for sim
    point_sub = nh.subscribe("/camera/depth/color/points", 1,
                             &pointcloud_processing::pointCloudCallback, this);// for real
    timeRate = timeRate_in;
    inputCloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

}


pointcloud_processing::~pointcloud_processing(){}

void pointcloud_processing::publish_drone(){
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker_d;
    ////drone model
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker_d.header.frame_id = FRAME_ID;
    marker_d.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker_d.ns = "drone_model";
    marker_d.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker_d.type = visualization_msgs::Marker::MESH_RESOURCE;;
    marker_d.mesh_resource = "package://drone_ctrl/src/modules/m600.dae";
//    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker_d.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker_d.pose.position.x = 0;
    marker_d.pose.position.y = 0;
    marker_d.pose.position.z = -0.6;
    marker_d.pose.orientation.x = 0.0;
    marker_d.pose.orientation.y = 0.0;
    marker_d.pose.orientation.z = 0.0;
    marker_d.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker_d.scale.x = 1.0;
    marker_d.scale.y = 1.0;
    marker_d.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker_d.color.r = 1.0f;
    marker_d.color.g = 1.0f;
    marker_d.color.b = 1.0f;
    marker_d.color.a = 1.0;

    marker_d.lifetime = ros::Duration();
    marker_array.markers.push_back(marker_d);

    ////imu arrow
    visualization_msgs::Marker marker_i;
    marker_i.header.frame_id = FRAME_ID;
    marker_i.header.stamp = ros::Time::now();

    marker_i.ns = "imu_arrow";
    marker_i.id = 1;
    marker_i.type = visualization_msgs::Marker::ARROW;
    marker_i.action = visualization_msgs::Marker::ADD;

    geometry_msgs::Point p1,p2;
    p1.x=p1.y=p1.z=0;
    p2.x=imu.linear_acceleration.x*20;
    p2.y=imu.linear_acceleration.y*20;
    p2.z=0;
    marker_i.pose.orientation.w = 1.0;

    marker_i.scale.x = 0.1;
    marker_i.scale.y = 0.1;
    marker_i.scale.z = 0.0;

    marker_i.color.r = 1.0f;
    marker_i.color.g = 0.0f;
    marker_i.color.b = 1.0f;
    marker_i.color.a = 1.0;

    marker_i.lifetime = ros::Duration();
    marker_i.points.push_back(p1);
    marker_i.points.push_back(p2);
    marker_array.markers.push_back(marker_i);

    ////drone deteciton Box

    ///droneBox for collision detection
    Eigen::Vector3f uav_point(-0.85, -0.3, -0.8);
    float length =  10 - 2 * uav_point(2);;
    float width = -2 * uav_point(0);
    float height = -2 * uav_point(1);

    Box droneBox = createBox(uav_point, width, length, height, cameraPosition);

    visualization_msgs::Marker marker_de;
    marker_de.header.frame_id = FRAME_ID;
    marker_de.header.stamp = ros::Time::now();

    marker_de.ns = "drone_box";
    marker_de.id = 2;
    marker_de.type = visualization_msgs::Marker::LINE_LIST;
    marker_de.action = visualization_msgs::Marker::ADD;

    marker_de.pose.orientation.w = 1.0;
    marker_de.scale.x = 0.1;

    marker_de.color.r = 1.0f;
    marker_de.color.g = 1.0f;
    marker_de.color.b = 0.0f;
    marker_de.color.a = 1.0;

    marker_de.lifetime = ros::Duration();
    geometry_msgs::Point p[8];
    p[0].x=droneBox.z_min;
    p[0].y=-droneBox.x_min;
    p[0].z=-droneBox.y_max;

    p[1].x=droneBox.z_min;
    p[1].y=-droneBox.x_max;
    p[1].z=-droneBox.y_max;

    p[2].x=droneBox.z_max;
    p[2].y=-droneBox.x_max;
    p[2].z=-droneBox.y_max;

    p[3].x=droneBox.z_max;
    p[3].y=-droneBox.x_min;
    p[3].z=-droneBox.y_max;

    p[4].x=droneBox.z_min;
    p[4].y=-droneBox.x_min;
    p[4].z=-droneBox.y_min;

    p[5].x=droneBox.z_min;
    p[5].y=-droneBox.x_max;
    p[5].z=-droneBox.y_min;

    p[6].x=droneBox.z_max;
    p[6].y=-droneBox.x_max;
    p[6].z=-droneBox.y_min;

    p[7].x=droneBox.z_max;
    p[7].y=-droneBox.x_min;
    p[7].z=-droneBox.y_min;

    int line[] {0,1,1,2,2,3,3,0,4,5,5,6,6,7,7,4,0,4,1,5,2,6,3,7};
    int len = sizeof(line)/ sizeof (line[0]);
    for (int i=0;i<len;i++){
        marker_de.points.push_back(p[line[i]]);
    }
    marker_array.markers.push_back(marker_de);

    modelPub.publish(marker_array);
}
void pointcloud_processing::publish_box(){
    visualization_msgs::MarkerArray marker_array;
    int id = 0;
    if (cloudClusters.size()<clusters_size){
        for (int n = 0;n<clusters_size-cloudClusters.size();n++){
            visualization_msgs::Marker marker;
            marker.header.frame_id = FRAME_ID;
            marker.header.stamp = ros::Time::now();
            marker.ns = "obj_box";
            marker.id = n+cloudClusters.size();
            marker.action = visualization_msgs::Marker::DELETEALL;
            marker_array.markers.push_back(marker);
        }
    }
    boxPub.publish(marker_array);
    if (!cloudClusters.empty())
    {
        for (const PtCdtr &cluster : cloudClusters){
            visualization_msgs::Marker marker;
            marker.header.frame_id = FRAME_ID;
            marker.header.stamp = ros::Time::now();

            marker.ns = "obj_box";
            marker.id = id; id++;
            marker.type = visualization_msgs::Marker::LINE_LIST;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.1;

            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.7f;
            marker.color.a = 1.0;

            marker.lifetime = ros::Duration(1.0/timeRate);
            Box box = BoundingBox(cluster);
            geometry_msgs::Point p[8];
            p[0].x=box.z_min;
            p[0].y=-box.x_min;
            p[0].z=-box.y_max;

            p[1].x=box.z_min;
            p[1].y=-box.x_max;
            p[1].z=-box.y_max;

            p[2].x=box.z_max;
            p[2].y=-box.x_max;
            p[2].z=-box.y_max;

            p[3].x=box.z_max;
            p[3].y=-box.x_min;
            p[3].z=-box.y_max;

            p[4].x=box.z_min;
            p[4].y=-box.x_min;
            p[4].z=-box.y_min;

            p[5].x=box.z_min;
            p[5].y=-box.x_max;
            p[5].z=-box.y_min;

            p[6].x=box.z_max;
            p[6].y=-box.x_max;
            p[6].z=-box.y_min;

            p[7].x=box.z_max;
            p[7].y=-box.x_min;
            p[7].z=-box.y_min;

            int line[] {0,1,1,2,2,3,3,0,4,5,5,6,6,7,7,4,0,4,1,5,2,6,3,7,1,3,0,2,4,6,5,7};
            int length = sizeof(line)/ sizeof (line[0]);
            for (int i=0;i<length;i++){
                marker.points.push_back(p[line[i]]);
            }
            marker_array.markers.push_back(marker);
        }
        boxPub.publish(marker_array);
    }
    clusters_size = cloudClusters.size();
}

void pointcloud_processing::publish_loc(){
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    marker.header.frame_id = FRAME_ID;
    marker.header.stamp = ros::Time::now();

    marker.ns = "loc";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();

    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    int len = sizeof(loc)/ sizeof (loc[0]);
//    cout<<loc[90].x<<endl;
    for (int i = 0; i<len; i++) {
        marker.points.push_back(loc[i]);
    }
    marker_array.markers.push_back(marker);
    locPub.publish(marker_array);
}

void pointcloud_processing::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    imu = *msg;
    imu.header.frame_id = FRAME_ID;
    imu.header.stamp = ros::Time::now();
    imuPub.publish(imu);
}

void pointcloud_processing::location(){
    int len = sizeof(loc)/ sizeof (loc[0]);
    if (imu_state){
        double displacement_x = 1.0/timeRate*v.x;
        double displacement_y = 1.0/timeRate*v.y;
        double displacement_z = 1.0/timeRate*v.z;
        double yaw_change = toEulerAngle(imu.orientation).z-toEulerAngle(pre_imu.orientation).z;
//        cout<<v.x<<" "<<v.z<<" "<<yaw_change<<endl;
        for (int i=0; i<len;i++){
            double x0,y0,z0,x1,y1,z1;
            x0 = loc[i].x;
            y0 = loc[i].y;
            z0 = loc[i].z;
            x1 = x0 * cos(yaw_change)+y0* sin(yaw_change) - displacement_x;
            y1 = -x0 * sin(yaw_change)+y0* cos(yaw_change)- displacement_y;
            z1 = z0 - displacement_z;
            loc[i].x = x1;
            loc[i].y = y1;
            loc[i].z = z1;
        }

        for(int i=len-1; i > 0; i--){
            loc[i] = loc[i-1];
        }
    }
//    cout<<loc[90].x<<endl;
    loc[0].x = loc[0].y = loc[0].z = 0;
    pre_imu = imu;
    if (!imu_state) imu_state =true;
}
double pointcloud_processing::getLength(const double *v) {
    return (sqrt(pow(v[0],2)+pow(v[1],2)));
}

geometry_msgs::Vector3 pointcloud_processing::toEulerAngle(geometry_msgs::Quaternion quat)
{
    geometry_msgs::Vector3 ans;

    tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
    R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
    return ans;
}

void pointcloud_processing::velocityCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
    v=msg->vector;
}

void pointcloud_processing::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
    pcl::fromROSMsg (*cloud_msg, inputCloud);
    inputCloud_ptr = inputCloud.makeShared();
//    if (!inputCloud.points.empty()) {cout<<"get point cloud!"<<endl;}
//    else {cout<<"no point cloud!"<<endl;}
}
void pointcloud_processing::processStep(){

    ////Initialize Parameter
    if (!inputCloud.points.empty()){


         publish_drone();
         location();
         publish_loc();


        //// 0th:Calibration
        Eigen::Matrix4f ts_1(4, 4), ts_2(4, 4);
        ts_1 << 0.06100477,-0.15948637,0.98571972,0.20833718,
                -0.99978308,-0.01784935,0.0302663,0.08271715,
                0.01276229,-0.98695716,-0.16047643,-0.20692879,
                0,0,0,1;
        ts_2 << 0, -1, 0, 0,
                0, 0, -1, 0,
                1, 0, 0, 0,
                0, 0, 0, 1;
//        ts_1 << 1, 0, 0, 0,
//                0, 1, 0, 0,
//                0, 0, 1, 0,
//                0, 0, 0, 1;
//        ts_2 << 1, 0, 0, 0,
//                0, 1, 0, 0,
//                0, 0, 1, 0,
//                0, 0, 0, 1;

        PtCdtr inputCloud_ts_1, inputCloud_ts_2;
        inputCloud_ts_1.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        inputCloud_ts_2.reset(new pcl::PointCloud<pcl::PointXYZRGB>());


        //// 1st:Filter cloud to reduce amount of points
        PtCdtr filteredCloud = FilterCloud(inputCloud_ptr, filterRes);
        pcl::transformPointCloud (*filteredCloud, *inputCloud_ts_1, ts_1);
        pcl::transformPointCloud (*inputCloud_ts_1, *inputCloud_ts_2, ts_2);
//        viewer->addPointCloud<PointT> (inputCloud_ptr, "cloud_origin");
//        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud_origin");
//        viewer->addPointCloud<PointT> (inputCloud_ts_2, "cloud_ts");
//        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud_ts");
//        PtCdtr filteredCloud = FilterCloud(inputCloud_ptr, filterRes);
//        cp_filtered = filteredCloud;
        //// 2nd: Segment the filtered cloud into obstacles and road
        std::pair<PtCdtr, PtCdtr> segmentCloud = RansacSegmentPlane(inputCloud_ts_2, maxIterations, distanceThreshold);
        renderPointCloud(viewer, segmentCloud.first, "planeCloud");
        PtCdtr filteredCloud_1 = FilterCloud_1(segmentCloud.second,k_mean,thres_stat);
        cp_filtered.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        *cp_filtered = *segmentCloud.first + *segmentCloud.second;
        


        // Convert to ROS data type
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*inputCloud_ts_1, output);
        output.header.frame_id = FRAME_ID;

        // Publish the data
        pcPub.publish (output);
        //// 3rd: Cluster different obstacle cloud
        cloudClusters = EuclidCluster(filteredCloud_1, clusterTolerance, minsize, maxsize);
        renderCluster(viewer,cloudClusters);
        publish_box();


    }
}

PtCdtr pointcloud_processing::FilterCloud(PtCdtr cloud, float filterRes){
    auto startTime = std::chrono::steady_clock::now();

    PtCdtr cloud_filtered(new pcl::PointCloud<PointT>);
    PtCdtr cloud_condi(new pcl::PointCloud<PointT>);

    pcl::ConditionAnd<PointT>::Ptr range_cond(new pcl::ConditionAnd<PointT>);
    // 添加比较对象
    range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>(
            "z", pcl::ComparisonOps::GT, 0.0)));
    range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>(
            "z", pcl::ComparisonOps::LE, thres_z)));
    range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>(
            "x", pcl::ComparisonOps::GT, -thres_x)));
    range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>(
            "x", pcl::ComparisonOps::LE, thres_x)));
    range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>(
            "y", pcl::ComparisonOps::GT, -thres_y)));
    range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>(
            "y", pcl::ComparisonOps::LE, thres_y)));
    pcl::ConditionalRemoval<PointT> condRem;
    condRem.setInputCloud(cloud);
    // 设置过滤条件
    condRem.setCondition(range_cond);
    // true：将条件之外的点还保留，但是设置NAN, 或者setUserFilterValue修改
    condRem.setKeepOrganized(false);
    condRem.filter(*cloud_condi);

    //VoxelGrid filter
    pcl::VoxelGrid<PointT> vox;
    vox.setInputCloud(cloud_condi);
    vox.setLeafSize(filterRes,filterRes,filterRes);//体素网格大小
    vox.filter(*cloud_filtered);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//    ROS_INFO_STREAM("before filter :"<<cloud->points.size());
//    ROS_INFO_STREAM("after filter :" << cloud_filtered->points.size());
    ROS_INFO_STREAM( "filtering took " << elapsedTime.count() << " ms");

    return cloud_filtered;
}
PtCdtr pointcloud_processing::FilterCloud_1(PtCdtr cloud, int k_mean, float thres){
    PtCdtr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (k_mean);
    sor.setStddevMulThresh (thres);
    sor.filter (*cloud_filtered);
    return cloud_filtered;
}
std::pair<PtCdtr, PtCdtr>
pointcloud_processing::RansacSegmentPlane(PtCdtr cloud, int maxIterations, float distanceTol){
    auto startTime = std::chrono::steady_clock::now();
    PtCdtr out_plane(new pcl::PointCloud<PointT>());
    PtCdtr in_plane(new pcl::PointCloud<PointT>());
    PtCdtr cloud_condi_1(new pcl::PointCloud<PointT>);
    PtCdtr cloud_condi_2(new pcl::PointCloud<PointT>);

    pcl::ConditionAnd<PointT>::Ptr range_cond(new pcl::ConditionAnd<PointT>);
    range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>(
            "y", pcl::ComparisonOps::GT, h_seg)));
    range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>(
            "y", pcl::ComparisonOps::LE, thres_y)));
    pcl::ConditionalRemoval<PointT> condRem;
    condRem.setInputCloud(cloud);
    // 设置过滤条件
    condRem.setCondition(range_cond);
    // true：将条件之外的点还保留，但是设置NAN, 或者setUserFilterValue修改
    condRem.setKeepOrganized(false);
    condRem.filter(*cloud_condi_1);
    pcl::ConditionAnd<PointT>::Ptr range_cond_2(new pcl::ConditionAnd<PointT>);
    range_cond_2->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>(
            "y", pcl::ComparisonOps::GT, -thres_y)));
    range_cond_2->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>(
            "y", pcl::ComparisonOps::LE, h_seg)));
    condRem.setCondition(range_cond_2);
    condRem.filter(*cloud_condi_2);
    // Set Objects
    //创建分割时所需要的模型系数对象，coefficients及存储内点的点索引集合对象inliers
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // 创建分割对象
    pcl::SACSegmentation<PointT> seg;
    // 可选择配置，设置模型系数需要优化
    seg.setOptimizeCoefficients(true);
    // 必要的配置，设置分割的模型类型，所用的随机参数估计方法，距离阀值，输入点云
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    // you can modify the parameter below
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceTol);
    seg.setInputCloud(cloud_condi_1);
    //引发分割实现，存储分割结果到点几何inliers及存储平面模型的系数coefficients
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
        cout<<"error! Could not found any inliers!"<<endl;
    }
    // extract ground
    pcl::ExtractIndices<PointT> extractor;
    extractor.setInputCloud(cloud_condi_1);
    extractor.setIndices(inliers);
    extractor.setNegative(true);
    extractor.filter(*out_plane);
    extractor.setNegative(false);
    extractor.filter(*in_plane);
    // vise-versa, remove the ground not just extract the ground
    // just setNegative to be true

    *out_plane = *out_plane + *cloud_condi_2;
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    ROS_INFO_STREAM( "Ground removal took " << elapsedTime.count() << " ms");

    return std::pair<PtCdtr, PtCdtr>(in_plane, out_plane);
}

std::vector<PtCdtr>
pointcloud_processing::EuclidCluster(PtCdtr cloud, float clusterTolerance, int minSize,int maxSize){
    auto startTime = std::chrono::steady_clock::now();
    // Search Method: KdTree
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    // Euclidean Cluster
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance); // 设置空间聚类误差1m
    ec.setMinClusterSize(minSize);  // 设置有效聚类包含的最小的个数
    ec.setMaxClusterSize(maxSize);  // 设置有效聚类包含的最大的个数
    ec.setSearchMethod(tree);  // 设置搜索方法
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);  // 获取切割之后的聚类索引保存到cluster_indices
    std::vector<PtCdtr> clusters;
    // For each cluster indices
    for (pcl::PointIndices getIndices: cluster_indices){
        PtCdtr cloud_cluster(new pcl::PointCloud<PointT>);
        // For each point indices in each cluster
        for (int index:getIndices.indices){
            cloud_cluster->points.push_back(cloud->points[index]);}
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
        cout<<cloud_cluster->size()<<endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    ROS_INFO_STREAM( "Clustering took " << elapsedTime.count() << " ms and found " << clusters.size() << " clusters");
    return clusters;
}

void pointcloud_processing::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    if (gps_state == 0) { init_gps = *msg; gps_state = 1;}
    current_gps = *msg;
}
void pointcloud_processing::renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const  PtCdtr& cloud, std::string name){
    viewer->removePointCloud(name);
    viewer->addPointCloud<PointT> (cloud, name);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0.5, 0, name);
}
Box pointcloud_processing::BoundingBox(PtCdtr cluster) {

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

Box pointcloud_processing::createBox(Eigen::Vector3f uav_point,
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

void pointcloud_processing::renderCluster(pcl::visualization::PCLVisualizer::Ptr& viewer, std::vector<PtCdtr> clusters){
    int clusterId = 0;
    for (const PtCdtr &cluster : clusters) {
    viewer->removePointCloud("cluster" + std::to_string(clusterId));
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(cluster,
                                                                              color_bar[clusterId][0],
                                                                              color_bar[clusterId][1],
                                                                              color_bar[clusterId][2]);//赋予显示点云的颜色
    viewer->addPointCloud<PointT>(cluster, cloud_in_color_h, "cluster" + std::to_string(clusterId));
    clusterId++;
    }
}

void pointcloud_processing::directCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    if (!msg->data.empty())
    {
        minDis = msg->data.at(0);
        detec_l = msg->data.at(1);
        direct = msg->data.at(2);
        ang = msg->data.at(3);
    }
}
void pointcloud_processing::publish_decide(){
   visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    marker.header.frame_id = FRAME_ID;
    marker.header.stamp = ros::Time::now();

    marker.ns = "direct_line";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;

    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    geometry_msgs::Point p[3];
    float d_safe = 2;
    p[0].x= minDis - d_safe;
    p[0].y= 0;
    p[0].z= 0;

    p[1].x= minDis;
    p[1].y=-d_safe * tan(ang/180.*PI);
    p[1].z= 0;

    p[2].x= minDis + (d_safe + 1) + 1.6/2.0;
    p[2].y=-d_safe * tan(ang/180.*PI);
    p[2].z= 0;

    int line[] {0,1,1,2};
    int length = sizeof(line)/ sizeof (line[0]);
    for (int i=0;i<length;i++){
        marker.points.push_back(p[line[i]]);
    }
    marker_array.markers.push_back(marker);
    directPub.publish(marker_array);
}



