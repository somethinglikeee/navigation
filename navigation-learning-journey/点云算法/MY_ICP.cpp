#include <MY_ICP.h>


void findBestYawAngle_thread() {
    try {
        std::thread relocalizationThread(findBestYawAngle);
        relocalizationThread.detach(); // 让线程独立运行
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to create thread: %s", e.what());
    }
}

void publish_map_msg()
{
    // Publish the point clouds
    pcl::toROSMsg(*prior_map_, prior_map_msg);
    pcl::toROSMsg(*incoming_cloud_, incoming_cloud_msg);
    pcl::toROSMsg(*rotated_lidar_cloud_, rotated_lidar_map_msg);
    prior_map_msg.header.frame_id = "map";
    incoming_cloud_msg.header.frame_id = "map";
    rotated_lidar_map_msg.header.frame_id = "map";
    prior_map_msg.header.stamp = ros::Time::now();
    rotated_lidar_map_msg.header.stamp = ros::Time::now();
    incoming_cloud_msg.header.stamp = ros::Time::now();
    pub_prior_map.publish(prior_map_msg);
    pub_incoming_cloud.publish(incoming_cloud_msg); //发布实时雷达点云
}

//启动一个线程来发布地图点云消息
void publish_map_msg_thread()
{
    using namespace std::chrono_literals;
    while (ros::ok()) {
        publish_map_msg();
        // 休眠0.2秒
        std::this_thread::sleep_for(0.2s);
    }
}

// 函数，用于滤除与先验点云距离过远的点
pcl::PointCloud<pcl::PointXYZ>::Ptr FilterPointsByDistance(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
    float distance_threshold) {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(prior_map_);

    for (const auto &point : input_cloud->points) {
        std::vector<int> nearest_indices(1);
        std::vector<float> nearest_squared_distances(1);

        // 进行最近邻搜索
        if (kdtree.nearestKSearch(point, 1, nearest_indices, nearest_squared_distances) > 0) {
            // 如果距离小于阈值，则保留该点
            if (nearest_squared_distances[0] <= (distance_threshold * distance_threshold)) {
                filtered_cloud->points.push_back(point);
            }
        }
    }

    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;

    return filtered_cloud;
}

//点云接收回调
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input) {
    static int lidar_collect_times,saved_PCD;
    using namespace std::chrono_literals;

    if(!transformed)
    pub_map_to_camera_init();

    pcl::fromROSMsg(*input, *incoming_cloud_);
    PointCloudObstacleRemoval(prior_map_,incoming_cloud_,1.0);

    // // 创建VoxelGrid滤波器对象
    // pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    // voxel_grid.setInputCloud(incoming_cloud_);
    // voxel_grid.setLeafSize(0.03f, 0.03f, 0.03f); // 单位：m
    // voxel_grid.filter(*incoming_cloud_);

    if (incoming_cloud_->empty()) {
        ROS_ERROR("Received empty point cloud!");
        return;
    }
    //存储二十次
    if(lidar_collect_times<=20)
    {
    // 将接收到的点云添加到全局合并后的点云中
    std::lock_guard<std::mutex> lock(cloud_mutex);
    *merged_cloud += *incoming_cloud_;
    lidar_collect_times++;
    ROS_INFO("lidar_collect_times:%d",lidar_collect_times);
    }
    if(!saved_PCD && lidar_collect_times>20) //保存并且进行体素滤波和离群点滤波处理
    {
    // 创建StatisticalOutlierRemoval滤波器对象
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(merged_cloud);
    sor.setMeanK(30); // 设置每个点的邻近点数
    sor.setStddevMulThresh(1.0); // 设置标准偏差乘数阈值
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_sor(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*cloud_filtered_sor);

    // 创建VoxelGrid滤波器对象
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud_filtered_sor);
    voxel_grid.setLeafSize(0.05f, 0.05f, 0.05f); // 单位：m
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_voxel(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_grid.filter(*cloud_filtered_voxel);


        // 保存点云数据到PCD文件
    if (pcl::io::savePCDFile<pcl::PointXYZ>("/home/rm/catkin_livox_ros_driver2/src/NEXTE_Sentry_Nav/sentry_slam/FAST_LIO_LOCALIZATION/PCD/my_lidar.pcd", *cloud_filtered_voxel) != -1) {
            ROS_INFO("save PCD!");
    }
        saved_PCD=1;
    }

    if(saved_PCD)
    {
    if(!need_relocalization && !start_find)//创建单独寻优线程 仅在初始化时使用寻优 或者丢失定位时进行重定位时使用
    {
        start_find=true;
        if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/rm/catkin_livox_ros_driver2/src/NEXTE_Sentry_Nav/sentry_slam/FAST_LIO_LOCALIZATION/PCD/my_lidar.pcd", *lidar_cloud)!=-1)
        ROS_INFO("get lidar PCD!");   
        findBestYawAngle_thread();
    }   

    if(need_relocalization) //旋转雷达点云到最佳角度
    performRelocalization(initial_pose);

    }
}

void performRelocalization(const Eigen::Matrix4f& initial_pose){
    static Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    double tranDist,angleDist;
    static float last_score;
    static bool icp_publish;
    pcl::toROSMsg(*prior_map_, prior_map_msg);
    rotated_lidar_map_msg.header.frame_id = "map";
    pub_rotated_lidar_cloud.publish(rotated_lidar_map_msg); 
    PointCloud<PointXYZ> aligned;

    icp.setInputSource(incoming_cloud_);
    icp.setInputTarget(prior_map_);
    icp.setEuclideanFitnessEpsilon(1e-5);	// 设置收敛条件是均方误差和小于阈值，停止迭代;
    icp.setMaximumIterations(40);			// 最大迭代次数

    gicp.setInputSource(incoming_cloud_);
    gicp.setInputTarget(prior_map_);
    gicp.setEuclideanFitnessEpsilon(1e-5);	// 设置收敛条件是均方误差和小于阈值，停止迭代;
    gicp.setMaximumIterations(40);			// 最大迭代次数

    Eigen::Matrix4f current_transform = 
    transform.isApprox(Eigen::Matrix4f::Identity()) ? initial_pose : transform;

    ros::Time start_time=ros::Time::now();
    gicp.align(aligned, current_transform); // 使用初始位姿
    ros::Time end_time=ros::Time::now();
    double duration = (end_time - start_time).toSec();  // 计算持续时间，单位为秒
    ROS_INFO("time:%f",duration);
    if (gicp.hasConverged()) {
        ROS_INFO("ICP converged with score: %f", gicp.getFitnessScore());

        transform = gicp.getFinalTransformation(); // 获取变换矩阵

        if(gicp.getFitnessScore()>0.5 && transformed && last_score > 0.5)
        {
            need_relocalization=true;
            start_find=false;
            find_times=true;
        }
        last_score = gicp.getFitnessScore();

        // if(gicp.getFitnessScore()<0.008)
        publishTransform(transform);

        pcl::toROSMsg(*transformed_cloud, transformed_cloud_msg);
        transformed_cloud_msg.header.frame_id = "map";
        pub_transformed_cloud.publish(transformed_cloud_msg); 

        Eigen::Affine3f transform_3f = Eigen::Affine3f::Identity();
        transform_3f = gicp.getFinalTransformation(); // 获取变换矩阵

        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transform_3f, x, y, z, roll, pitch, yaw);
        double tranDist = sqrt(x*x + y*y);
        double angleDist = abs(yaw);
        if(!transformed)
        {
            pcl::transformPointCloud(*incoming_cloud_, *transformed_cloud, transform); // 应用变换
            transformed=true;
        }

        // if(tranDist>0.10 && transformed)
        pcl::transformPointCloud(*incoming_cloud_, *transformed_cloud, transform); // 应用变换
        ROS_INFO("tranDist:%f angleDist:%f",tranDist,angleDist);

    } else {
        ROS_ERROR("ICP did not converge.");
    }
}

void publishTransform(const Eigen::Matrix4f& transform) {
    // 创建一个TransformStamped消息
    geometry_msgs::TransformStamped transformStamped;

    // 设置时间戳
    transformStamped.header.stamp = ros::Time::now();
    // 设置父坐标帧ID
    transformStamped.header.frame_id = "map";
    // 设置子坐标帧ID
    transformStamped.child_frame_id = "camera_init";

    // 将Eigen::Matrix4f转换为geometry_msgs::Vector3和geometry_msgs::Quaternion
    Eigen::Affine3f eigen_affine(transform);
    transformStamped.transform.translation.x = eigen_affine.translation()(0);
    transformStamped.transform.translation.y = eigen_affine.translation()(1);
    transformStamped.transform.translation.z = eigen_affine.translation()(2);

    Eigen::Quaternionf quat(eigen_affine.rotation());
    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();

    // 发布变换
    map_to_odom_pub.publish(transformStamped);
    ROS_INFO("Published transform");
    // 打印变换信息
    ROS_INFO("Transform from %s to %s:", transformStamped.header.frame_id.c_str(), transformStamped.child_frame_id.c_str());
    ROS_INFO("Translation: x = %f, y = %f, z = %f", transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
    ROS_INFO("Rotation: x = %f, y = %f, z = %f, w = %f", transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
}

void findBestYawAngle(){
    static float last_score;
    float find_min_angle;
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp_yaw;
    float best_score = std::numeric_limits<float>::max();
    float best_yaw = 0.0;
    ROS_INFO("begin find");

    if(!find_times)
    find_min_angle=9;
    else
    find_min_angle=18;
    
    for (float yaw = -M_PI; yaw <= M_PI; yaw += M_PI / find_min_angle) {
        Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
        Eigen::Matrix3f rotation_matrix = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
        transformation.block<3,3>(0,0) = rotation_matrix;
        pcl::transformPointCloud(*lidar_cloud, *rotated_lidar_cloud_, transformation);
        PointCloud<PointXYZ> aligned;
        gicp_yaw.setInputSource(rotated_lidar_cloud_);
        gicp_yaw.setInputTarget(prior_map_);
        gicp_yaw.setEuclideanFitnessEpsilon(1e-5);	// 设置收敛条件是均方误差和小于阈值，停止迭代;
        gicp_yaw.setMaximumIterations(20);
        gicp_yaw.align(aligned);
        ROS_INFO("yaw:%f with score: %f",yaw,gicp_yaw.getFitnessScore());
        if (gicp_yaw.hasConverged()) {
            float score = gicp_yaw.getFitnessScore();
            if (score < best_score) {
                best_score = score;
                best_yaw = yaw;
            }

            if(score-last_score>0.01)
            {
                yaw += M_PI / find_min_angle;
            }
            last_score=score;
        }
        if(gicp_yaw.getFitnessScore()>0.02)//这里如果效果不好直接开始跳跃角度提升速度
            yaw += M_PI / find_min_angle * 2;
        if(gicp_yaw.getFitnessScore()>0.01)//小幅度跳跃
            yaw += M_PI / find_min_angle;
         // 将旋转后的先验地图点云转换为ROS消息
        pcl::toROSMsg(*rotated_lidar_cloud_, rotated_lidar_map_msg);
        rotated_lidar_map_msg.header.frame_id = "map";
        rotated_lidar_map_msg.header.stamp = ros::Time::now();

        // 发布旋转后的先验地图点云
        pub_rotated_lidar_cloud.publish(rotated_lidar_map_msg);
    }

    ROS_INFO("Best yaw angle: %f with score: %f", best_yaw, best_score);

    best_yaw_angle_ = best_yaw;

    // 设置初始位姿为 (0, 0, 0)  角度为最佳角度
    initial_pose.block<3, 3>(0, 0) = Eigen::AngleAxisf(best_yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    initial_pose.block<3, 1>(0, 3) = Eigen::Vector3f(0, 0, 0); // 设置位置为 (0, 0, 0)
    need_relocalization=true;
}


/**
 * 对点云中障碍点进行剔除
 * cloud_map_msg为参考点云，cloud_msg为需要剔除障碍点的点云
 */
void PointCloudObstacleRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_map_msg, 
                              pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_msg, 
                              double Distance_Threshold) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_removed(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud_map_msg);

    int K = 1;  // 1-nearest neighbor search
    for (size_t i = 0; i < cloud_msg->points.size(); ++i) {
        pcl::PointXYZ searchPoint = cloud_msg->points[i];
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            if (pointNKNSquaredDistance[0] > Distance_Threshold) {  // If distance is greater than threshold, remove the point
                cloud_msg->erase(cloud_msg->begin() + i);
                -i;  // Decrement i because the size of the cloud has changed
                cloud_removed->push_back(searchPoint);  // Optionally store the removed point
            }
        }
    }

    cloud_removed->width = cloud_removed->points.size();
    cloud_removed->height = 1;
    cloud_removed->is_dense = false;  // contains nans

    pcl::toROSMsg(*cloud_removed, cloud_removed_msg);
    cloud_removed_msg.header.frame_id = "map";
    cloud_removed_msg.header.stamp = ros::Time::now();
    // Assuming you have a publisher defined as `removal_pointcloud_publisher_`, publish the cloud_removed
    // pcl::toROSMsg(*cloud_removed, cloud_removed_msg);
    removal_pointcloud_publisher_.publish(cloud_removed_msg);
}

//防止报错的，在未进行icp之前发布空变换
void pub_map_to_camera_init() {
    // 创建一个TransformStamped消息
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "camera_init";

    // 设置变换的平移和旋转部分
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0;
    transformStamped.transform.rotation.x = 0;
    transformStamped.transform.rotation.y = 0;
    transformStamped.transform.rotation.z = 0;
    transformStamped.transform.rotation.w = 1; // 单位四元数，表示没有旋转

    // 发布变换
    map_to_odom_pub.publish(transformStamped);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "icp_node");
    ros::NodeHandle nh;

    if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/rm/catkin_livox_ros_driver2/src/NEXTE_Sentry_Nav/sentry_slam/FAST_LIO_LOCALIZATION/PCD/demo.pcd", *prior_map_)!=-1)
    ROS_INFO("get prior PCD!");   
    ROS_INFO("kdtree initialized!");   
    pub_prior_map = nh.advertise<sensor_msgs::PointCloud2>("icp_prior_map", 1);
    pub_rotated_lidar_cloud = nh.advertise<sensor_msgs::PointCloud2>("icp_rotated_lidar_map", 1);
    pub_incoming_cloud = nh.advertise<sensor_msgs::PointCloud2>("incoming_cloud", 1);
    pub_transformed_cloud = nh.advertise<sensor_msgs::PointCloud2>("transformed_cloud", 1);
    ros::Subscriber pointcloud_sub = nh.subscribe("/cloud_registered", 1, pointCloudCallback);
    map_to_odom_pub = nh.advertise<geometry_msgs::TransformStamped>("map_to_odom", 10);
    removal_pointcloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("removed_cloud", 1);
    // 启动发布线程
    std::thread publishThread(publish_map_msg_thread);

    ros::spin();
    return 0;
}


