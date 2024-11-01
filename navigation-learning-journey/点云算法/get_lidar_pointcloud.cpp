#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <pcl/registration/gicp.h>
#include <pcl_ros/transforms.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <thread>

using namespace pcl;
using namespace std;

//实时雷达点云
pcl::PointCloud<pcl::PointXYZ>::Ptr incoming_cloud_(new pcl::PointCloud<pcl::PointXYZ>);
//雷达融合二十次的点云
pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>());
std::mutex cloud_mutex;

//点云接收回调
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input) {
    static int lidar_collect_times,saved_PCD;
    pcl::fromROSMsg(*input, *incoming_cloud_);
    if (incoming_cloud_->empty()) {
        ROS_ERROR("Received empty point cloud!");
        return;
    }
    //存储二十次
    if(lidar_collect_times<=21)
    {
    // 将接收到的点云添加到全局合并后的点云中
    std::lock_guard<std::mutex> lock(cloud_mutex);
    *merged_cloud += *incoming_cloud_;
    lidar_collect_times++;
    ROS_INFO("lidar_collect_times:%d",lidar_collect_times);
    }
    if(!saved_PCD && lidar_collect_times>21) //保存并且进行体素滤波和离群点滤波处理
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
    voxel_grid.setLeafSize(0.03f, 0.03f, 0.03f); // 单位：m
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_voxel(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_grid.filter(*cloud_filtered_voxel);
        // 保存点云数据到PCD文件
    if (pcl::io::savePCDFile<pcl::PointXYZ>("/home/rm/catkin_livox_ros_driver2/src/NEXTE_Sentry_Nav/sentry_slam/FAST_LIO_LOCALIZATION/PCD/my_lidar.pcd", *cloud_filtered_voxel) != -1) {
            ROS_INFO("save PCD!");
    }
        saved_PCD=1;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "get_lidar_PCD_node");
    ros::NodeHandle nh;
    ros::Subscriber pointcloud_sub = nh.subscribe("/cloud_registered", 1, pointCloudCallback);
    ros::spin();
    return 0;
}