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
#include <mutex>
#include <chrono>
#include <tf2_ros/transform_broadcaster.h>
#include <ctime>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>

using namespace pcl;
using namespace std;

// 全局变量
//先验地图点云
pcl::PointCloud<pcl::PointXYZ>::Ptr prior_map_(new pcl::PointCloud<pcl::PointXYZ>);
//先验地图旋转地图
pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_lidar_cloud_(new pcl::PointCloud<pcl::PointXYZ>);
//实时雷达点云
pcl::PointCloud<pcl::PointXYZ>::Ptr incoming_cloud_(new pcl::PointCloud<pcl::PointXYZ>);
//存储后读取的雷达点云
pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//转换之后的点云
pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new PointCloud<pcl::PointXYZ>);
//雷达融合二十次的点云
pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>());

std::mutex cloud_mutex;

sensor_msgs::PointCloud2 prior_map_msg;
sensor_msgs::PointCloud2 rotated_lidar_map_msg;
sensor_msgs::PointCloud2 incoming_cloud_msg;
sensor_msgs::PointCloud2 transformed_cloud_msg; //转换之后的点云msg
sensor_msgs::PointCloud2 cloud_removed_msg;
//gicp对象
pcl::GeneralizedIterativeClosestPoint<PointXYZ, PointXYZ> gicp;
//sicp对象
pcl::GeneralizedIterativeClosestPoint<PointXYZ, PointXYZ> icp;

std::mutex global_mutex;

Eigen::Matrix4f initial_pose = Eigen::Matrix4f::Identity();

float best_yaw_angle_ = 0.0; // 存储最佳旋转角度
bool need_relocalization=false; //如果需要进行重定位，则为true
bool start_find;
bool find_times;
static bool transformed;
ros::Publisher pub_prior_map;
ros::Publisher pub_rotated_lidar_cloud;
ros::Publisher pub_transformed_cloud;
ros::Publisher pub_incoming_cloud;
ros::Publisher pub_icp;
ros::Publisher map_to_odom_pub;
ros::Publisher removal_pointcloud_publisher_;
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input);
void timerCallback(const ros::TimerEvent& event);
void performRelocalization(const Eigen::Matrix4f& initial_pose); //在定时器回调中进行的icp匹配
void publishTransform(const Eigen::Matrix4f& transform); //发布矩阵变换
void findBestYawAngle(); //寻找最优角度
void pub_map_to_camera_init();
void PointCloudObstacleRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_map_msg, 
                              pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_msg, 
                              double Distance_Threshold);

template<typename PointT>
pcl::PointCloud<pcl::PointXYZ>::Ptr FilterPointsByDistance(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
    float distance_threshold);
