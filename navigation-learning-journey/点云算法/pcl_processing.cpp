#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

// 全局变量
ros::Publisher pub;
ros::Subscriber sub;
ros::Time last_time;
ros::Time current_time;
int count = 0; // 计数器
ros::Time last_second_time; // 上一次计算频率的时间

// 回调函数
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud_msg) {
    // 更新当前时间
    current_time = ros::Time::now();

    // 累加计数器
    count++;

    // 如果已经过了1秒，计算频率并重置计数器
    if (current_time - last_second_time > ros::Duration(1.0)) {
        double frequency = count; // 频率为每秒计数器的值
        ROS_WARN_STREAM("Processing frequency: " << frequency << " Hz");
        last_second_time = current_time; // 更新上一次计算频率的时间
        count = 0; // 重置计数器
    }

    // 将sensor_msgs::PointCloud2转换为pcl::PCLPointCloud2
    pcl::PCLPointCloud2 cloud;
    pcl_conversions::toPCL(*input_cloud_msg, cloud);

    // 创建VoxelGrid滤波器对象
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_grid;
    voxel_grid.setInputCloud(boost::make_shared<pcl::PCLPointCloud2>(cloud));
    voxel_grid.setLeafSize(0.05f, 0.05f, 0.05f); // 单位：m
    pcl::PCLPointCloud2 cloud_filtered;
    voxel_grid.filter(cloud_filtered);

    // 创建StatisticalOutlierRemoval滤波器对象
    pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(boost::make_shared<pcl::PCLPointCloud2>(cloud_filtered));
    sor.setMeanK(50); // 设置每个点的邻近点数
    sor.setStddevMulThresh(1.0); // 设置标准偏差乘数阈值
    pcl::PCLPointCloud2 cloud_filtered_sor;
    sor.filter(cloud_filtered_sor);

    // 将pcl::PCLPointCloud2转换回sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 output_cloud_msg;
    pcl_conversions::fromPCL(cloud_filtered_sor, output_cloud_msg);
    output_cloud_msg.header = input_cloud_msg->header;

    // 发布处理后的点云
    pub.publish(output_cloud_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl_processing_node");
    ros::NodeHandle nh;

    // 创建Subscriber来订阅原始点云
    sub = nh.subscribe("cloud_registered", 1, cloudCallback);

    // 创建Publisher来发布处理后的点云
    pub = nh.advertise<sensor_msgs::PointCloud2>("processed_cloud", 1);

    last_second_time = ros::Time::now(); // 初始化上一次计算频率的时间

    ros::spin();
    return 0;
}