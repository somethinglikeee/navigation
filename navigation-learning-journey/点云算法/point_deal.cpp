#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

int main(int argc, char** argv) {
    // 加载点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/rm/catkin_livox_ros_driver2/src/NEXTE_Sentry_Nav/sentry_slam/FAST_LIO/PCD/scans.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read file scans.pcd \n");
        return (-1);
    }
    std::cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from scans.pcd with the following fields: "
              << std::endl;
    for (std::size_t i = 0; i < cloud->points.size(); ++i)
        std::cout << "    " << cloud->points[i].x
                  << " "    << cloud->points[i].y
                  << " "    << cloud->points[i].z << std::endl;

    // 创建StatisticalOutlierRemoval滤波器对象
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(40); // 设置每个点的邻近点数
    sor.setStddevMulThresh(1.0); // 设置标准偏差乘数阈值
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_sor(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*cloud_filtered_sor);

    // 创建VoxelGrid滤波器对象
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud_filtered_sor);
    voxel_grid.setLeafSize(0.03f, 0.03f, 0.03f); // 单位：m
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_voxel(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_grid.filter(*cloud_filtered_voxel);

    // 保存处理后的点云数据
    if (pcl::io::savePCDFile<pcl::PointXYZ>("/home/rm/catkin_livox_ros_driver2/src/NEXTE_Sentry_Nav/sentry_slam/FAST_LIO/PCD/demo.pcd", *cloud_filtered_voxel) == -1) {
        PCL_ERROR("Couldn't write file demo.pcd \n");
        return (-1);
    }
    pcl::io::savePCDFile<pcl::PointXYZ>("/home/rm/catkin_livox_ros_driver2/src/NEXTE_Sentry_Nav/sentry_slam/FAST_LIO_LOCALIZATION/PCD/demo.pcd", *cloud_filtered_voxel);
    std::cout << "Saved "
              << cloud_filtered_voxel->width * cloud_filtered_voxel->height
              << " data points to demo.pcd." << std::endl;
    return 0;
}
