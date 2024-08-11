#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>

#include <cmath>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <fstream>

#include "KdTree.hpp"

std::vector<double> listEps;
std::vector<double> listMinPts;
std::vector<std::vector<double>> matDist;
std::vector<int> vNumCluster;

template<typename T>
double calDistance(T const & p1, T const & p2);
template <typename PointCloudPtr>
void ShowPointCloud(const PointCloudPtr);
// void ShowPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr );

struct Color {
    int r, g, b;

    // 构造函数
    Color(int red, int green, int blue) : r(red), g(green), b(blue) {}
};

std::vector<Color> generateDistinctiveColors(unsigned long long);

int saveCluster(char const *);



int main(int argc, char const *argv[])
{
    
    if (argc != 3)
    {
        std::cerr << "./Cluster <filename>.pcd <output>.pcd " <<  std::endl;
        return 1;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = std::make_shared< pcl::PointCloud<pcl::PointXYZRGB> >();
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // std::string path = "../../data/fr3-w-x.pcd"; 
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud) == -1)
    {
        PCL_ERROR(argv[1]);
        return (-1);
    }
    std::cout << "number of raw points " << cloud->width * cloud->height << std::endl;
    
    // filter Raw cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFiltered = std::make_shared< pcl::PointCloud<pcl::PointXYZRGB> >();
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(cloud);
    // vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.setLeafSize(0.08f, 0.08f, 0.08f);
    vg.filter(*cloudFiltered);
    size_t numFiltered = cloudFiltered->points.size();
    std::cout << "number of filtered points " <<  numFiltered << std::endl;

    // KdTree
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    kdtree->setInputCloud(cloudFiltered);

    std::vector<pcl::PointIndices> clusterIndices;
    // clock_t start_ms = clock();
    KdTreeCluster<pcl::PointXYZRGB> ec;
    ec.setCorePointMinPts(15);
    ec.setClusterTolerance(0.124);
    // ec.setMinClusterSize(500);
    // ec.setMaxClusterSize(25000);
    ec.setSearchMethod(kdtree);
    ec.setInputCloud(cloudFiltered);
    ec.setCorePoints(cloudFiltered);
    ec.extract(clusterIndices);
    // clock_t end_ms = clock();
    std::cout << "num of cluster: " << clusterIndices.size() <<  std::endl;
    // int numCurrentCluster = clusterIndices.size();
    // ::vNumCluster.push_back(numCurrentCluster);
    
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudClustered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudClustered(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<Color> colors = ::generateDistinctiveColors(clusterIndices.size());
    int j = 1;
    // visualization, use indensity to show different color for each cluster.
    for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); it++, ++j) {
        for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            pcl::PointXYZRGB tmp;
            // pcl::PointXYZRGB tmp;
            tmp.x = cloudFiltered->points[*pit].x;
            tmp.y = cloudFiltered->points[*pit].y;
            tmp.z = cloudFiltered->points[*pit].z;
            tmp.r = colors[j].r;
            tmp.g = colors[j].g;
            tmp.b = colors[j].b;
            // tmp.intensity = j;
            cloudClustered->points.push_back(tmp);
        }
    }
    cloudClustered->width = cloudClustered->points.size();
    cloudClustered->height = 1;
    ::ShowPointCloud(cloudClustered);
    pcl::io::savePCDFile(argv[2], *cloudClustered);

    // pcl::visualization::CloudViewer viewer("cloud");
    // return saveCluster(argv[2]);

    return 0;
}

template<typename T>
double calDistance(T const & p1, T const & p2)
{
    double dx = (p1.x-p2.x);
    double dy = (p1.y-p2.y);
    double dz = (p1.z-p2.z);
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}


template <typename PointCloudPtr>
void ShowPointCloud(const PointCloudPtr cloud)
{
    // pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
    // viewer->setBackgroundColor(1.0, 1.0, 1.0);
    // pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> fildColor(cloud,"intensity");
    // viewer->addPointCloud<pcl::PointXYZI>(cloud, fildColor, "sample");
    // while (!viewer->wasStopped()){}

    pcl::visualization::CloudViewer viewer("cloud");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped()){}

}

std::vector<Color> generateDistinctiveColors(unsigned long long numColors) {
    std::vector<Color> colors;
    int numSegments = static_cast<int>(pow(numColors, 1.0 / 3.0));

    for (int r = 0; r < numSegments; ++r) {
        for (int g = 0; g < numSegments; ++g) {
            for (int b = 0; b < numSegments; ++b) {
                if (colors.size() >= numColors) break;
                int red = 255 * r / (numSegments - 1);
                int green = 255 * g / (numSegments - 1);
                int blue = 255 * b / (numSegments - 1);
                colors.emplace_back(red, green, blue);
            }
        }
    }
    return colors;
}
