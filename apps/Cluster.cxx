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

int saveCluster(char const *);

int main(int argc, char const *argv[])
{
    
    if (argc != 3)
    {
        std::cerr << "./Cluster <dilename>.pcd Cluster.txt" <<  std::endl;
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

    // cal Mat of Distance
    std::cout << "cal Q Mat " << std::endl;
    for (size_t i = 0; i < numFiltered; ++i)
    {
        std::vector<double> tempRows;
        // if (i<30)
        // {
        //     std::cout <<std::endl << "the row: " << i << std::endl;
        // }
        
        for (size_t j = 0; j < numFiltered; ++j)
        {
            double dist = calDistance(cloudFiltered->points[i], cloudFiltered->points[j]);
            tempRows.push_back(dist);
            std::cout << "\r\033[K";
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "Progress: " << static_cast<double>(i*numFiltered+j+1)/(numFiltered*numFiltered)*100 << "%" << std::flush;

            // if (i<30 && j<3)
            // {
            //     std::cout << dist << " ";
            // }
        }
        ::matDist.push_back(tempRows);
    }

    // copy

    // sort
    for (size_t i = 0; i < numFiltered; ++i)
    {
        std::sort(::matDist[i].begin(), ::matDist[i].end());
    }
    
    // cal list of Eps
    for (size_t i = 0; i < numFiltered; ++i)
    {
        double sum = 0;
        for (size_t j = 0; j < numFiltered; ++j)
        {
            sum += ::matDist[j][i];
        }
        ::listEps.push_back(sum/numFiltered);
        
    }
    // show eps
    std::cout << "\nlist length of ::listEps: " << ::listEps.size() << std::endl;
    // for (size_t i = 0; i < numFiltered; ++i)
    // {
    //     std::cout << "listEps: " << ::listEps[i] << std::endl;
    // }

    // cal list of MinPts
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    kdtree->setInputCloud(cloudFiltered);

    // every eps
    std::cout << "\ncal list of MinPts " << std::endl;
    for (size_t i = 0; i < numFiltered; ++i)
    {
        // 在半径内搜索点
        // std::vector<int> tmpPointIdxRadiusSearch;
        // std::vector<float> tmpPointRadiusSquaredDistance;
        std::vector<int> tmpPointIdxRadiusSearch;
        std::vector<float> tmpPointRadiusSquaredDistance;
        double sum = 0;
        // every points
        for (size_t j = 0; j < numFiltered; ++j)
        {
            sum += kdtree->radiusSearch(cloudFiltered->points[j], listEps[i], tmpPointIdxRadiusSearch, tmpPointRadiusSquaredDistance);
            std::cout << "\r\033[K";
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "Progress: " << static_cast<double>(i*numFiltered+j+1)/(numFiltered*numFiltered)*100 << "%" << std::flush;
        }
        listMinPts.push_back(sum/numFiltered);
    }
    // show MinPts
    // std::cout << "list length of ::listMinPts: " << ::listMinPts.size() << std::endl;
    // for (size_t i = 0; i < ::listMinPts.size(); ++i)
    // {
    //     std::cout << "listMinPts: " << ::listMinPts[i] << std::endl;
    // }
    
    // cal Cluster
    int numCurrent = 3;
    int numPrev = 2;
    int numPPrev = 1;
    std::cout << "\ncal num of cluster " << std::endl;
    for (size_t i = 1; i < numFiltered; ++i)
    {
        std::vector<pcl::PointIndices> clusterIndices;
        // clock_t start_ms = clock();
        KdTreeCluster<pcl::PointXYZRGB> ec;
        ec.setCorePointMinPts(listMinPts[i]);
        ec.setClusterTolerance(listEps[i]);
        // ec.setMinClusterSize(500);
        // ec.setMaxClusterSize(25000);
        ec.setSearchMethod(kdtree);
        ec.setInputCloud(cloudFiltered);
        ec.setCorePoints(cloudFiltered);
        ec.extract(clusterIndices);
        // clock_t end_ms = clock();
        int numCurrentCluster = clusterIndices.size();
        ::vNumCluster.push_back(numCurrentCluster);
        if (numCurrent == numPrev && numCurrent == numPPrev)
        {
            std::cout << "index id: " << i << " "
                      << "number of clusters is" << numCurrent << std::endl;
        }else
        {
            numPPrev = numPrev;
            numPrev =  numCurrent;
            numCurrent = numCurrentCluster;
        }
    }
    // pcl::visualization::CloudViewer viewer("cloud");
    return saveCluster(argv[2]);

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

int saveCluster(char const * path)
{
    std::ofstream outFile(path);
    if (!outFile) {
        std::cerr << "Error: Unable to open file." << std::endl;
        return 1;
    }
    // int numAllCluster = ::vNumCluster.size();
    for (size_t i = 0; i < ::vNumCluster.size(); ++i)
    {
        outFile << std::fixed << std::setprecision(3)
                << ::listEps[i+1] << " "
                << ::listMinPts[i+1] << " "
                << ::vNumCluster[i] <<std::endl;
    }
    return 0;
}
