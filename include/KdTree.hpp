#if !defined(KDTREE_H)
#define KDTREE_H

#include <pcl/point_types.h>

#include <iostream>

#include "Linear.hpp"


template <typename PointT>
class KdTreeCluster: public LinearCluster<PointT>{
public:
    typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
    /// @brief 
    /// @param clusterIndices 
    void extract(std::vector<pcl::PointIndices>& clusterIndices) override
    {
        std::vector<int> nn_indices;
        std::vector<float> nn_distances;
        std::vector<bool> is_noise(this->input_cloud_->points.size(), false);
        std::vector<int> types(this->input_cloud_->points.size(), UN_PROCESSED);

        this->getIndices();

        for (int idx = 0; idx < static_cast<int>(this->cloudCoreIndices.size()); ++idx)
        {
            int i = this->cloudCoreIndices[idx];
            if (types[i] == PROCESSED)
            {
                continue;
            }
            int nn_size = myRadiusSearch(i, this->eps_, nn_indices, nn_distances);
            if (nn_size < this->minPts_)
            {
                is_noise[i] = true;
                continue;
            }
            
            std::vector<int> seed_queue;
            seed_queue.push_back(i);
            types[i] = PROCESSED;
            
            for (int j = 0; j < nn_size; j++)
            {
                if (nn_indices[j] != i)
                {
                    seed_queue.push_back(nn_indices[j]);
                    types[nn_indices[j]] = PROCESSING;
                }
            } // for every point near the chosen core point.
            int sq_idx = 1;
            while (sq_idx < static_cast<int>(seed_queue.size()))
            {
                int point_index = seed_queue[sq_idx];
                if (is_noise[point_index] || types[point_index] == PROCESSED)
                {
                    // seed_queue.push_back(cloud_index);
                    types[point_index] = PROCESSED;
                    sq_idx++;
                    continue; // no need to check neighbors.
                }
                nn_size = myRadiusSearch(point_index, this->eps_, nn_indices, nn_distances);
                if (nn_size >= this->minPts_)
                {
                    for (int j = 0; j < nn_size; j++)
                    {
                        if (types[nn_indices[j]] == UN_PROCESSED)
                        {
                            seed_queue.push_back(nn_indices[j]);
                            types[nn_indices[j]] = PROCESSING;
                        }
                    }
                }
                
                types[point_index] = PROCESSED;
                sq_idx++;
            }
            if (static_cast<int>(seed_queue.size()) >= this->min_pts_per_cluster_ &&
                static_cast<int>(seed_queue.size()) <= this->max_pts_per_cluster_)
            {
                pcl::PointIndices pointIds;
                pointIds.indices.resize(seed_queue.size());
                for (int j = 0; j < static_cast<int>(seed_queue.size()); ++j)
                {
                    pointIds.indices[j] = seed_queue[j];
                }
                // These two lines should not be needed: (can anyone confirm?) -FF
                std::sort(pointIds.indices.begin(), pointIds.indices.end());
                pointIds.indices.erase(std::unique(pointIds.indices.begin(), pointIds.indices.end()), pointIds.indices.end());

                pointIds.header = this->input_cloud_->header;
                clusterIndices.push_back(pointIds);   // We could avoid a copy by working directly in the vector
            }
        } // for every point in input cloud
        std::sort(clusterIndices.rbegin(), clusterIndices.rend(), comparePointClusters);
    }

    void setCorePoints(PointCloudPtr core)
    {
        this->cloudCore = core;
    }

protected:
    int myRadiusSearch(int index, double radius, std::vector<int> &k_indices,
                       std::vector<float> &k_sqr_distances) const override
    {
        return this->search_method_->radiusSearch(index, radius, k_indices, k_sqr_distances);
    }
    void getIndices()
    {
        for (int i = 0; i < static_cast<int>(this->cloudCore->points.size()); i++)
        {
            std::vector<int> currPointIndices;
            std::vector<float> currPointDistances;
            this->search_method_->nearestKSearch(this->cloudCore->points[i], 1, currPointIndices, currPointDistances);
            // std::cout << i << "= " << currPointIndices[0] << std::endl;
            this->cloudCoreIndices.push_back(currPointIndices[0]);
        }
    }

protected:
    PointCloudPtr cloudCore;
    std::vector<int> cloudCoreIndices;
    // std::vector<float> cloudCoreDistance;

}; // class DBSCANCluster


#endif // KDTREE_H
