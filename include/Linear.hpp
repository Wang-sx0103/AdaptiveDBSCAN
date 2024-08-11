#if !defined(LINEAR_H)
#define LINEAR_H

#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/search/pcl_search.h>

#define UN_PROCESSED 0
#define PROCESSING 1
#define PROCESSED 2

inline bool comparePointClusters (const pcl::PointIndices &a, const pcl::PointIndices &b) {
    return (a.indices.size () < b.indices.size ());
}

template <typename PointT>
class LinearCluster {
public:
    typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
    typedef typename pcl::search::KdTree<PointT>::Ptr KdTreePtr;
    virtual void setInputCloud(PointCloudPtr cloud)
    {
        this->input_cloud_ = cloud;
    }

    void setSearchMethod(KdTreePtr tree)
    {
        this->search_method_ = tree;
    }

    /// @brief 
    /// @param clusterIndices 
    virtual void extract(std::vector<pcl::PointIndices>& clusterIndices)
    {
        std::vector<int> nn_indices;
        std::vector<float> nn_distances;
        std::vector<bool> is_noise(this->input_cloud_->points.size(), false);
        std::vector<int> types(this->input_cloud_->points.size(), UN_PROCESSED);
        for (int i = 0; i < static_cast<int>(this->input_cloud_->points.size()); i++)
        {
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
                int cloud_index = seed_queue[sq_idx];
                if (is_noise[cloud_index] || types[cloud_index] == PROCESSED)
                {
                    // seed_queue.push_back(cloud_index);
                    types[cloud_index] = PROCESSED;
                    sq_idx++;
                    continue; // no need to check neighbors.
                }
                nn_size = myRadiusSearch(cloud_index, this->eps_, nn_indices, nn_distances);
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
                
                types[cloud_index] = PROCESSED;
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

                pointIds.header = input_cloud_->header;
                clusterIndices.push_back(pointIds);   // We could avoid a copy by working directly in the vector
            }
        } // for every point in input cloud
        std::sort(clusterIndices.rbegin(), clusterIndices.rend(), comparePointClusters);
    }

    void setClusterTolerance(double tolerance)
    {
        this->eps_ = tolerance; 
    }

    void setMinClusterSize (int min_cluster_size)
    { 
        this->min_pts_per_cluster_ = min_cluster_size; 
    }

    void setMaxClusterSize (int max_cluster_size)
    {
        this->max_pts_per_cluster_ = max_cluster_size; 
    }
    
    void setCorePointMinPts(int core_point_min_pts)
    {
        this->minPts_ = core_point_min_pts;
    }

protected:
    PointCloudPtr input_cloud_;
    
    // search radius
    double eps_ {0.0};
    // min num of point in field, not including the point itself.
    int minPts_ {1};
    int min_pts_per_cluster_ {1};
    int max_pts_per_cluster_ {std::numeric_limits<int>::max()};

    // search through K-d tree 
    KdTreePtr search_method_;

    /// @brief Determine the points around point 'indes' that satisfy a distance threshold(radius),
    /// Save points that satisfy the threshold to the vector(k_indices)
    /// @param index 
    /// @param radius 
    /// @param k_indices Index of points that satisfy the threshold
    /// @param k_sqr_distances Distance from point 'index'
    /// @return 
    virtual int myRadiusSearch(int index, double radius, std::vector<int> &k_indices,
                               std::vector<float> &k_sqr_distances) const
    {
        k_indices.clear();
        k_sqr_distances.clear();
        k_indices.push_back(index);
        k_sqr_distances.push_back(0);
        int size = this->input_cloud_->points.size();
        double radius_square = radius * radius;
        for (int i = 0; i < size; i++)
        {
            if (i == index) continue;
            double distance_x = this->input_cloud_->points[i].x - this->input_cloud_->points[index].x;
            double distance_y = this->input_cloud_->points[i].y - this->input_cloud_->points[index].y;
            double distance_z = this->input_cloud_->points[i].z - this->input_cloud_->points[index].z;
            double distance_square = distance_x * distance_x + distance_y * distance_y + distance_z * distance_z;
            if (distance_square <= radius_square)
            {
                k_indices.push_back(i);
                k_sqr_distances.push_back(std::sqrt(distance_square));
            }
        }
        return k_indices.size();
    }
}; // class LinearCluster

#endif // LINEAR_H
