#ifndef HELPER_H
#define HELPER_H

/* Helper functions for processing point-cloud data */
#include <unordered_set>
#include <chrono>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>

namespace Helper {
    template <typename PointT>
    class Line {
        public:
            // constructor 
            Line();

            // de-constructor 
            ~Line();

            // line fitting using SVD method
            std::vector<float> fitSVD(std::vector<float> &x, std::vector<float> &y);

            // line fitting using least-squares method
            std::vector<float> fitLS(std::vector<float> &x, std::vector<float> &y);

            // line to point distance
            float distToPoint(std::vector<float> line_coeffs, PointT point);

            // RANSAC for 2D Points
            std::unordered_set<int> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);
    };

    template <typename PointT>
    class Plane {
        public:
            // constructor 
            Plane();

            // de-constructor 
            ~Plane();

            // plane fitting using SVD method
            std::vector<float> fitSVD(std::vector<float> &x, std::vector<float> &y, std::vector<float> &z);

            // plane fitting using least-squares method
            std::vector<float> fitLS(std::vector<float> &x, std::vector<float> &y, std::vector<float> &z);

            // plane to point distance
            float distToPoint(std::vector<float> plane_coeffs, PointT point);

            // RANSAC for 3D Points
            std::unordered_set<int> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceToPlane);
    };

    // Structure to represent node of kd tree
    struct Node {
        std::vector<float> point;
        int id;
        Node* left;
        Node* right;

        Node(std::vector<float> arr, int setId)
        :	point(arr), id(setId), left(NULL), right(NULL)
        {}
    };

    // k-d tree struct
    struct KdTree {
        // root of the tree
        Node* root;

        // default constructor
        KdTree() : root(NULL) {}

        // insert method
        void insert(std::vector<float> point, int id);

        // search function
        std::vector<int> search(std::vector<float> target, float distTolerance);

        // distance between two points
        float dist(std::vector<float> point_a, std::vector<float> point_b);

        // helper function for insert
        void insertHelper(Node ** node, unsigned depth, std::vector<float> point, int id);

        // helper function for search
        void searchHelper(Node * node, std::vector<float> target, float distTolerance, int depth, std::vector<int>& ids);
    };

    class EuclideanCluster {
        public:
            // constructor
            EuclideanCluster(const std::vector<std::vector<float>>& points, KdTree * tree)
            :   points(points), tree(tree)
            {}

            // clustering function
            std::vector<std::vector<int>> clustering(float distTolerance);

        private:
            // vector of 3d points
            const std::vector<std::vector<float>> points;

            // k-d tree
            KdTree * tree;

            // find points in the proximity of the point indexed 'pointIndex'
            void proximityPoints( int pointIndex,
						            std::vector<bool>& checked,
						            float distTolerance, 
						            std::vector<int>& cluster);
    };
}

#endif /* HELPER_H */