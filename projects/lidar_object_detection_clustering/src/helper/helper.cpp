#include "helper.h"
#include <Eigen/Dense>

namespace Helper {
	// constructor 
	template <typename PointT>
	Line<PointT>::Line() {}

	// de-constructor 
	template <typename PointT>
	Line<PointT>::~Line() {}

	template <typename PointT>
	std::vector<float> Line<PointT>::fitSVD(std::vector<float> &x, std::vector<float> &y)
	{
		/*
		System of linear equations of the form Ax = 0
		
		SVD method solves the equation Ax = 0 by performing
		singular-value decomposition of matrix A
		*/

		// Form Matrix A
		Eigen::MatrixXd A(x.size(), 3);
		for(int i = 0; i < x.size(); ++i)
		{
			A(i, 0) = x[i];
			A(i, 1) = y[i];
			A(i, 2) = 1.0;
		}
			
		// Take SVD of A
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::DecompositionOptions::ComputeThinU | Eigen::DecompositionOptions::ComputeThinV);
		Eigen::MatrixXd V = svd.matrixV();
		
		// store in vector
		std::vector<float> line_coeffs;
		for(auto i = 0; i < V.rows(); ++i)
		{
			line_coeffs.push_back(V(i, (V.cols() - 1)));
		}
		
		// Return coeffs
		return line_coeffs;
	}

	template <typename PointT>
	std::vector<float> Line<PointT>::fitLS(std::vector<float> &x, std::vector<float> &y)
	{
		/*
		System of linear equations of the form Y = Hx
		
		least-squares method attempts to minimize
		the energy of error, J(x) = ( ||Y - Hx|| )^2
		where, ||Y - Hx|| is the Euclidian length of vector (Y - Hx)
		*/
		
		// Form Matrix H
		Eigen::MatrixXd H(x.size(), 2);
		for(int i = 0; i < x.size(); ++i)
		{
			H(i, 0) = x[i];
			H(i, 1) = 1.0;
		}

		// Form Matrix Y
		Eigen::MatrixXd Y(y.size(), 1);
		for(int i = 0; i < y.size(); ++i)
		{
			Y(i, 0) = y[i];
		}

		// Transpose of H
		auto H_transpose = H.transpose();

		// get line coefficients
		auto coeffs = ((H_transpose*H).inverse()) * (H_transpose*Y);

		// Line equation is of the form y = a'x + b'
		// let's convert it to the form ax + by + c = 0
		// a = a'; b = -1; c = b'
		std::vector<float> line_coeffs;
		line_coeffs.push_back(coeffs(0, 0));
		line_coeffs.push_back(-1.0);
		line_coeffs.push_back(coeffs(1, 0));

		// Return coefficients
		return line_coeffs;
	}

	template <typename PointT>
	float Line<PointT>::distToPoint(std::vector<float> line_coeffs, PointT point)
	{
		float dist = fabs(line_coeffs[0] * point.x + line_coeffs[1] * point.y + line_coeffs[2]) /
						sqrt(pow(line_coeffs[0], 2) + pow(line_coeffs[1], 2));
		
		return dist;
	}

	template <typename PointT>
	std::unordered_set<int> Line<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distTolerance)
	{
		// random number seed
		srand(time(NULL));
		// get the start timestamp
		auto t_start = std::chrono::high_resolution_clock::now();

		std::unordered_set<int> inliersResult;
		
		// number of random samples to select per iteration
		const int n_random_samples = 2;
		
		// number of inliers for each iteration
		std::vector<int> n_inliers(maxIterations, 0);
		// coefficients for each line
		std::vector<std::vector<float>> coeffs(maxIterations);

		// iterate 'maxIterations' number of times
		for(int i = 0; i < maxIterations; ++i)
		{
			// x, y, and z points as a vector
			std::vector<float> x, y, z;
			// select random samples
			for(int j = 0; j < n_random_samples; ++j)
			{
				int idx = rand()%cloud->size();
				x.push_back(cloud->at(idx).x);
				y.push_back(cloud->at(idx).y);
				z.push_back(cloud->at(idx).z);
			}
			// fit a line
			coeffs[i] = this->fitSVD(x, y);

			for(typename pcl::PointCloud<PointT>::iterator it = cloud->begin(); it != cloud->end(); ++it)
			{
				if(this->distToPoint(coeffs[i], *it) <= distTolerance)
				{
					n_inliers[i]++;
				}
			}
		}

		// find the index for number of inliers
		auto inliers_it = std::max_element(n_inliers.begin(), n_inliers.end());
		int index_max_n_inlier = std::distance(n_inliers.begin(), inliers_it);

		// find inliers with the best fit
		int index = 0;
		for(typename pcl::PointCloud<PointT>::iterator it = cloud->begin(); it != cloud->end(); ++it)
		{
			if(this->distToPoint(coeffs[index_max_n_inlier], *it) <= distTolerance)
			{
				inliersResult.insert(index);
			}
			index++;
		}

		// get the end timestamp
		auto t_end = std::chrono::high_resolution_clock::now();

		// measure execution time
		auto t_duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start);
		std::cout << "Time taken by RANSAC: "
			<< t_duration.count() << " milliseconds" << std::endl; 

		// Return indicies of inliers from fitted line with most inliers
		return inliersResult;
	}

	// constructor 
	template <typename PointT>
	Plane<PointT>::Plane() {}

	// de-constructor 
	template <typename PointT>
	Plane<PointT>::~Plane() {}

	template <typename PointT>
	std::vector<float> Plane<PointT>::fitSVD(std::vector<float> &x, std::vector<float> &y, std::vector<float> &z)
	{
		/*
		System of linear equations of the form Ax = 0
		
		SVD method solves the equation Ax = 0 by performing
		singular-value decomposition of matrix A
		*/
		
		// Form Matrix A
		Eigen::MatrixXd A(x.size(), 4);
		for(int i = 0; i < x.size(); ++i)
		{
			A(i, 0) = x[i];
			A(i, 1) = y[i];
			A(i, 2) = z[i];
			A(i, 3) = 1.0;
		}
			
		// Take SVD of A
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::DecompositionOptions::ComputeThinU | Eigen::DecompositionOptions::ComputeThinV);
		Eigen::MatrixXd V = svd.matrixV();

		// Plane equation is of the form ax + by + cz + d = 0
		std::vector<float> plane_coeffs;
		for(auto i = 0; i < V.rows(); ++i)
		{
			plane_coeffs.push_back(V(i, (V.cols() - 1)));
		}
		
		// Return coeffs
		return plane_coeffs;
	}

	template <typename PointT>
	std::vector<float> Plane<PointT>::fitLS(std::vector<float> &x, std::vector<float> &y, std::vector<float> &z)
	{
		/*
		System of linear equations of the form Y = Hx
		
		least-squares method attempts to minimize
		the energy of error, J(x) = ( ||Y - Hx|| )^2
		where, ||Y - Hx|| is the Euclidian length of vector (Y - Hx)
		*/
		
		// Form Matrix H
		Eigen::MatrixXd H(x.size(), 3);
		for(int i = 0; i < x.size(); ++i)
		{
			H(i, 0) = x[i];
			H(i, 1) = y[i];
			H(i, 2) = 1.0;
		}

		// Form Matrix Y
		Eigen::MatrixXd Y(z.size(), 1);
		for(int i = 0; i < z.size(); ++i)
		{
			Y(i, 0) = z[i];
		}

		// Transpose of H
		auto H_transpose = H.transpose();

		// get plane coefficients
		auto coeffs = ((H_transpose*H).inverse()) * (H_transpose*Y);

		// Plane equation is of the form z = a'x + b'y + c'
		// let's convert it to the form ax + by + cz + d = 0
		// a = a'; b = b'; c = -1; d = c'
		std::vector<float> plane_coeffs;
		plane_coeffs.push_back(coeffs(0, 0));
		plane_coeffs.push_back(coeffs(1, 0));
		plane_coeffs.push_back(-1.0);
		plane_coeffs.push_back(coeffs(2, 0));

		// Return coefficients
		return plane_coeffs;
	}

	template <typename PointT>
	float Plane<PointT>::distToPoint(std::vector<float> plane_coeffs, PointT point)
	{
		float dist = fabs(plane_coeffs[0] * point.x + plane_coeffs[1] * point.y + plane_coeffs[2] * point.z + plane_coeffs[3]) /
						sqrt(pow(plane_coeffs[0], 2) + pow(plane_coeffs[1], 2) + pow(plane_coeffs[2], 2));
		
		return dist;
	}

	template <typename PointT>
	std::unordered_set<int> Plane<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceToPlane)
	{
		// random number seed
		srand(time(NULL));
		// get the start timestamp
		auto t_start = std::chrono::high_resolution_clock::now();

		std::unordered_set<int> inliersResult;
		
		// number of random samples to select per iteration
		const int n_random_samples = 3;
		
		// number of inliers for each iteration
		std::vector<int> n_inliers(maxIterations, 0);
		// coefficients for each plane
		std::vector<std::vector<float>> coeffs(maxIterations);

		// iterate 'maxIterations' number of times
		for(int i = 0; i < maxIterations; ++i)
		{
			// x, y, and z points as a vector
			std::vector<float> x, y, z;
			// select random samples
			for(int j = 0; j < n_random_samples; ++j)
			{
				int idx = rand()%(cloud->size());
				x.push_back(cloud->at(idx).x);
				y.push_back(cloud->at(idx).y);
				z.push_back(cloud->at(idx).z);
			}
			// fit a plane
			coeffs[i] = this->fitLS(x, y, z);

			for(typename pcl::PointCloud<PointT>::iterator it = cloud->begin(); it != cloud->end(); ++it)
			{
				// std::cout << "dist = " << this->distToPoint(coeffs[i], *it) << std::endl;
				if(this->distToPoint(coeffs[i], *it) <= distanceToPlane)
				{
					n_inliers[i]++;
				}
			}
		}

		// find the index for number of inliers
		auto inliers_it = std::max_element(n_inliers.begin(), n_inliers.end());
		int index_max_n_inlier = std::distance(n_inliers.begin(), inliers_it);

		// find inliers with the best fit
		int index = 0;
		for(typename pcl::PointCloud<PointT>::iterator it = cloud->begin(); it != cloud->end(); ++it)
		{
			if(this->distToPoint(coeffs[index_max_n_inlier], *it) <= distanceToPlane)
			{
				inliersResult.insert(index);
			}
			index++;
		}

		// get the end timestamp
		auto t_end = std::chrono::high_resolution_clock::now();

		// measure execution time
		auto t_duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start);
		std::cout << "Time taken by RANSAC: "
			<< t_duration.count() << " milliseconds" << std::endl; 

		// Return indicies of inliers from fitted line with most inliers
		return inliersResult;
	}

	void KdTree::insertHelper(Node ** node, unsigned depth, std::vector<float> point, int id)
	{
		if(*node != NULL) {
			// x split when (depth % 3) = 0; y split when (depth % 3) = 1; z split when (depth % 3) = 2
			// index for accessing point.x is 0; index for accessing point.y is 1; index for accessing point.z is 2
			if(point[(depth % 3)] < ((*node)->point[(depth % 3)])) {
				node = &((*node)->left);
			}
			else {
				node = &((*node)->right);
			}

			// call this function recursively until a NULL is hit
			insertHelper(node, depth+1, point, id);
		}
		else {
			// create a node and insert the point
			*node = new Node(point, id);
		}
	}

	void KdTree::insert(std::vector<float> point, int id)
	{
		// This function inserts a new point into the tree
		// the function creates a new node and places correctly with in the root 
		insertHelper(&this->root, 0, point, id);		
	}

	// this function returns euclidian distance between two points
	float KdTree::dist(std::vector<float> point_a, std::vector<float> point_b)
	{
		// compute distance
		float dist = sqrt(pow((point_a[0] - point_b[0]), 2) 
						+ pow((point_a[1] - point_b[1]), 2)
						+ pow((point_a[2] - point_b[2]), 2));

		// Return the euclidian distance between points
		return dist;
	}

	void KdTree::searchHelper(Node * node, std::vector<float> target, float distTolerance, int depth, std::vector<int>& ids)
	{
		if(node != NULL) {
			// add this node id to the list if its distance from target is less than distTolerance
			if(dist(node->point, target) <= distTolerance) 
				ids.push_back(node->id);	

			// x split when (depth % 3) = 0; y split when (depth % 3) = 1; z split when (depth % 3) = 2
			// index for accessing point.x is 0; index for accessing point.y is 1; index for accessing point.z is 2
			if((target[depth % 3] - distTolerance) < node->point[(depth % 3)])
				searchHelper(node->left, target, distTolerance, depth+1, ids);
			if((target[depth % 3] + distTolerance) > node->point[(depth % 3)])
				searchHelper(node->right, target, distTolerance, depth+1, ids);
		}
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> KdTree::search(std::vector<float> target, float distTolerance)
	{
		std::vector<int> ids;
		searchHelper(this->root, target, distTolerance, 0, ids);
		return ids;
	}

	void EuclideanCluster::proximityPoints( int pointIndex,
							std::vector<bool>& checked,
							float distTolerance, 
							std::vector<int>& cluster) 
	{
		std::vector<int> nearby = this->tree->search(this->points[pointIndex], distTolerance);
		for(auto it = nearby.begin(); it != nearby.end(); ++it) {
			if(! checked[*it]) {
				checked[*it] = true;
				cluster.push_back(*it);
				// call this function recursively to find all the points within proximity (i.e. points within proximity of proximity)
				proximityPoints(*it, checked, distTolerance, cluster);
			}
		}
	}

	std::vector<std::vector<int>> EuclideanCluster::clustering(float distTolerance)
	{
		std::vector<std::vector<int>> clusters;

		// vector to keep track of checked points
		std::vector<bool> checked(points.size(), false);
		for(int i = 0; i < this->points.size(); ++i) {
			// create a new cluster if this point was not processed already
			if(! checked[i]) {
				std::vector<int> cluster;
				checked[i] = true;
				cluster.push_back(i);
				// find points within the proximity
				proximityPoints(i, checked, distTolerance, cluster);
				// add this cluster to the vector of clusters
				clusters.push_back(cluster);
			}
		}
	
		return clusters;
	}
}

// explicit instantiation to avoid linker error
template class Helper::Line<pcl::PointXYZ>;
template class Helper::Line<pcl::PointXYZI>;
template class Helper::Plane<pcl::PointXYZ>;
template class Helper::Plane<pcl::PointXYZI>;