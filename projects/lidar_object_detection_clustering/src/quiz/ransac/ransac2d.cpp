/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
// Eigen library
#include <Eigen/Dense>

using Eigen::MatrixXd;

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
		// rand() / RAND_MAX ensures that the random number generated is between 0 and 1
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);	// -1 to +1
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);	// -1 to +1
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 1;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 1;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::vector<float> fitLine(std::vector<float> &x, std::vector<float> &y)
{
    // Form Matrix A
	MatrixXd A(x.size(), 3);
	for(int i = 0; i < x.size(); ++i)
	{
		A(i, 0) = x[i];
		A(i, 1) = y[i];
		A(i, 2) = 1.0;
	}
        
    // Take SVD of A
	Eigen::JacobiSVD<MatrixXd> svd(A, Eigen::DecompositionOptions::ComputeThinU | Eigen::DecompositionOptions::ComputeThinV);
	MatrixXd V = svd.matrixV();
    
	// store in vector
	std::vector<float> line_coeffs;
	for(auto i = 0; i < V.rows(); ++i)
	{
		line_coeffs.push_back(V(i, (V.cols() - 1)));
	}
    
	// Return coeffs
	return line_coeffs;
}

float lineToPointDist(std::vector<float> line_coeffs, pcl::PointXYZ point)
{
	float dist = fabs(line_coeffs[0] * point.x + line_coeffs[1] * point.y + line_coeffs[2]) /
					sqrt(pow(line_coeffs[0], 2) + pow(line_coeffs[1], 2));
	
	return dist;
}

std::vector<float> fitPlane(std::vector<float> &x, std::vector<float> &y, std::vector<float> &z)
{
    // Form Matrix A
	MatrixXd A(x.size(), 4);
	for(int i = 0; i < x.size(); ++i)
	{
		A(i, 0) = x[i];
		A(i, 1) = y[i];
		A(i, 2) = z[i];
		A(i, 3) = 1.0;
	}
        
    // Take SVD of A
	Eigen::JacobiSVD<MatrixXd> svd(A, Eigen::DecompositionOptions::ComputeThinU | Eigen::DecompositionOptions::ComputeThinV);
	MatrixXd V = svd.matrixV();

	// Plane equation is of the form ax + by + cz + d = 0
	std::vector<float> plane_coeffs;
	for(auto i = 0; i < V.rows(); ++i)
	{
		plane_coeffs.push_back(V(i, (V.cols() - 1)));
	}
    
	// Return coeffs
	return plane_coeffs;
}

float planeToPointDist(std::vector<float> plane_coeffs, pcl::PointXYZ point)
{
	float dist = fabs(plane_coeffs[0] * point.x + plane_coeffs[1] * point.y + plane_coeffs[2] * point.z + plane_coeffs[3]) /
					sqrt(pow(plane_coeffs[0], 2) + pow(plane_coeffs[1], 2) + pow(plane_coeffs[2], 2));
	
	return dist;
}

std::unordered_set<int> Ransac2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	// random number seed
	srand(time(NULL));
	// get the start timestamp
	auto t_start = std::chrono::high_resolution_clock::now();

	std::unordered_set<int> inliersResult;
	
	// number of random samples to select per iteration
	const int n_random_samples = 2;

	// x, y, and z points as a vector
	std::vector<float> x, y, z;
	
	// number of inliers for each iteration
	std::vector<int> n_inliers(maxIterations, 0);
	// coefficients for each line
	std::vector<std::vector<float>> coeffs(maxIterations);

	// iterate 'maxIterations' number of times
	for(int i = 0; i < maxIterations; ++i)
	{
		// clear the vectors
		x.clear();
		y.clear();
		z.clear();
		// select random samples
		for(int j = 0; j < n_random_samples; ++j)
		{
			int idx = rand()%cloud->size();
			x.push_back(cloud->at(idx).x);
			y.push_back(cloud->at(idx).y);
			z.push_back(cloud->at(idx).z);
		}
		// fit a line
		coeffs[i] = fitLine(x, y);

		for(pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin(); it != cloud->end(); ++it)
		{
			if(lineToPointDist(coeffs[i], *it) <= distanceTol)
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
	for(pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin(); it != cloud->end(); ++it)
	{
		if(lineToPointDist(coeffs[index_max_n_inlier], *it) <= distanceTol)
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

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceToPlane)
{
	// random number seed
	srand(time(NULL));
	// get the start timestamp
	auto t_start = std::chrono::high_resolution_clock::now();

	std::unordered_set<int> inliersResult;
	
	// number of random samples to select per iteration
	const int n_random_samples = 3;

	// x, y, and z points as a vector
	std::vector<float> x, y, z;
	
	// number of inliers for each iteration
	std::vector<int> n_inliers(maxIterations, 0);
	// coefficients for each plane
	std::vector<std::vector<float>> coeffs(maxIterations);

	// iterate 'maxIterations' number of times
	for(int i = 0; i < maxIterations; ++i)
	{
		// clear the vectors
		x.clear();
		y.clear();
		z.clear();
		// select random samples
		for(int j = 0; j < n_random_samples; ++j)
		{
			int idx = rand()%cloud->size();
			x.push_back(cloud->at(idx).x);
			y.push_back(cloud->at(idx).y);
			z.push_back(cloud->at(idx).z);
		}
		// fit a plane
		coeffs[i] = fitPlane(x, y, z);

		for(pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin(); it != cloud->end(); ++it)
		{
			if(planeToPointDist(coeffs[i], *it) <= distanceToPlane)
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
	for(pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin(); it != cloud->end(); ++it)
	{
		if(planeToPointDist(coeffs[index_max_n_inlier], *it) <= distanceToPlane)
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

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac3D(cloud, 100, 0.8);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
