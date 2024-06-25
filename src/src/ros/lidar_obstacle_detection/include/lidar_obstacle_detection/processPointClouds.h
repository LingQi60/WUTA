// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "lidar_obstacle_detection/box.h"

// Structure to represent node of kd tree
struct Node                                     //链表
{
	pcl::PointXYZI point;
	int id;
	Node* left;
	Node* right;

	Node(pcl::PointXYZI arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}
	void insertHelper(Node** node, int depth, pcl::PointXYZI point, int id)
    {
      if(*node == NULL) (*node) = new Node(point, id);
      else
      {
        int cd = depth % 2;
        if(cd == 0){
            if(point.x < (*node)->point.x) insertHelper(&(*node)->left, depth + 1, point, id);
            else insertHelper(&(*node)->right, depth + 1, point, id);
        }
        else{
            if(point.y < (*node)->point.y) insertHelper(&(*node)->left, depth + 1, point, id);
            else insertHelper(&(*node)->right, depth + 1, point, id);
        }
        
      }
    }
  
	void insert(pcl::PointXYZI point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id);
	}

void searchHelper(pcl::PointXYZI pivot, Node* node, int depth, float distanceTol, std::vector<int>& ids)
{
	if(node != NULL)
    {
	  if((node->point.x >= (pivot.x-distanceTol)&&(node->point.x <= (pivot.x+distanceTol)))&&(node->point.y >= (pivot.y-distanceTol)&&(node->point.y <= (pivot.y+distanceTol))))
      {
        float distance = sqrt((node->point.x - pivot.x) * (node->point.x - pivot.x) + (node->point.y - pivot.y) * (node->point.y - pivot.y));
        if(distance <= distanceTol) ids.push_back(node->id);
      }
      if(depth%2 == 0){
        if((pivot.x - distanceTol) < node->point.x) searchHelper(pivot, node->left, depth+1, distanceTol, ids);
        if((pivot.x + distanceTol) > node->point.x) searchHelper(pivot, node->right, depth+1, distanceTol, ids);
      }
      else{
        if((pivot.y - distanceTol) < node->point.y) searchHelper(pivot, node->left, depth+1, distanceTol, ids);
        if((pivot.y + distanceTol) > node->point.y) searchHelper(pivot, node->right, depth+1, distanceTol, ids);
      }
      
    }
}
  
	// return a list of point ids in the tree that are within distance of pivot
	std::vector<int> search(pcl::PointXYZI pivot, float distanceTol)
	{
		std::vector<int> ids;
    searchHelper(pivot, root, 0, distanceTol, ids);
		return ids;
	}
	

};





template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    // 点云个数 5 ~
    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    // 过滤点云 2
    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint,Eigen::Vector4f carmin, Eigen::Vector4f carmax);

    //单独的点云
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    //段平面 3
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    //聚类 4
    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    //团簇 
    void clusterHelper(int idx, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol);

    //欧几里得簇
    std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol, int minSize, int maxSize);
                

    //包围框 6 ~
    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    //保存为pcd文件 
    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    //读取pcd文件 1
    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    //目录中提取文件名存在vector容器中 0
    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
    
};
#endif /* PROCESSPOINTCLOUDS_H_ */
