#include "helper.h"
#include <pcl/registration/icp.h>
#include <Eigen/Geometry>
#include <pcl/console/time.h>



typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


Eigen::Matrix4d ICP(PointCloudT::Ptr target, PointCloudT::Ptr source, Pose startingPose, int iterations);