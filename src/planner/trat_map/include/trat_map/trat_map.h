#pragma once
#include <math.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/impl/mls.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

using namespace std;

struct tMappingParameters{
    double scanVoxelSize = 0.05;
    double decayTime = 2.0;
    double noDecayDis = 4.0;
    double clearingDis = 8.0;
    bool clearingCloud = false;
    bool useSorting = true;
    double quantileZ = 0.25;
    bool considerDrop = false;
    bool limitGroundLift = false;
    double maxGroundLift = 0.15;
    bool clearDyObs = false;
    double minDyObsDis = 0.3;
    double minDyObsAngle = 0;
    double minDyObsRelZ = -0.5;
    double minDyObsVFOV = -16.0;
    double maxDyObsVFOV = 16.0;
    int minDyObsPointNum = 1;
    bool noDataObstacle = false;
    int noDataBlockSkipNum = 0;
    int minBlockPointNum = 10;
    double vehicleHeight = 1.5;
    int voxelPointUpdateThre = 100;
    double voxelTimeUpdateThre = 2.0;
    double minRelZ = -1.5;
    double maxRelZ = 0.2;
    double disRatioZ = 0.2;

    // terrain voxel parameters
    float terrainVoxelSize = 1.0;
    int terrainVoxelShiftX = 0;
    int terrainVoxelShiftY = 0;
    const int terrainVoxelWidth = 21;
    int terrainVoxelHalfWidth = (terrainVoxelWidth - 1) / 2;
    const int terrainVoxelNum = terrainVoxelWidth * terrainVoxelWidth;

    // planar voxel parameters
    float planarVoxelSize = 0.2;
    const int planarVoxelWidth = 51;
    int planarVoxelHalfWidth = (planarVoxelWidth - 1) / 2;
    const int planarVoxelNum = planarVoxelWidth * planarVoxelWidth;

    // rough voxel parameters
    float roughVoxelSize = 0.2;
    const int roughVoxelWidth = 51;
    int roughVoxelHalfWidth = (roughVoxelWidth - 1) / 2;
    const int roughVoxelNum = roughVoxelWidth * roughVoxelWidth;
    double min_cnormal = 0.0;
    double max_rho = 0.0;
    double eta = 0.0;
};

struct tMappingData{
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz;
    pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudElev;
    pcl::PointCloud<pcl::PointXYZI>::Ptr roughCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloud[441];
    pcl::PointCloud<pcl::PointXYZI>::Ptr roughVoxelCloud[2601];

    int terrainVoxelUpdateNum[441] = {0};
    float terrainVoxelUpdateTime[441] = {0};
    float planarVoxelElev[2601] = {0};
    int planarVoxelEdge[2601] = {0};
    int planarVoxelDyObs[2601] = {0};
    vector<float> planarPointElev[2601];
    float roughVoxelRho[2601] = {0};

    double laserCloudTime = 0;
    bool newlaserCloud = false;
    bool has_odom = false;

    double systemInitTime = 0;
    bool systemInited = false;
    int noDataInited = 0;

    float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
    float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
    float vehicleXRec = 0, vehicleYRec = 0;

    float sinVehicleRoll = 0, cosVehicleRoll = 0;
    float sinVehiclePitch = 0, cosVehiclePitch = 0;
    float sinVehicleYaw = 0, cosVehicleYaw = 0;

    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
};

class TratMap {
public:
  TratMap() {}
  ~TratMap() {}

  enum { POSE_STAMPED = 1, ODOMETRY = 2, INVALID_IDX = -10000 };

  // occupancy map management
  void resetBuffer();
  void resetBuffer(Eigen::Vector2d min, Eigen::Vector2d max);

  inline void posToIndex(const Eigen::Vector2d& pos, Eigen::Vector2i& id);
  inline void indexToPos(const Eigen::Vector2i& id, Eigen::Vector2d& pos);
  inline int toAddress(const Eigen::Vector2i& id);
  inline int toAddress(int& x, int& y);
  inline bool isInMap(const Eigen::Vector2d& pos);
  inline bool isInMap(const Eigen::Vector2i& idx);
  inline double getResolution() { return mp_.roughVoxelSize/2; }

  inline int getInflateOccupancy(Eigen::Vector3d pos);
  
  void initMap(ros::NodeHandle& nh);

  void publishMapInflate(bool all_info = false);

  typedef std::shared_ptr<TratMap> Ptr;
private:
  tMappingParameters mp_;
  tMappingData md_;

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& img);
  void odomCallback(const nav_msgs::OdometryConstPtr& odom);
  void proCallback(const ros::TimerEvent& /*event*/);
  void visCallback(const ros::TimerEvent& /*event*/);
  
  ros::NodeHandle node_;
  ros::Subscriber odom_sub_, cloud_sub_;
  ros::Publisher map_inf_pub_;
  ros::Timer pro_timer_;
  ros::Timer vis_timer_;
};

inline int TratMap::toAddress(const Eigen::Vector2i& id) {
  int a = mp_.planarVoxelWidth * id[0] + id[1];
  if (a>=0&&a<mp_.planarVoxelNum)
  {
    return a;
  }
  else
  {
    return -1;
  }
}

inline int TratMap::toAddress(int& x, int& y) {
  int indX =
      int((x - md_.vehicleX + mp_.planarVoxelSize / 2) / mp_.planarVoxelSize) +
      mp_.planarVoxelHalfWidth;
  int indY =
      int((y - md_.vehicleY + mp_.planarVoxelSize / 2) / mp_.planarVoxelSize) +
      mp_.planarVoxelHalfWidth;

  if (x - md_.vehicleX + mp_.planarVoxelSize / 2 < 0)
    indX--;
  if (y - md_.vehicleY + mp_.planarVoxelSize / 2 < 0)
    indY--;
  int a = mp_.planarVoxelWidth * indX + indY;
  if (a>=0&&a<mp_.planarVoxelNum)
  {
    return a;
  }
  else
  {
    return -1;
  }
}

inline int TratMap::getInflateOccupancy(Eigen::Vector3d pos) {
  Eigen::Vector2d po(pos[0],pos[1]);
  // return 0;
  Eigen::Vector2i id;
  posToIndex(po, id);
  if (!isInMap(id)) return 0;

  // return int(md_.occupancy_buffer_inflate_[toAddress(id)]);
  if (md_.roughVoxelRho[mp_.planarVoxelWidth * id[0] + id[1]]>mp_.max_rho-1e-5)
  {
    return 1;
  }else
  {
    return 0;
  }
}

inline bool TratMap::isInMap(const Eigen::Vector2d& pos) {
  int indX =
      int((pos(0) - md_.vehicleX + mp_.planarVoxelSize / 2) / mp_.planarVoxelSize) +
      mp_.planarVoxelHalfWidth;
  int indY =
      int((pos(1) - md_.vehicleY + mp_.planarVoxelSize / 2) / mp_.planarVoxelSize) +
      mp_.planarVoxelHalfWidth;

  if (pos(0) - md_.vehicleX + mp_.planarVoxelSize / 2 < 0)
    indX--;
  if (pos(1) - md_.vehicleY + mp_.planarVoxelSize / 2 < 0)
    indY--;
  int a = mp_.planarVoxelWidth * indX + indY;
  if (a>=0&&a<mp_.planarVoxelNum)
  {
    return true;
  }
  else
  {
    return false;
  }
}

inline bool TratMap::isInMap(const Eigen::Vector2i& idx) {
  int a = mp_.planarVoxelWidth * idx[0] + idx[1];
  if (a>=0&&a<mp_.planarVoxelNum)
  {
    return true;
  }
  else
  {
    return false;
  }
}

inline void TratMap::posToIndex(const Eigen::Vector2d& pos, Eigen::Vector2i& id) {
  int indX =
      int((pos(0) - md_.vehicleX + mp_.planarVoxelSize / 2) / mp_.planarVoxelSize) +
      mp_.planarVoxelHalfWidth;
  int indY =
      int((pos(1) - md_.vehicleY + mp_.planarVoxelSize / 2) / mp_.planarVoxelSize) +
      mp_.planarVoxelHalfWidth;

  if (pos(0) - md_.vehicleX + mp_.planarVoxelSize / 2 < 0)
    indX--;
  if (pos(1) - md_.vehicleY + mp_.planarVoxelSize / 2 < 0)
    indY--;
  id[0] = indX;
  id[1] = indY;
}

inline void TratMap::indexToPos(const Eigen::Vector2i& id, Eigen::Vector2d& pos) {
  pos(0) = (id[0] - mp_.planarVoxelHalfWidth)*mp_.planarVoxelSize + md_.vehicleX;
  pos(1) = (id[1] - mp_.planarVoxelHalfWidth)*mp_.planarVoxelSize + md_.vehicleY;
}
