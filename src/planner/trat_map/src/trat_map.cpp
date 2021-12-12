#include "trat_map/trat_map.h"

void TratMap::initMap(ros::NodeHandle &nh)
{
  node_ = nh;

  /* get parameter */
  node_.param("trat_map/scanVoxelSize", mp_.scanVoxelSize, 1.0);
  node_.param("trat_map/decayTime", mp_.decayTime, 1.0);
  node_.param("trat_map/noDecayDis", mp_.noDecayDis, 1.0);
  node_.param("trat_map/clearingDis", mp_.clearingDis, 1.0);
  node_.param("trat_map/useSorting", mp_.useSorting, false);
  node_.param("trat_map/quantileZ", mp_.quantileZ, 1.0);
  node_.param("trat_map/considerDrop", mp_.considerDrop, true);
  node_.param("trat_map/limitGroundLift", mp_.limitGroundLift, false);
  node_.param("trat_map/maxGroundLift", mp_.maxGroundLift, 1.0);
  node_.param("trat_map/clearDyObs", mp_.clearDyObs, true);
  node_.param("trat_map/minDyObsDis", mp_.minDyObsDis, 1.0);
  node_.param("trat_map/minDyObsAngle", mp_.minDyObsAngle,1.0);
  node_.param("trat_map/minDyObsRelZ",mp_. minDyObsRelZ, 1.0);
  node_.param("trat_map/minDyObsVFOV", mp_.minDyObsVFOV, 1.0);
  node_.param("trat_map/maxDyObsVFOV", mp_.maxDyObsVFOV, 1.0);
  node_.param("trat_map/minDyObsPointNum", mp_.minDyObsPointNum, 1);
  node_.param("trat_map/noDataObstacle", mp_.noDataObstacle, false);
  node_.param("trat_map/noDataBlockSkipNum", mp_.noDataBlockSkipNum, 0);
  node_.param("trat_map/minBlockPointNum", mp_.minBlockPointNum, 10);
  node_.param("trat_map/vehicleHeight", mp_.vehicleHeight, 1.0);
  node_.param("trat_map/voxelPointUpdateThre", mp_.voxelPointUpdateThre, 100);
  node_.param("trat_map/voxelTimeUpdateThre", mp_.voxelTimeUpdateThre, 1.0);
  node_.param("trat_map/minRelZ", mp_.minRelZ, 1.0);
  node_.param("trat_map/maxRelZ", mp_.maxRelZ, 1.0);
  node_.param("trat_map/disRatioZ", mp_.disRatioZ, 1.0);
  node_.param("trat_map/min_cnormal", mp_.min_cnormal, 1.0);
  node_.param("trat_map/max_rho", mp_.max_rho, 1.0);
  node_.param("trat_map/eta", mp_.eta, 1.0);
  node_.param("trat_map/rough_threshold", mp_.rough_threshold, 1.0);
  
  md_.laserCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  md_.laserCloudCrop.reset(new pcl::PointCloud<pcl::PointXYZI>());
  md_.laserCloudDwz.reset(new pcl::PointCloud<pcl::PointXYZI>());
  md_.terrainCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  md_.terrainCloudElev.reset(new pcl::PointCloud<pcl::PointXYZI>());
  md_.roughCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  for (auto i=0;i<441;i++)
  {
    md_.terrainVoxelCloud[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
    
  for (auto i=0;i<2601;i++)
  {
    md_.roughVoxelCloud[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
  md_.downSizeFilter.setLeafSize(mp_.scanVoxelSize, mp_.scanVoxelSize, mp_.scanVoxelSize);

  /* init callback */
  cloud_sub_ = node_.subscribe<sensor_msgs::PointCloud2>("trat_map/cloud", 5, &TratMap::cloudCallback, this);
  odom_sub_ = node_.subscribe<nav_msgs::Odometry>("trat_map/odom", 5, &TratMap::odomCallback, this);

  // pro_timer_ = node_.createTimer(ros::Duration(0.05), &TratMap::proCallback, this);
  // vis_timer_ = node_.createTimer(ros::Duration(0.11), &TratMap::visCallback, this);

  map_inf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("trat_map/occupancy_inflate", 2);
}

// state estimation callback function
void TratMap::odomCallback(const nav_msgs::Odometry::ConstPtr &odom) {
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w))
      .getRPY(roll, pitch, yaw);

  md_.vehicleRoll = roll;
  md_.vehiclePitch = pitch;
  md_.vehicleYaw = yaw;
  md_.vehicleX = odom->pose.pose.position.x;
  md_.vehicleY = odom->pose.pose.position.y;
  md_.vehicleZ = odom->pose.pose.position.z;

  md_.sinVehicleRoll = sin(md_.vehicleRoll);
  md_.cosVehicleRoll = cos(md_.vehicleRoll);
  md_.sinVehiclePitch = sin(md_.vehiclePitch);
  md_.cosVehiclePitch = cos(md_.vehiclePitch);
  md_.sinVehicleYaw = sin(md_.vehicleYaw);
  md_.cosVehicleYaw = cos(md_.vehicleYaw);

  if (md_.noDataInited == 0) {
    md_.vehicleXRec = md_.vehicleX;
    md_.vehicleYRec =md_. vehicleY;
    md_.noDataInited = 1;
  }
  if (md_.noDataInited == 1) {
    float dis = sqrt((md_.vehicleX - md_.vehicleXRec) * (md_.vehicleX - md_.vehicleXRec) +
                     (md_.vehicleY - md_.vehicleYRec) * (md_.vehicleY - md_.vehicleYRec));
    if (dis >= mp_.noDecayDis)
      md_.noDataInited = 2;
  }
  md_.has_odom = true;
}

void TratMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &laserCloud2) {
  md_.laserCloudTime = laserCloud2->header.stamp.toSec();

  if (!md_.systemInited) {
    md_.systemInitTime = md_.laserCloudTime;
    md_.systemInited = true;
  }

  md_.laserCloud->clear();
  pcl::fromROSMsg(*laserCloud2, *md_.laserCloud);

  pcl::PointXYZI point;
  md_.laserCloudCrop->clear();
  int laserCloudSize = md_.laserCloud->points.size();
  for (int i = 0; i < laserCloudSize; i++) {
    point = md_.laserCloud->points[i];

    float pointX = point.x;
    float pointY = point.y;
    float pointZ = point.z;

    float dis = sqrt((pointX - md_.vehicleX) * (pointX - md_.vehicleX) +
                     (pointY - md_.vehicleY) * (pointY - md_.vehicleY));
    if (pointZ - md_.vehicleZ > mp_.minRelZ - mp_.disRatioZ * dis &&
        pointZ - md_.vehicleZ < mp_.maxRelZ + mp_.disRatioZ * dis &&
        dis < mp_.terrainVoxelSize * (mp_.terrainVoxelHalfWidth + 1)) {
      point.x = pointX;
      point.y = pointY;
      point.z = pointZ;
      point.intensity = md_.laserCloudTime - md_.systemInitTime;
      md_.laserCloudCrop->push_back(point);
    }
  }

  md_.newlaserCloud = true;

  if (md_.has_odom)
  {
    // terrain voxel roll over
    float terrainVoxelCenX = mp_.terrainVoxelSize * mp_.terrainVoxelShiftX;
    float terrainVoxelCenY = mp_.terrainVoxelSize * mp_.terrainVoxelShiftY;

    while (md_.vehicleX - terrainVoxelCenX < -mp_.terrainVoxelSize) {
      for (int indY = 0; indY < mp_.terrainVoxelWidth; indY++) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
            md_.terrainVoxelCloud[mp_.terrainVoxelWidth * (mp_.terrainVoxelWidth - 1) +
                              indY];
        for (int indX = mp_.terrainVoxelWidth - 1; indX >= 1; indX--) {
          md_.terrainVoxelCloud[mp_.terrainVoxelWidth * indX + indY] =
              md_.terrainVoxelCloud[mp_.terrainVoxelWidth * (indX - 1) + indY];
        }
        md_.terrainVoxelCloud[indY] = terrainVoxelCloudPtr;
        md_.terrainVoxelCloud[indY]->clear();
      }
      mp_.terrainVoxelShiftX--;
      terrainVoxelCenX = mp_.terrainVoxelSize * mp_.terrainVoxelShiftX;
    }

    while (md_.vehicleX - terrainVoxelCenX > mp_.terrainVoxelSize) {
      for (int indY = 0; indY < mp_.terrainVoxelWidth; indY++) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
            md_.terrainVoxelCloud[indY];
        for (int indX = 0; indX < mp_.terrainVoxelWidth - 1; indX++) {
          md_.terrainVoxelCloud[mp_.terrainVoxelWidth * indX + indY] =
              md_.terrainVoxelCloud[mp_.terrainVoxelWidth * (indX + 1) + indY];
        }
        md_.terrainVoxelCloud[mp_.terrainVoxelWidth * (mp_.terrainVoxelWidth - 1) +
                          indY] = terrainVoxelCloudPtr;
        md_.terrainVoxelCloud[mp_.terrainVoxelWidth * (mp_.terrainVoxelWidth - 1) + indY]
            ->clear();
      }
      mp_.terrainVoxelShiftX++;
      terrainVoxelCenX = mp_.terrainVoxelSize * mp_.terrainVoxelShiftX;
    }

    while (md_.vehicleY - terrainVoxelCenY < -mp_.terrainVoxelSize) {
      for (int indX = 0; indX < mp_.terrainVoxelWidth; indX++) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
            md_.terrainVoxelCloud[mp_.terrainVoxelWidth * indX +
                              (mp_.terrainVoxelWidth - 1)];
        for (int indY = mp_.terrainVoxelWidth - 1; indY >= 1; indY--) {
          md_.terrainVoxelCloud[mp_.terrainVoxelWidth * indX + indY] =
              md_.terrainVoxelCloud[mp_.terrainVoxelWidth * indX + (indY - 1)];
        }
        md_.terrainVoxelCloud[mp_.terrainVoxelWidth * indX] = terrainVoxelCloudPtr;
        md_.terrainVoxelCloud[mp_.terrainVoxelWidth * indX]->clear();
      }
      mp_.terrainVoxelShiftY--;
      terrainVoxelCenY = mp_.terrainVoxelSize * mp_.terrainVoxelShiftY;
    }

    while (md_.vehicleY - terrainVoxelCenY > mp_.terrainVoxelSize) {
      for (int indX = 0; indX < mp_.terrainVoxelWidth; indX++) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
            md_.terrainVoxelCloud[mp_.terrainVoxelWidth * indX];
        for (int indY = 0; indY < mp_.terrainVoxelWidth - 1; indY++) {
          md_.terrainVoxelCloud[mp_.terrainVoxelWidth * indX + indY] =
              md_.terrainVoxelCloud[mp_.terrainVoxelWidth * indX + (indY + 1)];
        }
        md_.terrainVoxelCloud[mp_.terrainVoxelWidth * indX +
                          (mp_.terrainVoxelWidth - 1)] = terrainVoxelCloudPtr;
        md_.terrainVoxelCloud[mp_.terrainVoxelWidth * indX + (mp_.terrainVoxelWidth - 1)]
            ->clear();
      }
      mp_.terrainVoxelShiftY++;
      terrainVoxelCenY = mp_.terrainVoxelSize * mp_.terrainVoxelShiftY;
    }

    // stack registered laser scans
    pcl::PointXYZI point;
    int laserCloudCropSize = md_.laserCloudCrop->points.size();
    for (int i = 0; i < laserCloudCropSize; i++) {
      point = md_.laserCloudCrop->points[i];

      int indX = int((point.x - md_.vehicleX + mp_.terrainVoxelSize / 2) /
                      mp_.terrainVoxelSize) +
                  mp_.terrainVoxelHalfWidth;
      int indY = int((point.y - md_.vehicleY + mp_.terrainVoxelSize / 2) /
                      mp_.terrainVoxelSize) +
                  mp_.terrainVoxelHalfWidth;

      if (point.x - md_.vehicleX + mp_.terrainVoxelSize / 2 < 0)
        indX--;
      if (point.y - md_.vehicleY + mp_.terrainVoxelSize / 2 < 0)
        indY--;

      if (indX >= 0 && indX < mp_.terrainVoxelWidth && indY >= 0 &&
          indY < mp_.terrainVoxelWidth) {
        md_.terrainVoxelCloud[mp_.terrainVoxelWidth * indX + indY]->push_back(point);
        md_.terrainVoxelUpdateNum[mp_.terrainVoxelWidth * indX + indY]++;
      }
    }

    for (int ind = 0; ind < mp_.terrainVoxelNum; ind++) {
      if (md_.terrainVoxelUpdateNum[ind] >= mp_.voxelPointUpdateThre ||
          md_.laserCloudTime - md_.systemInitTime - md_.terrainVoxelUpdateTime[ind] >=
              mp_.voxelTimeUpdateThre ||
          mp_.clearingCloud) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
            md_.terrainVoxelCloud[ind];

        md_.laserCloudDwz->clear();
        md_.downSizeFilter.setInputCloud(terrainVoxelCloudPtr);
        md_.downSizeFilter.filter(*md_.laserCloudDwz);

        terrainVoxelCloudPtr->clear();
        int laserCloudDwzSize = md_.laserCloudDwz->points.size();
        for (int i = 0; i < laserCloudDwzSize; i++) {
          point = md_.laserCloudDwz->points[i];
          float dis = sqrt((point.x - md_.vehicleX) * (point.x - md_.vehicleX) +
                            (point.y - md_.vehicleY) * (point.y -md_. vehicleY));
          if (point.z - md_.vehicleZ > mp_.minRelZ - mp_.disRatioZ * dis &&
              point.z - md_.vehicleZ < mp_.maxRelZ + mp_.disRatioZ * dis &&
              (md_.laserCloudTime - md_.systemInitTime - point.intensity <
                    mp_.decayTime ||
                dis < mp_.noDecayDis) &&
              !(dis < mp_.clearingDis && mp_.clearingCloud)) {
            terrainVoxelCloudPtr->push_back(point);
          }
        }

        md_.terrainVoxelUpdateNum[ind] = 0;
        md_.terrainVoxelUpdateTime[ind] = md_.laserCloudTime - md_.systemInitTime;
      }
    }

    md_.terrainCloud->clear();
    for (int indX = mp_.terrainVoxelHalfWidth - 5;
          indX <= mp_.terrainVoxelHalfWidth + 5; indX++) {
      for (int indY = mp_.terrainVoxelHalfWidth - 5;
            indY <= mp_.terrainVoxelHalfWidth + 5; indY++) {
        *(md_.terrainCloud) += *(md_.terrainVoxelCloud[mp_.terrainVoxelWidth * indX + indY]);
      }
    }
    
    // pcl::search::KdTree<pcl::PointXYZI>::Ptr treeSampling (new pcl::search::KdTree<pcl::PointXYZI>);// 创建用于最近邻搜索的KD-Tree
    // pcl::PointCloud<pcl::PointXYZI> mls_point;    //输出MLS
    // pcl::MovingLeastSquares<pcl::PointXYZI,pcl::PointXYZI> mls; // 定义最小二乘实现的对象mls
    // mls.setComputeNormals(false);  //设置在最小二乘计算中是否需要存储计算的法线
    // mls.setInputCloud(md_.terrainCloud);         //设置待处理点云
    // mls.setPolynomialOrder(2);            // 拟合2阶多项式拟合
    // mls.setPolynomialFit(false);     // 设置为false可以 加速 smooth
    // mls.setSearchMethod(treeSampling);         // 设置KD-Tree作为搜索方法
    // mls.setSearchRadius(0.05);           // 单位m.设置用于拟合的K近邻半径
    // mls.process(mls_point);                 //输出
    // md_.terrainCloud->clear();
    // md_.terrainCloud = mls_point.makeShared();

    // estimate ground and compute elevation for each point
    for (int i = 0; i < mp_.planarVoxelNum; i++) {
      md_.roughVoxelCloud[i]->clear();
      // md_.roughVoxelRho[i] = 0;
    }

    int terrainCloudSize = md_.terrainCloud->points.size();
    for (int i = 0; i < terrainCloudSize; i++) {
      point = md_.terrainCloud->points[i];

      int indX =
          int((point.x - md_.vehicleX + mp_.planarVoxelSize / 2) / mp_.planarVoxelSize) +
          mp_.planarVoxelHalfWidth;
      int indY =
          int((point.y - md_.vehicleY + mp_.planarVoxelSize / 2) / mp_.planarVoxelSize) +
          mp_.planarVoxelHalfWidth;

      if (point.x - md_.vehicleX + mp_.planarVoxelSize / 2 < 0)
        indX--;
      if (point.y - md_.vehicleY + mp_.planarVoxelSize / 2 < 0)
        indY--;

      if (point.z - md_.vehicleZ > mp_.minRelZ && point.z - md_.vehicleZ < mp_.maxRelZ) {
        if (indX>= 0 && indX < mp_.planarVoxelWidth && indY >= 0 && indY < mp_.planarVoxelWidth) {
          md_.roughVoxelCloud[mp_.planarVoxelWidth * indX + indY]->push_back(point);
        }
        // for (int dX = -1; dX <= 1; dX++) {
        //   for (int dY = -1; dY <= 1; dY++) {
        //     if (indX + dX >= 0 && indX + dX < mp_.planarVoxelWidth &&
        //         indY + dY >= 0 && indY + dY < mp_.planarVoxelWidth) {
        //       md_.planarPointElev[mp_.planarVoxelWidth * (indX + dX) + indY + dY]
        //           .push_back(point.z);
        //     }
        //   }
        // }
      }

      if (mp_.clearDyObs) {
        if (indX >= 0 && indX < mp_.planarVoxelWidth && indY >= 0 &&
            indY < mp_.planarVoxelWidth) {
          float pointX1 = point.x - md_.vehicleX;
          float pointY1 = point.y - md_.vehicleY;
          float pointZ1 = point.z - md_.vehicleZ;

          float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
          if (dis1 > mp_.minDyObsDis) {
            float angle1 = atan2(pointZ1 - mp_.minDyObsRelZ, dis1) * 180.0 / M_PI;
            if (angle1 > mp_.minDyObsAngle) {
              float pointX2 =
                  pointX1 * md_.cosVehicleYaw + pointY1 * md_.sinVehicleYaw;
              float pointY2 =
                  -pointX1 * md_.sinVehicleYaw + pointY1 * md_.cosVehicleYaw;
              float pointZ2 = pointZ1;

              float pointX3 =
                  pointX2 * md_.cosVehiclePitch - pointZ2 * md_.sinVehiclePitch;
              float pointY3 = pointY2;
              float pointZ3 =
                  pointX2 * md_.sinVehiclePitch + pointZ2 * md_.cosVehiclePitch;

              float pointX4 = pointX3;
              float pointY4 =
                  pointY3 * md_.cosVehicleRoll + pointZ3 * md_.sinVehicleRoll;
              float pointZ4 =
                  -pointY3 * md_.sinVehicleRoll + pointZ3 * md_.cosVehicleRoll;

              float dis4 = sqrt(pointX4 * pointX4 + pointY4 * pointY4);
              float angle4 = atan2(pointZ4, dis4) * 180.0 / M_PI;
              if (angle4 > mp_.minDyObsVFOV && angle4 < mp_.maxDyObsVFOV) {
                md_.planarVoxelDyObs[mp_.planarVoxelWidth * indX + indY]++;
              }
            }
          } else {
            md_.planarVoxelDyObs[mp_.planarVoxelWidth * indX + indY] +=
                mp_.minDyObsPointNum;
          }
        }
      }
    }

    // estimate normal and rho: algorithm one: 
    md_.roughCloud->clear();
    // for (auto i=0;i<mp_.planarVoxelNum;i++)
    // {
    //   pcl::PointCloud<pcl::PointXYZI>::Ptr roughVoxelCloudPtr = md_.roughVoxelCloud[i];
    //   if (roughVoxelCloudPtr->size()<mp_.minBlockPointNum)
    //   {
    //     md_.roughVoxelRho[i] = 0;
    //   }
    //   else
    //   {
    //     double cnormal  = 0;
    //     Eigen::Matrix<double,3,1> mean_p = Eigen::Vector3d::Zero();
    //     Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();

    //     for (size_t j = 0; j <roughVoxelCloudPtr->size(); j++)
    //     {
    //       mean_p += Eigen::Matrix<double,3,1>((*roughVoxelCloudPtr)[j].x, (*roughVoxelCloudPtr)[j].y, (*roughVoxelCloudPtr)[j].z);
    //     }
    //     mean_p /= 3;

    //     for (size_t j = 0; j <roughVoxelCloudPtr->size(); j++)
    //     {
    //       Eigen::Matrix<double, 3, 1> v = Eigen::Matrix<double, 3, 1>((*roughVoxelCloudPtr)[j].x,  
    //         (*roughVoxelCloudPtr)[j].y, (*roughVoxelCloudPtr)[j].z) - mean_p;
    //       cov += v * v.transpose();
    //     }

    //     /// opposite PCA
    //     Eigen::EigenSolver<Eigen::Matrix3d> es(cov);
    //     Eigen::Matrix<double, 3, 1> D = es.pseudoEigenvalueMatrix().diagonal();// eigenValue
    //     Eigen::Matrix3d V = es.pseudoEigenvectors();    // eigenVector
    //     Eigen::MatrixXd::Index evalsMax;
    //     D.minCoeff(&evalsMax);
    //     Eigen::Matrix<double, 3, 1> Z_t = V.col(evalsMax);
    //     cnormal = fabs(Z_t.col(0)[2]);
    //     if (cnormal<mp_.min_cnormal)
    //     {
    //       md_.roughVoxelRho[i] = mp_.max_rho + 0.05;
    //     }else
    //     {
    //       // estimate planar
    //       Eigen::Matrix<double,3,1> mean_p = Eigen::Vector3d::Zero();
    //       Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    //       pcl::PointCloud<pcl::PointXYZI>::Ptr temp(new pcl::PointCloud<pcl::PointXYZI>());
    //       for (int dX = -1; dX <= 1; dX++) {
    //         for (int dY = -1; dY <= 1; dY++) {
    //           int a = i + dX*mp_.planarVoxelWidth + dY;
    //           if (a >= 0 && a < mp_.planarVoxelNum) {
    //                 *(temp) += *(md_.roughVoxelCloud[a]);
    //           }
    //         }
    //       }

    //       for (size_t j = 0; j <temp->size(); j++)
    //       {
    //         mean_p += Eigen::Matrix<double,3,1>((*temp)[j].x, (*temp)[j].y, (*temp)[j].z);
    //       }
    //       mean_p /= 3;

    //       for (size_t j = 0; j <temp->size(); j++)
    //       {
    //         Eigen::Matrix<double, 3, 1> v = Eigen::Matrix<double, 3, 1>((*temp)[j].x, 
    //           (*temp)[j].y, (*temp)[j].z) - mean_p;
    //         cov += v * v.transpose();
    //       }

    //       // opposite PCA
    //       Eigen::EigenSolver<Eigen::Matrix3d> es(cov);
    //       Eigen::Matrix<double, 3, 1> D = es.pseudoEigenvalueMatrix().diagonal();// eigenValue
    //       Eigen::Matrix3d V = es.pseudoEigenvectors();    // eigenVector
    //       Eigen::MatrixXd::Index evalsMax;
    //       D.minCoeff(&evalsMax);
    //       Eigen::Matrix<double, 3, 1> n = V.col(evalsMax);

    //       // get rho
    //       std::vector<double> nears;
    //       for (size_t j = 0; j <roughVoxelCloudPtr->size(); j++)
    //       {
    //         Eigen::Matrix<double, 3, 1> pi = { (*roughVoxelCloudPtr)[j].x, 
    //         (*roughVoxelCloudPtr)[j].y, (*roughVoxelCloudPtr)[j].z};
    //         nears.push_back(n.transpose().dot(pi - mean_p));
    //       }
    //       sort(nears.begin(), nears.end());
    //       size_t a = static_cast<double>(nears.size()*mp_.eta / 2.0);
    //       double d_min = nears[a];
    //       double d_max = nears[nears.size() - 1 - a];
    //       double rho_res = fabs(d_max - d_min);
    //       md_.roughVoxelRho[i] = rho_res>mp_.max_rho?mp_.max_rho:rho_res;
    //     }
    //   }

    //   for (size_t j=0; j<roughVoxelCloudPtr->size(); j++)
    //   {
    //     (*roughVoxelCloudPtr)[j].intensity = md_.roughVoxelRho[i];
    //     md_.roughCloud->push_back((*roughVoxelCloudPtr)[j]);
    //   }
    // }

    // estimate normal and rho: algorithm two: 
    for (auto i=0;i<mp_.planarVoxelNum;i++)
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr roughVoxelCloudPtr = md_.roughVoxelCloud[i];
      
      double cnormal  = 0;
      // estimate planar
      Eigen::Matrix<double,3,1> mean_p = Eigen::Vector3d::Zero();
      Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
      pcl::PointCloud<pcl::PointXYZI>::Ptr temp(new pcl::PointCloud<pcl::PointXYZI>());
      for (int dX = -1; dX <= 1; dX++) {
        for (int dY = -1; dY <= 1; dY++) {
          int a = i + dX*mp_.planarVoxelWidth + dY;
          if (a >= 0 && a < mp_.planarVoxelNum) {
                *(temp) += *(md_.roughVoxelCloud[a]);
          }
        }
      }

      if (temp->size()<mp_.minBlockPointNum)
      {
        md_.roughVoxelRho[i] = 0;
      }
      else
      {
        for (size_t j = 0; j <temp->size(); j++)
        {
          mean_p += Eigen::Matrix<double,3,1>((*temp)[j].x, (*temp)[j].y, (*temp)[j].z);
        }
        mean_p /= 3;

        for (size_t j = 0; j <temp->size(); j++)
        {
          Eigen::Matrix<double, 3, 1> v = Eigen::Matrix<double, 3, 1>((*temp)[j].x, 
            (*temp)[j].y, (*temp)[j].z) - mean_p;
          cov += v * v.transpose();
        }

        // opposite PCA
        Eigen::EigenSolver<Eigen::Matrix3d> es(cov);
        Eigen::Matrix<double, 3, 1> D = es.pseudoEigenvalueMatrix().diagonal();// eigenValue
        Eigen::Matrix3d V = es.pseudoEigenvectors();    // eigenVector
        Eigen::MatrixXd::Index evalsMax;
        D.minCoeff(&evalsMax);
        Eigen::Matrix<double, 3, 1> n = V.col(evalsMax);

        cnormal = fabs(n.col(0)[2]);
        
        if (cnormal<mp_.min_cnormal)
        {
          md_.roughVoxelRho[i] = mp_.max_rho;
        }else
        {
          // get rho
          std::vector<double> nears;
          for (size_t j = 0; j <roughVoxelCloudPtr->size(); j++)
          {
            Eigen::Matrix<double, 3, 1> pi = { (*roughVoxelCloudPtr)[j].x, 
            (*roughVoxelCloudPtr)[j].y, (*roughVoxelCloudPtr)[j].z};
            nears.push_back(n.transpose().dot(pi - mean_p));
          }
          sort(nears.begin(), nears.end());
          size_t a = static_cast<double>(nears.size()*mp_.eta / 2.0);
          if (nears.empty())
          {
            md_.roughVoxelRho[i] = 0;
          }else
          {
            double d_min = nears[a];
            double d_max = nears[nears.size() - 1 - a];
            double rho_res = fabs(d_max - d_min);
            md_.roughVoxelRho[i] = rho_res>mp_.max_rho?mp_.max_rho:rho_res;
          }
        }
      }
      for (size_t j=0; j<roughVoxelCloudPtr->size(); j++)
      {
        (*roughVoxelCloudPtr)[j].intensity = md_.roughVoxelRho[i];
        md_.roughCloud->push_back((*roughVoxelCloudPtr)[j]);
      }
    }

    if (mp_.clearDyObs) {
      for (int i = 0; i < laserCloudCropSize; i++) {
        point = md_.laserCloudCrop->points[i];

        int indX = int((point.x - md_.vehicleX + mp_.planarVoxelSize / 2) /
                        mp_.planarVoxelSize) +
                    mp_.planarVoxelHalfWidth;
        int indY = int((point.y - md_.vehicleY + mp_.planarVoxelSize / 2) /
                        mp_.planarVoxelSize) +
                    mp_.planarVoxelHalfWidth;

        if (point.x - md_.vehicleX + mp_.planarVoxelSize / 2 < 0)
          indX--;
        if (point.y - md_.vehicleY + mp_.planarVoxelSize / 2 < 0)
          indY--;

        if (indX >= 0 && indX < mp_.planarVoxelWidth && indY >= 0 &&
            indY < mp_.planarVoxelWidth) {
          float pointX1 = point.x - md_.vehicleX;
          float pointY1 = point.y - md_.vehicleY;
          float pointZ1 = point.z - md_.vehicleZ;

          float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
          float angle1 = atan2(pointZ1 - mp_.minDyObsRelZ, dis1) * 180.0 / M_PI;
          if (angle1 > mp_.minDyObsAngle) {
            md_.planarVoxelDyObs[mp_.planarVoxelWidth * indX + indY] = 0;
          }
        }
      }
    }

    mp_.clearingCloud = false;
    // publish points with elevation
    sensor_msgs::PointCloud2 terrainCloudElev2;
    pcl::toROSMsg(*md_.roughCloud, terrainCloudElev2);
    terrainCloudElev2.header.stamp = ros::Time().fromSec(md_.laserCloudTime);
    terrainCloudElev2.header.frame_id = "world";
    map_inf_pub_.publish(terrainCloudElev2);
  }
}

void TratMap::proCallback(const ros::TimerEvent& /*event*/)
{
}

void TratMap::visCallback(const ros::TimerEvent& /*event*/)
{
    
}

void TratMap::evaluateRhoWithGrad(const Eigen::Vector3d& pos, const Eigen::Vector3d& direction, double& cost, Eigen::Vector3d& grad)
{
  double rho;
  Eigen::Vector2d po(pos[0],pos[1]);
  Eigen::Vector2i id;
  posToIndex(po, id);
  if (!isInMap(id))
  {
    cost = 0;
    grad = Eigen::Vector3d::Zero();
    return;
  }else
  {
    rho = md_.roughVoxelRho[mp_.planarVoxelWidth * id[0] + id[1]];
    if (rho<mp_.rough_threshold)
    {
      cost = 0;
      grad = Eigen::Vector3d::Zero();
      return;
    }else
    {
      cost = rho - mp_.rough_threshold;
    }
  }

  // try get gridient use simple idea
  double rho_r=0, rho_l=0;
  Eigen::Vector2i idr,idl;
  Eigen::Vector2d por = po+direction.head(2)*mp_.roughVoxelSize;
  Eigen::Vector2d pol = po-direction.head(2)*mp_.roughVoxelSize;

  posToIndex(por,idr);
  posToIndex(pol,idl);
  if (isInMap(idr))
  {
    rho_r = md_.roughVoxelRho[mp_.planarVoxelWidth * idr[0] + idr[1]];
  }
  if (isInMap(idl))
  {
    rho_l = md_.roughVoxelRho[mp_.planarVoxelWidth * idl[0] + idl[1]];
  }

  double a = rho_r-rho;
  double b = rho_l-rho;
  if (a<=b && a<-mp_.max_rho*0.2)
  {
    grad = -direction;
  }else if(b<a && b< -mp_.max_rho*0.2)
  {
    grad = direction;
  }else
  {
    grad = Eigen::Vector3d::Zero();
  }
}
