#include <car_planner/car_manager.h>

void CarManager::init(ros::NodeHandle &nh)
{
    /*  param  */
    nh.param("car/car_l",car_l,0.6);
    nh.param("car/car_w",car_w,0.4);
    nh.param("car/car_h",car_h,0.3);
    nh.param("car/max_steer",max_steer,0.7);
    nh.param("car/wheelbase",wheelbase,0.6);
    nh.param("car/mesh",mesh_resource,std::string("package://car_planner/param/car.dae"));
    nh.param("car/frame",frame,std::string("world"));

    /* callback */
    viscar_pub = nh.advertise<visualization_msgs::Marker>("odom_mesh", 100,true);
    waypoints_sub = nh.subscribe("waypoints", 1, &CarManager::rcvWaypointsCallback, this);
    odom_sub = nh.subscribe("odom", 1, &CarManager::odomCallback, this);
}

void CarManager::odomCallback(const nav_msgs::OdometryConstPtr& odom)
{
    visualization_msgs::Marker WpMarker;
    WpMarker.id = 0;
    WpMarker.header.stamp = ros::Time::now();
    WpMarker.header.frame_id = "world";
    WpMarker.action = visualization_msgs::Marker::ADD;
    WpMarker.type = visualization_msgs::Marker::MESH_RESOURCE;
    WpMarker.ns = "car_mesh";
    WpMarker.mesh_use_embedded_materials = true;
    WpMarker.color.r = 0.0;
    WpMarker.color.g = 0.0;
    WpMarker.color.b = 0.0;
    WpMarker.color.a = 0.0;
    WpMarker.scale.x = car_l/4.5;
    WpMarker.scale.y = car_l/4.5;
    WpMarker.scale.z = car_l/4.5;
    Eigen::Quaterniond q(odom->pose.pose.orientation.w, 
    odom->pose.pose.orientation.x, odom->pose.pose.orientation.y,
    odom->pose.pose.orientation.z);
    Eigen::Quaterniond qz(cos(M_PI/4),0,0,sin(M_PI/4));
    q = qz*q;
    WpMarker.pose.orientation.w = q.w();
    WpMarker.pose.orientation.x = q.x();
    WpMarker.pose.orientation.y = q.y();
    WpMarker.pose.orientation.z = q.z();
    WpMarker.pose.position = odom->pose.pose.position;
    WpMarker.mesh_resource = mesh_resource;
    viscar_pub.publish(WpMarker);
}

void CarManager::rcvWaypointsCallback(const geometry_msgs::PoseStamped& msg)
{
   std::cout<<"manager get waypoints!"<<std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "car_vis_node");
  ros::NodeHandle nh("~");
  CarManager car;
  car.init(nh);
  ros::spin();
  return 0;
}