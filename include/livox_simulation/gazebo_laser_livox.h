#ifndef ARTI_GAZEBO_LASER_LIVOX_H
#define ARTI_GAZEBO_LASER_LIVOX_H
#include <string>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/PointCloud2.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/plugins/RayPlugin.hh>

#include <sdf/Param.hh>

#include "gazebo/gazebo_config.h"
#include "livox_simulation/ray_data.h"

namespace gazebo
{

class ArtiGazeboLaserLivox: public RayPlugin
{
public:
  // Public functions
  ArtiGazeboLaserLivox();
  ~ArtiGazeboLaserLivox();
  void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

protected:
  // Protected functions
  virtual void OnNewLaserScans();

private:
  // Functions
  void PutLaserData(common::Time &_updateTime);
  int connect_count_;
  void Connect();
  void Disconnect();
  void LoadThread();
  void LaserQueueThread();
  bool AddRayEllipseShape(double rotation_degrees);
  /*void CalculatePoints(livox::RayData rays, double horiz_angles, double vert_angles);*/

#if GAZEBO_MAJOR_VERSION >= 9
  typedef ignition::math::Vector3d V3;
  typedef ignition::math::Pose3d P3;
  typedef ignition::math::Quaterniond Q3;
#else
  typedef math::Vector3 V3;
  typedef math::Pose3 P3;
  typedef math::Quaternion Q3;
#endif
  // Variables
  physics::WorldPtr world_;
  sensors::SensorPtr parent_sensor_;
  sensors::RaySensorPtr parent_ray_sensor_;

  ros::NodeHandle* rosnode_;
  ros::Publisher pub_;
  std::string topic_name_;
  std::string frame_name_;
  double update_rate_;
  double update_period_;
  common::Time last_update_time_;
  std::string robot_namespace_;
  ros::CallbackQueue laser_queue_;
  boost::thread callback_queue_thread_;
  sdf::ElementPtr sdf;

  boost::thread deferred_load_thread_;
  gazebo::physics::PhysicsEnginePtr engine_;

  std::vector<gazebo::physics::RayShapePtr> rays_;

  std::vector<livox::RayData> double_ellipse_rays_;

  physics::CollisionPtr collision_ptr_;
  physics::MultiRayShapePtr multi_rays_;
  std::vector<physics::CollisionPtr> collision_ptr_list_;
  std::vector<float> vertical_ray_angles_;
  std::vector<float> horizontal_ray_angles_;
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d ray_startpoint_;
  ignition::math::Pose3d sensor_pose_;
#else
  math::Vector3 ray_startpoint_;
  math::Pose3 sensor_pose_;
#endif
  sensor_msgs::PointCloud cloud_msg_;
  sensor_msgs::PointCloud2 pc2_msgs_;

  int samples_;
  int num_ellipses_;
  size_t interpolation_points_;
  float max_interpolation_distance_;
  float min_range_;
  float max_range_;
  float rotation_increment_;
  double current_rot_angle_ = 0;


  bool init_finished_ = false;

  int debug_counter = 0;

  ros::Publisher pub1_;
  ros::Publisher pub2_;
  ros::Publisher pub3_;
  ros::Publisher pub4_;
  sensor_msgs::PointCloud cloud1_msg_;
  sensor_msgs::PointCloud cloud2_msg_;
  sensor_msgs::PointCloud cloud3_msg_;
  sensor_msgs::PointCloud cloud4_msg_;
  sensor_msgs::PointCloud cloud1all_msg_;

  void SetX(V3& v, const double val)
  {
  #if GAZEBO_MAJOR_VERSION >= 8
    v.X(val);
  #else
    v.x = val;
  #endif
  }
  
  void SetY(V3& v, const double val)
  {
  #if GAZEBO_MAJOR_VERSION >= 8
    v.Y(val);
  #else
    v.y = val;
  #endif
  }
  
  void SetZ(V3& v, const double val)
  {
  #if GAZEBO_MAJOR_VERSION >= 8
    v.Z(val);
  #else
    v.z = val;
  #endif
  }
  
  double GetX(const V3& v)
  {
  #if GAZEBO_MAJOR_VERSION >= 8
    return v.X();
  #else
    return v.x;
  #endif
  }
  
  double GetY(const V3& v)
  {
  #if GAZEBO_MAJOR_VERSION >= 8
    return v.Y();
  #else
    return v.y;
  #endif
  }
  
  double GetZ(const V3& v)
  {
  #if GAZEBO_MAJOR_VERSION >= 8
    return v.Z();
  #else
    return v.z;
  #endif
  }
  
  Q3 GetQuaternion(const P3& p)
  {
  #if GAZEBO_MAJOR_VERSION >= 8
    return p.Rot();
  #else
    return p.rot;
  #endif
  }

  V3 GetPose(const P3& p)
  {
  #if GAZEBO_MAJOR_VERSION >= 8
    return p.Pos();
  #else
    return p.pos;
  #endif
  }

  V3 GetEuler(const Q3& q)
  {
  #if GAZEBO_MAJOR_VERSION >= 8
    return q.Euler();
  #else
    return q.GetAsEuler();
  #endif
  }

  void SetEuler(Q3& q, const V3& v)
  {
  #if GAZEBO_MAJOR_VERSION >= 8
    q.Euler(v);
  #else
    q.SetFromEuler(v);
  #endif
  }
};
}
#endif // ARTI_GAZEBO_LASER_LIVOX_H
