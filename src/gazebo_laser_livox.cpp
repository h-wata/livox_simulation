
#include <gazebo/gazebo.hh>
#include <tf/tf.h>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "livox_simulation/gazebo_laser_livox.h"

namespace gazebo
{
// Register this plugin
GZ_REGISTER_SENSOR_PLUGIN(ArtiGazeboLaserLivox)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor
ArtiGazeboLaserLivox::ArtiGazeboLaserLivox()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Destructor
ArtiGazeboLaserLivox::~ArtiGazeboLaserLivox()
{
  this->laser_queue_.clear();
  this->laser_queue_.disable();
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Load the controller
void ArtiGazeboLaserLivox::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  RayPlugin::Load(_parent, this->sdf);

  this->parent_sensor_ = _parent;
  std::string worldName = _parent->WorldName();
  this->world_ = physics::get_world(worldName);
#if GAZEBO_MAJOR_VERSION>=9
  this->engine_ = this->world_->Physics();
#else
  this->engine_ = this->world_->GetPhysicsEngine();
#endif
  this->engine_->InitForThread();
  this->sdf = _sdf;

  GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
  this->parent_ray_sensor_ =
    dynamic_pointer_cast<sensors::RaySensor>(this->parent_sensor_);

  if (!this->parent_ray_sensor_)
    gzthrow("ArtiGazeboLaserLivox controller requires a Ray-Sensor as it's parent");

  this->robot_namespace_ = "";
  if (this->sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = this->sdf->Get<std::string>("robotNamespace") + "/";

  if (!this->sdf->HasElement("frameName"))
  {
    ROS_INFO_NAMED("livox", "Livox-plugin parameter <frameName> is missing, defaults to /world");
    this->frame_name_ = "/world";
  }
  else
    this->frame_name_ = this->sdf->Get<std::string>("frameName");

  if (!this->sdf->HasElement("topicName"))
  {
    ROS_INFO_NAMED("livox", "Livox-plugin parameter <topicName> is missing, defaults to /laser/livox40");
    this->topic_name_ = "/laser/livox40";
  }
  else
    this->topic_name_ = this->sdf->Get<std::string>("topicName");

  if (!this->sdf->HasElement("updateRate"))
  {
    ROS_INFO_NAMED("livox", "Livox-plugin parameter <updateRate> is missing, defaults to 0");
    this->update_rate_ = 0;
  }
  else
    this->update_rate_ = this->sdf->Get<double>("updateRate");

  if (!this->sdf->HasElement("samples"))
  {
    ROS_INFO_NAMED("livox", "Livox-plugin parameter <samples> is missing, defaults to 30");
    this->samples_ = 30;
  }
  else
    this->samples_ = this->sdf->Get<int>("samples");

  if (!this->sdf->HasElement("numDoubleEllipses"))
  {
    ROS_INFO_NAMED("livox", "Livox-plugin parameter <numDoubleEllipses> is missing, defaults to 8");
    this->num_ellipses_ = 1;
  }
  else this->num_ellipses_ = this->sdf->Get<int>("numDoubleEllipses");

  if (!this->sdf->HasElement("rotationIncrement"))
  {
    ROS_INFO_NAMED("livox", "Livox-plugin parameter <rotationIncrement> is missing, defaults to 0.2 [rad].");
    this->rotation_increment_ = 0.2;
  }
  else
    this->rotation_increment_ = this->sdf->Get<float>("rotationIncrement");

  if (!this->sdf->HasElement("interpolationPoints"))
  {
    ROS_INFO_NAMED("livox", "Livox-plugin parameter <interpolationPoints> is missing, defaults to 10.");
    this->interpolation_points_ = 10;
  }
  else
    this->interpolation_points_ = this->sdf->Get<unsigned int>("interpolationPoints");

  if (!this->sdf->HasElement("maxInterpolationDistance"))
  {
    ROS_INFO_NAMED("livox", "Livox-plugin parameter <maxInterpolationDistance> is missing, defaults to 1.0.");
    this->max_interpolation_distance_ = 1.0;
  }
  else
    this->max_interpolation_distance_ = this->sdf->Get<float>("maxInterpolationDistance");

  if (!this->sdf->HasElement("minRange"))
  {
    ROS_INFO_NAMED("livox", "Livox-plugin parameter <minRange> is missing, defaults to 0.0");
    this->min_range_ = 0.0;
  }
  else
    this->min_range_ = this->sdf->Get<float>("minRange");

  if (!this->sdf->HasElement("maxRange"))
  {
    ROS_INFO_NAMED("livox", "Livox-plugin parameter <maxRange> is missing, defaults to 30.0");
    this->max_range_ = 30.0;
  }
  else
    this->max_range_ = this->sdf->Get<float>("maxRange");

  if (this->update_rate_ > 0.0)
    this->update_period_ = 1.0 / this->update_rate_;
  else
    this->update_period_ = 0.0;

  this->connect_count_ = 0;

  if (ros::isInitialized())
  {
    this->deferred_load_thread_ = boost::thread(boost::bind(&ArtiGazeboLaserLivox::LoadThread, this));
  }
  else
  {
    gzerr << "ERROR: ROS hasn't been initialized!\n";
  }

  collision_ptr_list_.clear();

  this->parent_ray_sensor_->SetActive(false);
  // Create number of ellipse-8-figures
  double increment = 180.0 / this->num_ellipses_;
  for (int i = 0; i < this->num_ellipses_; i++)
  {
    this->AddRayEllipseShape(0.0 + i * increment);
  }

  this->last_update_time_ = common::Time(0);

  this->parent_ray_sensor_->SetActive(true);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Load Thread
void ArtiGazeboLaserLivox::LoadThread()
{
  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);
  this->frame_name_ = tf::resolve(prefix, this->frame_name_);

  if (this->topic_name_ != "")
  {
    ros::AdvertiseOptions ao;
    ao = ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>(this->topic_name_, 1,
                                                                 boost::bind(&ArtiGazeboLaserLivox::Connect, this),
                                                                 boost::bind(&ArtiGazeboLaserLivox::Disconnect, this),
                                                                 ros::VoidPtr(), &this->laser_queue_);

    this->pub_ = this->rosnode_->advertise(ao);

  }

  this->parent_ray_sensor_->SetActive(false);

  this->callback_queue_thread_ =
    boost::thread(boost::bind(&ArtiGazeboLaserLivox::LaserQueueThread, this));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Increment counter
void ArtiGazeboLaserLivox::Connect()
{
  this->connect_count_++;
  this->parent_ray_sensor_->SetActive(true);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Decrement counter
void ArtiGazeboLaserLivox::Disconnect()
{
  this->connect_count_--;

  if (this->connect_count_ == 0)
    this->parent_ray_sensor_->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Create new Laserscan
void ArtiGazeboLaserLivox::OnNewLaserScans()
{
  if (this->topic_name_ != "")
  {
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time cur_time = this->world_->SimTime();
#else
    common::Time cur_time = this->world_->GetSimTime();
#endif
    if (cur_time < this->last_update_time_)
    {
      ROS_WARN_NAMED("livox", "WARNING: Current time was negative (smaller than last-update-time).");
      this->last_update_time_ = cur_time;
    }

    if (cur_time - this->last_update_time_ >= this->update_period_)
    {
      common::Time sensor_update_time = this->parent_sensor_->LastUpdateTime();
      // timeval dt;
      // dt.tv_sec = 1;
      // dt.tv_usec = 200;
      // sensor_update_time -= dt;
      this->PutLaserData(last_update_time_);
      this->last_update_time_ = cur_time;
    }
  }
  else
  {
    ROS_ERROR_NAMED("livox", "ERROR: Topic for Livox-lidar not set!");
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Add points from raytracing into Pointcloud
void ArtiGazeboLaserLivox::PutLaserData(common::Time &_updateTime)
{
  this->parent_ray_sensor_->SetActive(false);

  double dist;
  std::string entityName;

  this->cloud_msg_.points.clear();
  this->cloud_msg_.channels.clear();

  sensor_msgs::ChannelFloat32 intensity = sensor_msgs::ChannelFloat32();
  intensity.values.clear();
  intensity.name = "intensity";

  P3 offset;
  // offset = this->collision_ptr_->RelativePose();
  offset = this->parent_ray_sensor_->Pose();
  V3 offset_rot = GetEuler(GetQuaternion(offset));

  Q3 rot_only;
  SetEuler(rot_only, V3(this->current_rot_angle_ + GetX(offset_rot), GetY(offset_rot), GetZ(offset_rot)));
  #if GAZEBO_MAJOR_VERSION >= 8
  offset.Rot() = rot_only;
  #else
  offset.rot = rot_only;
  #endif

  bool has_prev_value = false;
 // ROS_ERROR_STREAM("num ellipses: " << this->double_ellipse_rays_.size());
 // ROS_ERROR_STREAM("num ellipse rays: " << this->double_ellipse_rays_[0].getSizeLeftLowerQuadrant());

  // sensor_msgs::ChannelFloat32 intensity1 = sensor_msgs::ChannelFloat32();
  // intensity1.values.clear();
  // intensity1.name = "intensity";

  // Go throw every double-ellipse
  for (int j = 0; j < this->double_ellipse_rays_.size(); j++)
  // for (int j = 0; j < 1; j++)
  {
    // ROS_INFO_STREAM("double_elipse_rays:" << j);
    // In every double-ellipse, go throw every quadrant
    cloud1_msg_.points.clear();
    cloud1_msg_.channels.clear();
    cloud2_msg_.points.clear();
    cloud2_msg_.channels.clear();
    cloud3_msg_.points.clear();
    cloud3_msg_.channels.clear();
    cloud4_msg_.points.clear();
    cloud4_msg_.channels.clear();
    // Go throw left lower quadrant

    // ROS_INFO_STREAM("double_elipse_rays:" << j);
    std::vector<int> quadrant_size = {
      double_ellipse_rays_[j].getSizeLeftLowerQuadrant(), 
      double_ellipse_rays_[j].getSizeLeftUpperQuadrant(), 
      double_ellipse_rays_[j].getSizeRightLowerQuadrant(), 
      double_ellipse_rays_[j].getSizeRightUpperQuadrant()};

    // ROS_INFO_STREAM("horizon angle:" << j);
    std::vector<std::vector<float>> horizon_angles = {
      double_ellipse_rays_[j].left_lower_horizontal_ray_angles_,
      double_ellipse_rays_[j].left_upper_horizontal_ray_angles_,
      double_ellipse_rays_[j].right_lower_horizontal_ray_angles_,
      double_ellipse_rays_[j].right_upper_horizontal_ray_angles_,
    };


    // ROS_INFO_STREAM("vertical angle:" << j);
    std::vector<std::vector<float>> vertical_angles = {
      double_ellipse_rays_[j].left_lower_vertical_ray_angles_,
      double_ellipse_rays_[j].left_upper_vertical_ray_angles_,
      double_ellipse_rays_[j].right_lower_vertical_ray_angles_,
      double_ellipse_rays_[j].right_upper_vertical_ray_angles_,
    };

    // ROS_INFO_STREAM("quadrants:" << j);
    std::vector<std::vector<gazebo::physics::RayShapePtr>> quadrants =
    { 
      double_ellipse_rays_[j].left_lower_quadrant_rays_,
      double_ellipse_rays_[j].left_upper_quadrant_rays_,
      double_ellipse_rays_[j].right_lower_quadrant_rays_,
      double_ellipse_rays_[j].right_upper_quadrant_rays_,
    };

    for(int c = 0; c < 4; c++)
    {
      Q3 ray;
      V3 axis;
      V3 endpoint;
      geometry_msgs::Point32 point, rot_point;
      // ROS_INFO_STREAM("c:" << c);
      for (int i = 0; i < quadrant_size[c]; i++)
      {
        float horizontal_ray_angle = horizon_angles[c][i];
        float vertical_ray_angle = vertical_angles[c][i];
        
        // ray.Euler(V3(0.0, -vertical_ray_angle, horizontal_ray_angle));
        SetEuler(ray, V3(0.0, -vertical_ray_angle, horizontal_ray_angle));
        // axis = offset.rot * ray * math::Vector3(1.0, 0.0, 0.0);
        // axis = ray * V3(1.0, 0.0, 0.0);
        axis = GetQuaternion(offset) * ray * V3(1.0, 0.0, 0.0);
        endpoint = (axis * this->max_range_) + GetPose(offset);

        quadrants[c][i]->SetPoints(this->ray_startpoint_, endpoint);
        if(i==0){
        }
        quadrants[c][i]->Update();
        // Get distance of collision for the current ray
        quadrants[c][i]->GetIntersection(dist, entityName);
        // If the distance is below 1000.0, than a collision with an object happened
        if (dist < 999.9)
        {
          // Calculate point from ellipse-shaped-rays without considering the rotation-angle
          point.x = cos(vertical_ray_angle) * cos(horizontal_ray_angle) * dist;
          point.y = sin(horizontal_ray_angle) * cos(vertical_ray_angle) * dist;
          point.z = sin(vertical_ray_angle) * dist;

          rot_point.x = point.x;
          rot_point.y = point.y * cos(this->current_rot_angle_) - point.z * sin(this->current_rot_angle_);
          rot_point.z = point.y * sin(this->current_rot_angle_) + point.z * cos(this->current_rot_angle_);
          this->cloud_msg_.points.push_back(rot_point);
        }
        else
          has_prev_value = false;
        
      }
    }
  }
  // Add intensity-field to the pointcloud
  this->cloud_msg_.channels.push_back(intensity);

  // this->cloud1_msg_.channels.push_back(intensity1);
  // Increment the current rotation angle
  this->current_rot_angle_ += this->rotation_increment_;
  // Don't exceed pi
  if (this->current_rot_angle_ >= M_PI)
    this->current_rot_angle_ -= M_PI;

  this->cloud_msg_.header.frame_id = this->frame_name_;
  this->cloud_msg_.header.stamp.sec = _updateTime.sec;
  this->cloud_msg_.header.stamp.nsec = _updateTime.nsec;

  // Transform PointCloud to PointCloud2
  sensor_msgs::convertPointCloudToPointCloud2(this->cloud_msg_, this->pc2_msgs_);

  // The first message published is always invalid, therefore don't publish it
  if(this->init_finished_ == false)
  {
    this->init_finished_ = true;
  }
  else
  {
    // Publish ROS-pointcloud
    this->pub_.publish(this->pc2_msgs_);

  }
  this->parent_ray_sensor_->SetActive(true);
}

/*void ArtiGazeboLaserLivox::CalculatePoints(gazebo::physics::RayShapePtr rays, double horiz_angles, double vert_angles)
{

}*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ArtiGazeboLaserLivox::LaserQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->laser_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Create an eight-figure suing two ellipses as a ray-pattern for raytracing
bool ArtiGazeboLaserLivox::AddRayEllipseShape(double rotation_degrees)
{
  double samples_a = this->samples_;
  double ell_a = 0.1746; // length of ellipse parameter a in a distance of 1 m with a FOV of 38.4°
  double ell_b = 0.0364; // length of ellipse parameter b in a distance of 1 m with a FOV of 38.4°
  double dx = 2.0 * ell_a / samples_a;
  Q3 ray;
  V3 axis;
  P3 offset;
  V3 start, end1, end2, end3, end4;
  livox::RayData eight_ray_pattern;

  start.Set(0.0, 0.0, 0.0);
  

  std::string parent_name = this->parent_ray_sensor_->ParentName();
  this->multi_rays_ = this->parent_ray_sensor_->LaserShape();
  while (this->multi_rays_->RayCount() < 4)
  {
    this->multi_rays_->AddRay(start, end1);
  }
  // this->collision_ptr_ = this->engine_->CreateCollision("ray", parent_name);
  // this->collision_ptr_->SetName("own_ray_sensor_collision");
  // this->sensor_pose_ = this->parent_ray_sensor_->Pose();
  // this->collision_ptr_->SetRelativePose(this->sensor_pose_);
  // this->collision_ptr_->SetInitialRelativePose(this->sensor_pose_);
  // 
  // offset = this->collision_ptr_->RelativePose();
  offset = this->parent_ray_sensor_->Pose();
  // V3 offset_rot = offset.Rot().Euler();
  V3 offset_rot = GetEuler(GetQuaternion(offset));
  SetEuler(ray, offset_rot);
  axis = GetQuaternion(offset) * ray * V3(1.0, 0.0, 0.0);

  // Get the position of the pose of the parent_ray_sensor and add its position to the new ray (its relative to the parent-frame)
  start = (axis * this->min_range_) + GetPose(offset);
  this->ray_startpoint_ = start;



  //std::cout << "Time 1: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - tstart).count() << " ms." << std::endl;

  std::vector<float> beta_1_values;
  std::vector<float> beta_2_values;
  std::vector<float> beta_3_values;
  std::vector<float> beta_4_values;

  std::vector<float> alpha_1_values;
  std::vector<float> alpha_2_values;
  std::vector<float> alpha_3_values;
  std::vector<float> alpha_4_values;

  std::vector<physics::CollisionPtr> collision_1_elements;
  std::vector<physics::CollisionPtr> collision_2_elements;
  std::vector<physics::CollisionPtr> collision_3_elements;
  std::vector<physics::CollisionPtr> collision_4_elements;

  std::vector<gazebo::physics::RayShapePtr> rays_1;
  std::vector<gazebo::physics::RayShapePtr> rays_2;
  std::vector<gazebo::physics::RayShapePtr> rays_3;
  std::vector<gazebo::physics::RayShapePtr> rays_4;

  for (int i = 0; i <= samples_a; i++)
  {
    // use x as indexed variable that moves along the ellipse
    double val_x = ((double) i) * dx - ell_a;
    // calculate the two y-coordinates from the x-coordinate of the ellipse
    double val_y_1 = sqrt((1 - pow(val_x, 2.0) / pow(ell_a, 2.0)) * pow(ell_b, 2.0));
    double val_y_2 = -sqrt((1 - pow(val_x, 2.0) / pow(ell_a, 2.0)) * pow(ell_b, 2.0));

    // The point (0,0) is in the middle of the ellipse, therefore we move the ellipse to one side by the amount of ell_a
    double val_x_1 = val_x + ell_a;
    // And a second time to the other side. Now we have four ellipse value pairs
    double val_x_2 = val_x - ell_a;

    // The four ellipse points are:
    // (val_x_1, val_y_1)
    // (val_x_1, val_y_2)
    // (val_x_2, val_y_1)
    // (val_x_2, val_y_2)

    double rot_rad = rotation_degrees / 180.0 * M_PI;
    double rot_mat[2][2] = {{cos(rot_rad), -sin(rot_rad)}, {sin(rot_rad), cos(rot_rad)}};

    double ell_x1 = rot_mat[0][0] * val_x_1 + rot_mat[0][1] * val_y_1;
    double ell_y1 = rot_mat[1][0] * val_x_1 + rot_mat[1][1] * val_y_1;

    double ell_x2 = rot_mat[0][0] * val_x_1 + rot_mat[0][1] * val_y_2;
    double ell_y2 = rot_mat[1][0] * val_x_1 + rot_mat[1][1] * val_y_2;

    double ell_x3 = rot_mat[0][0] * val_x_2 + rot_mat[0][1] * val_y_1;
    double ell_y3 = rot_mat[1][0] * val_x_2 + rot_mat[1][1] * val_y_1;

    double ell_x4 = 0.0;
    double ell_y4 = 0.0;
    // for the first sample there are 4 rays and 2 of them are equal. Modify one to get one ray in the middle
    // and 2 on the outer side of the ellipse.
    if(false) //if (i == 0)
    {
      ell_x4 = rot_mat[0][0] * -val_x_2 + rot_mat[0][1] * val_y_2;
      ell_y4 = rot_mat[1][0] * -val_x_2 + rot_mat[1][1] * val_y_2;
    }
    else
    {
      ell_x4 = rot_mat[0][0] * val_x_2 + rot_mat[0][1] * val_y_2;
      ell_y4 = rot_mat[1][0] * val_x_2 + rot_mat[1][1] * val_y_2;
    }

    // Note: The current ellipse has the following axis: y is up and x is in the middle and pointing to the right.
    //       Therefore we need to map the y-axis of the ellipse to the Ray-z-axis and the x-axis of the ellipse to the
    //       -Ray-y-axis.

    SetX(end1, 1.0 + GetX(GetPose(offset)));
    SetY(end1, -ell_x1 + GetY(GetPose(offset)));
    SetZ(end1, ell_y1 + GetZ(GetPose(offset)));

    SetX(end2, 1.0 + GetX(GetPose(offset)));
    SetY(end2, -ell_x2 + GetY(GetPose(offset)));
    SetZ(end2, ell_y2 + GetZ(GetPose(offset)));

    SetX(end3, 1.0 + GetX(GetPose(offset)));
    SetY(end3, -ell_x3 + GetY(GetPose(offset)));
    SetZ(end3, ell_y3 + GetZ(GetPose(offset)));

    SetX(end4, 1.0 + GetX(GetPose(offset)));
    SetY(end4, -ell_x4 + GetY(GetPose(offset)));
    SetZ(end4, ell_y4 + GetZ(GetPose(offset)));

    double beta1 = (atan2(GetY(end1) - GetY(start), GetX(end1) - GetX(start)));
    double beta2 = (atan2(GetY(end2) - GetY(start), GetX(end2) - GetX(start)));
    double beta3 = (atan2(GetY(end3) - GetY(start), GetX(end3) - GetX(start)));
    double beta4 = (atan2(GetY(end4) - GetY(start), GetX(end4) - GetX(start)));
    double alpha1 = (atan2(GetZ(end1) - GetZ(start), GetX(end1) - GetX(start)));
    double alpha2 = (atan2(GetZ(end2) - GetZ(start), GetX(end2) - GetX(start)));
    double alpha3 = (atan2(GetZ(end3) - GetZ(start), GetX(end3) - GetX(start)));
    double alpha4 = (atan2(GetZ(end4) - GetZ(start), GetX(end4) - GetX(start)));

    SetX(end1, (1.0 + GetX(GetPose(offset))) * this->max_range_);
    SetY(end1, (-ell_x1 + GetY(GetPose(offset))) * this->max_range_);
    SetZ(end1, (ell_y1 + GetZ(GetPose(offset))) * this->max_range_);
    // ray.Euler(V3(0.0 + GetX(offset_rot), -alpha1 + GetY(offset_rot), beta1 + GetZ(offset_rot)));
    SetEuler(ray, V3(0.0 + GetX(offset_rot), -alpha1 + GetY(offset_rot), beta1 + GetZ(offset_rot)));
    axis = GetQuaternion(offset) * ray * V3(1.0, 0.0, 0.0);
    end1 = (axis * this->max_range_) + GetPose(offset);

    SetX(end2, (1.0 + GetX(GetPose(offset))) * this->max_range_);
    SetY(end2, (-ell_x2 + GetY(GetPose(offset))) * this->max_range_);
    SetZ(end2, (ell_y2 + GetZ(GetPose(offset))) * this->max_range_);
    // ray.Euler(V3(0.0 + GetX(offset_rot), -alpha2 + GetY(offset_rot), beta2 + GetZ(offset_rot))); SetEuler(ray, V3(0.0 + GetX(offset_rot), -alpha2 + GetY(offset_rot), beta2 + GetZ(offset_rot)));
    axis = GetQuaternion(offset) * ray * V3(1.0, 0.0, 0.0);
    end2 = (axis * this->max_range_) + GetPose(offset);

    SetX(end3, (1.0 + GetX(GetPose(offset))) * this->max_range_);
    SetY(end3, (-ell_x3 + GetY(GetPose(offset))) * this->max_range_);
    SetZ(end3, (ell_y3 + GetZ(GetPose(offset))) * this->max_range_);
    // ray.Euler(V3(0.0 + GetX(offset_rot), -alpha3 + GetY(offset_rot), beta3 + GetZ(offset_rot)));
    SetEuler(ray, V3(0.0 + GetX(offset_rot), -alpha3 + GetY(offset_rot), beta3 + GetZ(offset_rot)));
    axis = GetQuaternion(offset) * ray * V3(1.0, 0.0, 0.0);
    end3 = (axis * this->max_range_) + GetPose(offset);

    SetX(end4, (1.0 + GetX(GetPose(offset))) * this->max_range_);
    SetY(end4, (-ell_x4 + GetY(GetPose(offset))) * this->max_range_);
    SetZ(end4, (ell_y4 + GetZ(GetPose(offset))) * this->max_range_);
    // ray.Euler(V3(0.0 + GetX(offset_rot), -alpha4 + GetY(offset_rot), beta4 + GetZ(offset_rot)));
    SetEuler(ray, V3(0.0 + GetX(offset_rot), -alpha4 + GetY(offset_rot), beta4 + GetZ(offset_rot)));
    axis = GetQuaternion(offset) * ray * V3(1.0, 0.0, 0.0);
    end4 = (axis * this->max_range_) + GetPose(offset);

    std::string parent_name = this->parent_ray_sensor_->ParentName();
    //this->parent_ray_sensor_->LaserShape()->shared_from_this();
    this->multi_rays_->SetRay(0, start, end1);
    this->multi_rays_->SetRay(1, start, end2);
    this->multi_rays_->SetRay(2, start, end3);
    this->multi_rays_->SetRay(3, start, end4);


    this->multi_rays_->Ray(0)->SetName("ray_sensor_1");
    this->multi_rays_->Ray(1)->SetName("ray_sensor_2");
    this->multi_rays_->Ray(2)->SetName("ray_sensor_3");
    this->multi_rays_->Ray(3)->SetName("ray_sensor_4");
    rays_1.push_back(this->multi_rays_->Ray(0));
    rays_2.push_back(this->multi_rays_->Ray(1));
    rays_3.push_back(this->multi_rays_->Ray(2));
    rays_4.push_back(this->multi_rays_->Ray(3));
    this->multi_rays_->Reset();

    // Push back the angles that are not transformed due to the pose because we create the pointcloud from the local frame.
    // The psoe-orientation only plays a part in setting the ray-positions.
    beta_1_values.push_back(beta1);
    beta_2_values.push_back(beta2);
    beta_3_values.push_back(beta3);
    beta_4_values.push_back(beta4);

    alpha_1_values.push_back(alpha1);
    alpha_2_values.push_back(alpha2);
    alpha_3_values.push_back(alpha3);
    alpha_4_values.push_back(alpha4);

  }
  ROS_INFO("Complete Set Rays");

  for (size_t i = 0; i < beta_1_values.size(); ++i)
  {
    this->horizontal_ray_angles_.push_back(beta_1_values[i]);
    this->vertical_ray_angles_.push_back(alpha_1_values[i]);
    eight_ray_pattern.left_upper_horizontal_ray_angles_.push_back(beta_1_values[i]);
    eight_ray_pattern.left_upper_vertical_ray_angles_.push_back(alpha_1_values[i]);
    eight_ray_pattern.addLeftUpperQuadrant(rays_1[i]);
    // ROS_ERROR_STREAM("addLeftUpperQuadrant" << i);
  }

  for (size_t i = 0; i < beta_3_values.size(); ++i)
  {
    this->horizontal_ray_angles_.push_back(beta_3_values[i]);
    this->vertical_ray_angles_.push_back(alpha_3_values[i]);
    eight_ray_pattern.right_upper_horizontal_ray_angles_.push_back(beta_3_values[i]);
    eight_ray_pattern.right_upper_vertical_ray_angles_.push_back(alpha_3_values[i]);
    eight_ray_pattern.addRightUpperQuadrant(rays_3[i]);
    //ROS_ERROR("addRightUpperQuadrant");
  }
  for (size_t i = 0; i < beta_2_values.size(); ++i)
  {
    this->horizontal_ray_angles_.push_back(beta_2_values[i]);
    this->vertical_ray_angles_.push_back(alpha_2_values[i]);
    eight_ray_pattern.left_lower_horizontal_ray_angles_.push_back(beta_2_values[i]);
    eight_ray_pattern.left_lower_vertical_ray_angles_.push_back(alpha_2_values[i]);
    eight_ray_pattern.addLeftLowerQuadrant(rays_2[i]);
    //ROS_ERROR("addLeftLowerQuadrant");
  }

  for (size_t i = 0; i < beta_4_values.size(); ++i)
  {
    this->horizontal_ray_angles_.push_back(beta_4_values[i]);
    this->vertical_ray_angles_.push_back(alpha_4_values[i]);
    eight_ray_pattern.right_lower_horizontal_ray_angles_.push_back(beta_4_values[i]);
    eight_ray_pattern.right_lower_vertical_ray_angles_.push_back(alpha_4_values[i]);
    eight_ray_pattern.addRightLowerQuadrant(rays_4[i]);
    //ROS_ERROR("addRightLowerQuadrant");
  }
  //--------------------------------------------------------------------------------------------------------------------

  this->double_ellipse_rays_.push_back(eight_ray_pattern);
  //std::cout << "Time 5: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - tstart).count() << " ms." << std::endl;
}
}
