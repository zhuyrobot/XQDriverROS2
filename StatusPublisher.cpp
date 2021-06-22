#include "StatusPublisher.h"
#include <memory.h>
#include <math.h>
#include <stdlib.h>

#include "tf2_msgs/msg/tf_message.hpp"
#include <tf2/LinearMath/Quaternion.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/transform_broadcaster.h"

#define DISABLE 0
#define ENABLE 1

namespace xqserial_server
{
//typedef sensor_msgs::msg::PointCloud2 PointCloud;

void StatusPublisher::StatusPublisher1()
{
  mbUpdated = false;
  wheel_separation = 0.37;
  wheel_radius = 0.06;

  CarPos2D.x = 0.0;
  CarPos2D.y = 0.0;
  CarPos2D.theta = 0.0;

  CarTwist.linear.x = 0.0;
  CarTwist.linear.y = 0.0;
  CarTwist.linear.z = 0.0;
  CarTwist.angular.x = 0.0;
  CarTwist.angular.y = 0.0;
  CarTwist.angular.z = 0.0;

  CarPower.data = 0.0;

  //int i = 0;
  //int *status;
  //status = (int *)&msg;
  //for (i = 0; i < 23; i++)
  //{
  //  status[i] = 0;
  //}
  //msg->encoder_ppr = 4 * 12 * 64;

  //mPose2DPub = mNH.advertise<geometry_msgs::Pose2D>("xqserial_server/Pose2D", 1, true);
  //mStatusFlagPub = mNH.advertise<std_msgs::Int32>("xqserial_server/StatusFlag", 1, true);
  //mTwistPub = mNH.advertise<geometry_msgs::Twist>("xqserial_server/Twist", 1, true);
  //mPowerPub = mNH.advertise<std_msgs::Float64>("xqserial_server/Power", 1, true);
  //mOdomPub = mNH.advertise<nav_msgs::Odometry>("xqserial_server/Odom", 1, true);
  //pub_barpoint_cloud_ = mNH.advertise<PointCloud>("kinect/barpoints", 1, true);
  //pub_clearpoint_cloud_ = mNH.advertise<PointCloud>("kinect/clearpoints", 1, true);
  //mIMUPub = mNH.advertise<sensor_msgs::Imu>("xqserial_server/IMU", 1, true);
  
        
  /* static tf::TransformBroadcaster br;
   tf::Quaternion q;
   tf::Transform transform;
   transform.setOrigin( tf::Vector3(0.0, 0.0, 0.13) );//摄像头距离地面高度13cm
   q.setRPY(0, 0, 0);
   transform.setRotation(q);
   br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", "base_link"));
   */
  //base_time_ = ros::Time::now().toSec();
  //base_time_ = mNH->now().seconds();
  //current_time = mNH->now();
}

StatusPublisher::StatusPublisher(double separation, double radius, XQ4IO* sta_serial_, std::shared_ptr<rclcpp::Node> nodeHandler)
{
  this->StatusPublisher1();
  wheel_separation = separation;
  wheel_radius = radius;
  sta_serial = sta_serial_;

  nodeHandler_ = nodeHandler;
  mPose2DPub = nodeHandler_->create_publisher<geometry_msgs::msg::Pose2D>("xqserial_server/Pose2D", 10);
  mStatusFlagPub = nodeHandler_->create_publisher<std_msgs::msg::Int32>("xqserial_server/StatusFlag", 10);
  mTwistPub = nodeHandler_->create_publisher<geometry_msgs::msg::Twist>("xqserial_server/Twist", 10);
  mPowerPub = nodeHandler_->create_publisher<std_msgs::msg::Float64>("xqserial_server/Power", 10);
  mOdomPub = nodeHandler_->create_publisher<nav_msgs::msg::Odometry>("xqserial_server/Odom", 10);
  pub_barpoint_cloud_ = nodeHandler_->create_publisher<sensor_msgs::msg::PointCloud2>("kinect/barpoints", 10);
  pub_clearpoint_cloud_ = nodeHandler_->create_publisher<sensor_msgs::msg::PointCloud2>("kinect/clearpoints", 10);
  mIMUPub = nodeHandler_->create_publisher<sensor_msgs::msg::Imu>("xqserial_server/IMU", 10);
}

void StatusPublisher::Update()
{
    //Read loop in spawned thread
    for (;;)
    {
        sta_serial->getStatus(&msg);
        //if (msg != 0) mbUpdated = true;
        cout << "Update" << endl;
    }

  return;
}

void StatusPublisher::Refresh()
{
  //boost::mutex::scoped_lock lock(mMutex);
  lock_guard<std::mutex> lock(mMutex);
  static double theta_last = 0.0;
  static unsigned int ii = 0;
  static bool theta_updateflag = false;
  ii++;
  //std::cout<<"runR"<< mbUpdated<<std::endl;

  cout << "Refresh" <<endl;

  //CarPos2D.x = 0.0;
  //CarPos2D.y = 0.0;
  //CarPos2D.theta = 0.0;
  //mPose2DPub->publish(CarPos2D);

  //mbUpdated = false;
  if (msg != 0)
  //if (false)
  {
    // Time
    //ros::Time current_time;
    //rclcpp::Time current_time;

    if (msg->status == 0)
    {
      theta_updateflag = false;
    }
    else
    {
      theta_updateflag = true;
    }
    //pose
    double delta_car, delta_x, delta_y, delta_theta, var_len, var_angle;

    var_len = (50.0f / msg->encoder_ppr * 2.0f * PI * wheel_radius) * (50.0f / msg->encoder_ppr * 2.0f * PI * wheel_radius);
    var_angle = (0.01f / 180.0f * PI) * (0.01f / 180.0f * PI);

    delta_car = (msg->encoder_delta_r + msg->encoder_delta_l) / 2.0f * 1.0f / msg->encoder_ppr * 2.0f * PI * wheel_radius;
    if (delta_car > 0.05 || delta_car < -0.05)
    {
      // std::cout<<"get you!"<<std::endl;
      delta_car = 0;
    }
     if(ii%50==0|| msg->encoder_delta_car>3000|| msg->encoder_delta_car<-3000)
     {
       std::cout<<"delta_encoder_car:"<< msg->encoder_delta_car <<std::endl;
       std::cout<<"delta_encoder_r:"<< msg->encoder_delta_r <<std::endl;
       std::cout<<"delta_encoder_l:"<< msg->encoder_delta_l <<std::endl;
       std::cout<<"ppr:"<< msg->encoder_ppr <<std::endl;
       std::cout<<"delta_car:"<< delta_car <<std::endl;
     }
    delta_x = delta_car * cos(CarPos2D.theta * PI / 180.0f);
    delta_y = delta_car * sin(CarPos2D.theta * PI / 180.0f);

    delta_theta = msg->theta - theta_last;
    theta_last = msg->theta;
    if (delta_theta > 270)
      delta_theta -= 360;
    if (delta_theta < -270)
      delta_theta += 360;

    if ((!theta_updateflag) || delta_theta > 20 || delta_theta < -20)
    {
      delta_theta = 0;
    }
    CarPos2D.x += delta_x;
    CarPos2D.y += delta_y;
    CarPos2D.theta += delta_theta;

    if (CarPos2D.theta > 360.0)
      CarPos2D.theta -= 360;
    if (CarPos2D.theta < 0.0)
      CarPos2D.theta += 360;

    mPose2DPub->publish(CarPos2D);

    //flag
    std_msgs::msg::Int32 flag;
    flag.data = msg->status;
    //底层障碍物信息
    if ((msg->distance1 + msg->distance2 + msg->distance3 + msg->distance4) > 0.1 && (msg->distance1 + msg->distance2 + msg->distance3 + msg->distance4) < 5.0)
    {
      //有障碍物
      flag.data = 2;
    }
    mStatusFlagPub->publish(flag);

    int barArea_nums = 0;
    int clearArea_nums = 0;
    if (msg->distance1 > 0.1)
    {
      barArea_nums += 3;
    }
    else
    {
      clearArea_nums += 6;
    }
    if (msg->distance2 > 0.1)
    {
      barArea_nums += 3;
    }
    else
    {
      clearArea_nums += 6;
    }
    if (msg->distance4 > 0.1)
    {
      barArea_nums += 3;
    }
    else
    {
      clearArea_nums += 6;
    }

    if (barArea_nums > 0)
    {
      //发布雷区
      sensor_msgs::msg::PointCloud2::Ptr barcloud_msg(new sensor_msgs::msg::PointCloud2);
      //barcloud_msg->header.stamp = current_time.fromSec(base_time_);
      barcloud_msg->header.stamp = current_time;
      barcloud_msg->height = 1;
      barcloud_msg->width = barArea_nums;
      barcloud_msg->is_dense = true;
      barcloud_msg->is_bigendian = false;
      barcloud_msg->header.frame_id = "kinect_link_new";
      sensor_msgs::PointCloud2Modifier pcd_modifier1(*barcloud_msg);
      pcd_modifier1.setPointCloud2FieldsByString(1, "xyz");
      sensor_msgs::PointCloud2Iterator<float> bariter_x(*barcloud_msg, "x");
      sensor_msgs::PointCloud2Iterator<float> bariter_y(*barcloud_msg, "y");
      sensor_msgs::PointCloud2Iterator<float> bariter_z(*barcloud_msg, "z");
      if (msg->distance2 > 0.1)
      {
        for (int k = 0; k < 3; k++, ++bariter_x, ++bariter_y, ++bariter_z)
        {
          *bariter_x = 0.2;
          *bariter_y = -0.10 - k * 0.05;
          *bariter_z = 0.15;
        }
      }
      if (msg->distance4 > 0.1)
      {
        for (int k = 0; k < 3; k++, ++bariter_x, ++bariter_y, ++bariter_z)
        {
          *bariter_x = 0.2;
          *bariter_y = -0.1 + k * 0.05;
          *bariter_z = 0.15;
        }
      }
      if (msg->distance1 > 0.1)
      {
        for (int k = 0; k < 3; k++, ++bariter_x, ++bariter_y, ++bariter_z)
        {
          *bariter_x = 0.2;
          *bariter_y = 0.05 + k * 0.05;
          *bariter_z = 0.15;
        }
      }
      if (ii % 5 == 0)
      {
        //pub_barpoint_cloud_->publish(barcloud_msg);
        pub_barpoint_cloud_->publish(*barcloud_msg);
      }
    }
    if (clearArea_nums > 0)
    {
      //发布雷区
      //sensor_msgs::msg::PointCloud2::Ptr clearcloud_msg(new sensor_msgs::msg::PointCloud2);
      ////clearcloud_msg->header.stamp = current_time.fromSec(base_time_);
      //clearcloud_msg->header.stamp = current_time;
      //clearcloud_msg->height = 1;
      //clearcloud_msg->width = clearArea_nums;
      //clearcloud_msg->is_dense = true;
      //clearcloud_msg->is_bigendian = false;
      //clearcloud_msg->header.frame_id = "kinect_link_new";

      sensor_msgs::msg::PointCloud2::Ptr clearcloud_msg(new sensor_msgs::msg::PointCloud2);
      clearcloud_msg->header.stamp = current_time;
      clearcloud_msg->height = 1;
      clearcloud_msg->width = clearArea_nums;
      clearcloud_msg->is_dense = true;
      clearcloud_msg->is_bigendian = false;
      clearcloud_msg->header.frame_id = "kinect_link_new";

      sensor_msgs::PointCloud2Modifier pcd_modifier1(*clearcloud_msg);
      pcd_modifier1.setPointCloud2FieldsByString(1, "xyz");
      sensor_msgs::PointCloud2Iterator<float> cleariter_x(*clearcloud_msg, "x");
      sensor_msgs::PointCloud2Iterator<float> cleariter_y(*clearcloud_msg, "y");
      sensor_msgs::PointCloud2Iterator<float> cleariter_z(*clearcloud_msg, "z");
      if (msg->distance2 < 0.1)
      {
        for (int k = 0; k < 3; k++, ++cleariter_x, ++cleariter_y, ++cleariter_z)
        {
          *cleariter_x = 0.2;
          *cleariter_y = -0.1 - k * 0.05;
          *cleariter_z = 0.0;
        }
        for (int k = 0; k < 3; k++, ++cleariter_x, ++cleariter_y, ++cleariter_z)
        {
          *cleariter_x = 0.15;
          *cleariter_y = -0.1 - k * 0.05;
          *cleariter_z = 0.0;
        }
      }
      if (msg->distance4 < 0.1)
      {
        for (int k = 0; k < 3; k++, ++cleariter_x, ++cleariter_y, ++cleariter_z)
        {
          *cleariter_x = 0.2;
          *cleariter_y = -0.1 + k * 0.05;
          *cleariter_z = 0.0;
        }
        for (int k = 0; k < 3; k++, ++cleariter_x, ++cleariter_y, ++cleariter_z)
        {
          *cleariter_x = 0.15;
          *cleariter_y = -0.1 + k * 0.05;
          *cleariter_z = 0.0;
        }
      }
      if (msg->distance1 < 0.1)
      {
        for (int k = 0; k < 3; k++, ++cleariter_x, ++cleariter_y, ++cleariter_z)
        {
          *cleariter_x = 0.2;
          *cleariter_y = 0.05 + k * 0.05;
          *cleariter_z = 0.0;
        }
        for (int k = 0; k < 3; k++, ++cleariter_x, ++cleariter_y, ++cleariter_z)
        {
          *cleariter_x = 0.15;
          *cleariter_y = 0.05 + k * 0.05;
          *cleariter_z = 0.0;
        }
      }
      if (ii % 5 == 0)
      {
        //pub_clearpoint_cloud_->publish(clearcloud_msg);
          pub_clearpoint_cloud_->publish(*clearcloud_msg);
      }
    }

    //Twist
    double angle_speed;
    CarTwist.linear.x = delta_car * 50.0f;
    angle_speed = -msg->imudata[5];
    CarTwist.angular.z = angle_speed * PI / 180.0f;
    mTwistPub->publish(CarTwist);

    CarPower.data = msg->power;
    mPowerPub->publish(CarPower);

    //CarOdom.header.stamp = current_time.fromSec(base_time_);
    CarOdom.header.stamp = current_time;
    CarOdom.header.frame_id = "odom";
    CarOdom.pose.pose.position.x = CarPos2D.x;
    CarOdom.pose.pose.position.y = CarPos2D.y;
    CarOdom.pose.pose.position.z = 0.0f;
    //geometry_msgs::msg::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(CarPos2D.theta / 180.0f * PI);
    //CarOdom.pose.pose.orientation = odom_quat;
    //CarOdom.pose.covariance = boost::assign::list_of(var_len)(0)(0)(0)(0)(0)(0)(var_len)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(var_angle);
    geometry_msgs::msg::Quaternion odom_quat;
    tf2::Quaternion odom_q;
    odom_q.setRPY(0, 0, CarPos2D.theta / 180 * PI);
    odom_quat.w = odom_q.getW();
    odom_quat.x = odom_q.getX();
    odom_quat.y = odom_q.getY();
    odom_quat.z = odom_q.getZ(); 
    CarOdom.pose.pose.orientation = odom_quat;
    CarOdom.pose.covariance = {var_len,0,0,0,0,0,0,var_len,0,0,0,0,0,0,999,0,0,0,0,0,0,999,0,0,0,0,0,0,999,0,0,0,0,0,0,var_angle};
    CarOdom.child_frame_id = "base_footprint";
    CarOdom.twist.twist.linear.x = CarTwist.linear.x; // * cos(CarPos2D.theta* PI / 180.0f);
    CarOdom.twist.twist.linear.y = CarTwist.linear.y; // * sin(CarPos2D.theta* PI / 180.0f);
    CarOdom.twist.twist.angular.z = CarTwist.angular.z;
    //CarOdom.twist.covariance = boost::assign::list_of(var_len)(0)(0)(0)(0)(0)(0)(var_len)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(999)(0)(0)(0)(0)(0)(0)(var_angle);
    CarOdom.twist.covariance = { var_len,0,0,0,0,0,0,var_len,0,0,0,0,0,0,999,0,0,0,0,0,0,999,0,0,0,0,0,0,999,0,0,0,0,0,0,var_angle };
    mOdomPub->publish(CarOdom);

    //publish IMU
    
    tf2::Quaternion q_imu;
    q_imu.setRPY(0, 0, msg->theta / 180.0 * PI);
    //CarIMU.header.stamp = current_time;
    CarIMU.header.stamp = current_time;
    CarIMU.header.frame_id = "imu";
    CarIMU.orientation.x = q_imu.x();
    CarIMU.orientation.y = q_imu.y();
    CarIMU.orientation.z = q_imu.z();
    CarIMU.orientation.w = q_imu.w();

    CarIMU.angular_velocity.x = -msg->imudata[3] * PI / 180.0f;
    CarIMU.angular_velocity.y = msg->imudata[4] * PI / 180.0f;
    CarIMU.angular_velocity.z = -msg->imudata[5] * PI / 180.0f;
    CarIMU.linear_acceleration.x = -msg->imudata[0];
    CarIMU.linear_acceleration.y = msg->imudata[1];
    CarIMU.linear_acceleration.z = -msg->imudata[2];

    mIMUPub->publish(CarIMU);

    // pub transform

    //static tf::TransformBroadcaster br;
    //tf::Quaternion q;
    //tf::Transform transform;
    //transform.setOrigin(tf::Vector3(CarPos2D.x, CarPos2D.y, 0.0));
    //q.setRPY(0, 0, CarPos2D.theta / 180 * PI);
    //transform.setRotation(q);
    //br.sendTransform(tf::StampedTransform(transform, current_time.fromSec(base_time_), "odom", "base_footprint"));

    //ros::spinOnce();

    //rclcpp::Node::SharedPtr node_handle = nullptr;
    static tf2_ros::TransformBroadcaster br(nodeHandler_);
    geometry_msgs::msg::TransformStamped static_transformStamped; // 用于记录转换关系的消息
    static_transformStamped.transform.translation.x = CarPos2D.x;
    static_transformStamped.transform.translation.y = CarPos2D.y;
    static_transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, CarPos2D.theta / 180 * PI);

    geometry_msgs::msg::Quaternion out;
    out.w = q.getW();
    out.x = q.getX();
    out.y = q.getY();
    out.z = q.getZ();

    //static_transformStamped.transform.rotation = tf2::toMsg(q);
    static_transformStamped.transform.rotation = out;
    static_transformStamped.header.stamp = current_time;
    static_transformStamped.header.frame_id = "odom";
    static_transformStamped.child_frame_id = "base_footprint";
    //发布坐标变换
    br.sendTransform(static_transformStamped);

    rclcpp::spin_some(nodeHandler_);

    mbUpdated = false;
  }
}

double StatusPublisher::get_wheel_separation()
{
  return wheel_separation;
}

double StatusPublisher::get_wheel_radius()
{
  return wheel_radius;
}

int StatusPublisher::get_wheel_ppr()
{
  return msg->encoder_ppr;
}

void StatusPublisher::get_wheel_speed(double speed[2])
{
  //右一左二
  speed[0] = msg->omga_r / msg->encoder_ppr * 2.0 * PI * wheel_radius;
  speed[1] = msg->omga_l / msg->encoder_ppr * 2.0 * PI * wheel_radius;
}

geometry_msgs::msg::Pose2D StatusPublisher::get_CarPos2D()
{
  return CarPos2D;
}

geometry_msgs::msg::Twist StatusPublisher::get_CarTwist()
{
  return CarTwist;
}

std_msgs::msg::Float64 StatusPublisher::get_power()
{
  return CarPower;
}

nav_msgs::msg::Odometry StatusPublisher::get_odom()
{
  return CarOdom;
}
int StatusPublisher::get_status()
{
  return msg->status;
}

} //namespace xqserial_server
