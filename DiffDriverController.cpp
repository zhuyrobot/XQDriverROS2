#include "DiffDriverController.h"
#include <time.h>

namespace xqserial_server
{

//DiffDriverController::DiffDriverController()
//{
//    max_wheelspeed = 2.0;
//    cmd_topic = "cmd_vel";
//    xq_status = new StatusPublisher();
//    cmd_serial = NULL;
//    MoveFlag = true;
//    //galileoStatus_.mapStatus = 0;
//    R_min_ = 0.25;
//}

DiffDriverController::DiffDriverController(double max_speed_, std::string cmd_topic_, StatusPublisher *xq_status_, XQ4IO *cmd_serial_, double r_min, std::shared_ptr<rclcpp::Node> nodeHandler)
{
    MoveFlag = true;
    max_wheelspeed = max_speed_;
    cmd_topic = cmd_topic_;
    xq_status = xq_status_;
    cmd_serial = cmd_serial_;
    R_min_ = r_min;
    nodeHandler_ = nodeHandler;
}

void DiffDriverController::run()
{
    //ros::NodeHandle nodeHandler;
    //ros::Subscriber sub = nodeHandler.subscribe(cmd_topic, 1, &DiffDriverController::sendcmd, this);
    //ros::Subscriber sub2 = nodeHandler.subscribe("/imu_cal", 1, &DiffDriverController::imuCalibration, this);
    //ros::Subscriber sub3 = nodeHandler.subscribe("/global_move_flag", 1, &DiffDriverController::updateMoveFlag, this);
    //ros::Subscriber sub4 = nodeHandler.subscribe("/barDetectFlag", 1, &DiffDriverController::updateBarDetectFlag, this);
    //ros::Subscriber sub5 = nodeHandler.subscribe("/galileo/status", 1, &DiffDriverController::UpdateNavStatus, this);
    //ros::spin();

    //for (;;)
    //{
    //    cout << "run" << endl;
    //}
    //
  
    //for (;;)
    {
        cout<<"run"<<endl;
        cmd_topicSub = nodeHandler_->create_subscription<geometry_msgs::msg::Twist>(cmd_topic,10,std::bind(&DiffDriverController::sendcmd, this, std::placeholders::_1));

        imu_calSub = nodeHandler_->create_subscription<std_msgs::msg::Bool>("imu_cal",10,std::bind(&DiffDriverController::imuCalibration, this, std::placeholders::_1));
        global_move_flagSub = nodeHandler_->create_subscription<std_msgs::msg::Bool>("global_move_flag",10,std::bind(&DiffDriverController::updateMoveFlag, this, std::placeholders::_1));
        barDetectFlagSub = nodeHandler_->create_subscription<std_msgs::msg::Bool>("barDetectFlag",10,std::bind(&DiffDriverController::updateBarDetectFlag, this, std::placeholders::_1));

    //rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub5;
    //sub5 = nodeHandler->create_subscription<std_msgs::msg::Int8>("/galileo/status",10,std::bind(&DiffDriverController::UpdateNavStatus, this, std::placeholders::_1));
    }
    //rclcpp::spin(nodeHandler_);
}
void DiffDriverController::updateMoveFlag(const std_msgs::msg::Bool::SharedPtr moveFlag)
{
    //boost::mutex::scoped_lock lock(mMutex);
    MoveFlag = moveFlag->data;
}
void DiffDriverController::imuCalibration(const std_msgs::msg::Bool::SharedPtr calFlag)
{
    if (calFlag->data)
    {
        //下发底层ｉｍｕ标定命令
        char cmd_str[5] = {(char)0xcd, (char)0xeb, (char)0xd7, (char)0x01, (char)0x43};
        if (NULL != cmd_serial)
        {
            cmd_serial->write(cmd_str, 5);
        }
    }
}
void DiffDriverController::updateBarDetectFlag(const std_msgs::msg::Bool::SharedPtr DetectFlag)
{
    if (DetectFlag->data)
    {
        //下发底层红外开启命令
        char cmd_str[6] = {(char)0xcd, (char)0xeb, (char)0xd7, (char)0x02, (char)0x44, (char)0x01};
        if (NULL != cmd_serial)
        {
            cmd_serial->write(cmd_str, 6);
        }
    }
    else
    {
        //下发底层红外禁用命令
        char cmd_str[6] = {(char)0xcd, (char)0xeb, (char)0xd7, (char)0x02, (char)0x44, (char)0x00};
        if (NULL != cmd_serial)
        {
            cmd_serial->write(cmd_str, 6);
        }
    }
}
void DiffDriverController::sendcmd(const geometry_msgs::msg::Twist::SharedPtr command)
{
    static time_t t1 = time(NULL), t2;
    int i = 0, wheel_ppr = 1;
    double separation = 0, radius = 0, speed_lin = 0, speed_ang = 0, speed_temp[2];
    char speed[2] = {0, 0}; //右一左二
    char cmd_str[13] = {(char)0xcd, (char)0xeb, (char)0xd7, (char)0x09, (char)0x74, (char)0x53, (char)0x53, (char)0x53, (char)0x53, (char)0x00, (char)0x00, (char)0x00, (char)0x00};

    if (xq_status->get_status() == 0)
        return; //底层还在初始化
    separation = xq_status->get_wheel_separation();
    radius = xq_status->get_wheel_radius();
    wheel_ppr = xq_status->get_wheel_ppr();
    float x_filter = command->linear.x, z_filter = command->angular.z ;
    //{
    //  //先过滤速度
    //  //boost::mutex::scoped_lock lock(mStausMutex_);
    //  if(galileoStatus_.mapStatus == 1)
    //  {
    //    if(command.angular.z <-0.001 || command.angular.z>0.001 )
    //    {
    //      float R_now =  std::fabs(command.linear.x / command.angular.z);
    //      if(R_now < R_min_)
    //      {
    //        if(command.angular.z>0.001)
    //        {
    //          z_filter = std::fabs(x_filter/R_min_);
    //        }
    //        else
    //        {
    //          z_filter = -std::fabs(x_filter/R_min_);
    //        }
    //      }
    //    }
    //  }
    //}

    if(std::fabs(x_filter)<0.1)
    {
      if(z_filter>0.0002&&z_filter<0.15) z_filter=0.15;
      if(z_filter<-0.0002&&z_filter>-0.15) z_filter=-0.15;
    }
    if(x_filter>0 && x_filter<0.05) x_filter=0.05;
    if(x_filter<0 && x_filter>-0.05) x_filter=-0.05;
    //转换速度单位，由米转换成转
    speed_lin = x_filter / (2.0 * PI * radius);
    speed_ang = z_filter * separation / (2.0 * PI * radius);

    float scale = std::max(std::abs(speed_lin + speed_ang / 2.0), std::abs(speed_lin - speed_ang / 2.0)) / max_wheelspeed;
    if (scale > 1.0)
    {
        scale = 1.0 / scale;
    }
    else
    {
        scale = 1.0;
    }
    //转出最大速度百分比,并进行限幅
    speed_temp[0] = scale * (speed_lin + speed_ang / 2) / max_wheelspeed * 100.0;
    speed_temp[0] = std::min(speed_temp[0], 100.0);
    speed_temp[0] = std::max(-100.0, speed_temp[0]);

    speed_temp[1] = scale * (speed_lin - speed_ang / 2) / max_wheelspeed * 100.0;
    speed_temp[1] = std::min(speed_temp[1], 100.0);
    speed_temp[1] = std::max(-100.0, speed_temp[1]);

    std::cout<<"radius "<<radius<<std::endl;
    std::cout<<"ppr "<<wheel_ppr<<std::endl;
    std::cout<<"pwm "<<speed_temp[0]<<std::endl;
    //  command.linear.x/
    for (i = 0; i < 2; i++)
    {
        speed[i] = (int8_t)speed_temp[i];
        if (speed[i] < 0)
        {
            cmd_str[5 + i] = (char)0x42; //B
            cmd_str[9 + i] = -speed[i];
        }
        else if (speed[i] > 0)
        {
            cmd_str[5 + i] = (char)0x46; //F
            cmd_str[9 + i] = speed[i];
        }
        else
        {
            cmd_str[5 + i] = (char)0x53; //S
            cmd_str[9 + i] = (char)0x00;
        }
    }

     //std::cout<<"distance1 "<<xq_status->car_status.distance1<<std::endl;
     //std::cout<<"distance2 "<<xq_status->car_status.distance2<<std::endl;
     //std::cout<<"distance3 "<<xq_status->car_status.distance3<<std::endl;
     //if(xq_status->get_status()==2)
     //{
     //  //有障碍物
     //  if(xq_status->car_status.distance1<30&&xq_status->car_status.distance1>0&&cmd_str[6]==(char)0x46)
     //  {
     //    cmd_str[6]=(char)0x53;
     //  }
     //  if(xq_status->car_status.distance2<30&&xq_status->car_status.distance2>0&&cmd_str[5]==(char)0x46)
     //  {
     //    cmd_str[5]=(char)0x53;
     //  }
     //  if(xq_status->car_status.distance3<20&&xq_status->car_status.distance3>0&&(cmd_str[5]==(char)0x42||cmd_str[6]==(char)0x42))
     //  {
     //    cmd_str[5]=(char)0x53;
     //    cmd_str[6]=(char)0x53;
     //  }
     //  if(xq_status->car_status.distance1<15&&xq_status->car_status.distance1>0&&(cmd_str[5]==(char)0x46||cmd_str[6]==(char)0x46))
     //  {
     //    cmd_str[5]=(char)0x53;
     //    cmd_str[6]=(char)0x53;
     //  }
     //  if(xq_status->car_status.distance2<15&&xq_status->car_status.distance2>0&&(cmd_str[5]==(char)0x46||cmd_str[6]==(char)0x46))
     //  {
     //    cmd_str[5]=(char)0x53;
     //    cmd_str[6]=(char)0x53;
     //  }
     //}

    //boost::mutex::scoped_lock lock(mMutex);
    lock_guard<std::mutex> lock(mMutex);
    if (!MoveFlag)
    {
        cmd_str[5] = (char)0x53;
        cmd_str[6] = (char)0x53;
    }
    if (NULL != cmd_serial)
    {
        cmd_serial->write(cmd_str, 13);
    }

    //command.linear.x
}

//void DiffDriverController::UpdateNavStatus(const galileo_serial_server::GalileoStatus& current_receive_status)
//{
//    boost::mutex::scoped_lock lock(mStausMutex_);
//    galileoStatus_.navStatus = current_receive_status.navStatus;
//    galileoStatus_.visualStatus = current_receive_status.visualStatus;
//    galileoStatus_.chargeStatus = current_receive_status.chargeStatus;
//    galileoStatus_.mapStatus = current_receive_status.mapStatus;
//
//}

} // namespace xqserial_server
