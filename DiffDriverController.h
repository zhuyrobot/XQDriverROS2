#ifndef DIFFDRIVERCONTROLLER_H
#define DIFFDRIVERCONTROLLER_H
//#include <ros/ros.h>
#include "StatusPublisher.h"
//#include <std_msgs/Bool.h>
#include "XQ4IO.h"

//#include "galileo_serial_server/GalileoStatus.h"
//#include <cscv/macxx.h>
#include "maros.h"

//#include <rclcpp/rclcpp.hpp>
//#include <tf2_ros/transform_listener.h>

namespace xqserial_server
{

class DiffDriverController
{
  public:
    //DiffDriverController();
    DiffDriverController(double max_speed_, std::string cmd_topic_, StatusPublisher *xq_status_, XQ4IO *cmd_serial_, double r_min, std::shared_ptr<rclcpp::Node> nodeHandler);
    void run();
    void sendcmd(const geometry_msgs::msg::Twist::SharedPtr command);
    void imuCalibration(const std_msgs::msg::Bool::SharedPtr calFlag);
    void setStatusPtr(StatusPublisher &status);
    void updateMoveFlag(const std_msgs::msg::Bool::SharedPtr moveFlag);
    void updateBarDetectFlag(const std_msgs::msg::Bool::SharedPtr DetectFlag);
    //void UpdateNavStatus(const galileo_serial_server::GalileoStatus& current_receive_status);

  private:
    double max_wheelspeed; //单位为转每秒,只能为正数
    std::string cmd_topic;
    StatusPublisher *xq_status;
    XQ4IO::XQFrame* msg = 0;
    //CallbackAsyncSerial *cmd_serial;
    XQ4IO *cmd_serial;
    //boost::mutex mMutex;
    std::mutex mMutex;
    bool MoveFlag;
    //boost::mutex mStausMutex_;
    //galileo_serial_server::GalileoStatus galileoStatus_;
    float R_min_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_topicSub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr imu_calSub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr global_move_flagSub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr barDetectFlagSub;

    std::shared_ptr<rclcpp::Node> nodeHandler_;
};

} // namespace xqserial_server
#endif // DIFFDRIVERCONTROLLER_H
