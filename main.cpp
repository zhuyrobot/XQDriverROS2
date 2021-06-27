#include "XQ4IO.h"

#include <iostream>
//#include <boost/thread.hpp>

//#include <json/json.h>
//#include <ros/ros.h>
//#include <ros/package.h>

//#include <tf2_ros/transform_listener.h>
//#include <std_msgs/String.h>
#include "DiffDriverController.h"
#include "StatusPublisher.h"
//#include "xiaoqiang_log/LogRecord.h"
//#include <cscv/macxx.h>
#include "maros.h"
//#include <rclcpp/rclcpp.hpp>

using namespace std;



inline bool exists(const std::string &name)
{
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
}

int main(int argc, char **argv)
{

    std::cout << "welcome to xiaoqiang serial server,please feel free at home!" << endl;

    //ros::init(argc, argv, "xqserial_server");
    //ros::start();

    rclcpp::init(argc, argv);
       


    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("xqserial_server");
    // 
    //��ȡ���ڲ���
    std::string port;
    //ros::param::param<std::string>("~port", port, "/dev/ttyUSB0");
	node->declare_parameter<std::string>("~port", "/dev/ttyUSB0");
	node->get_parameter<std::string>("~port",port);
    int baud;
    //ros::param::param<int>("~baud", baud, 115200);
	node->declare_parameter<int>("~baud", 115200);
	node->get_parameter<int>("~baud",baud);
    cout << "port:" << port << " baud:" << baud << endl;

    XQ4IO ioXQ;

    if (ioXQ.opened())
    {
        ioXQ.close();
    }
    else
    {
        if (ioXQ.open("/dev/ttyUSB0"))
        {
            spdlog::info("{} openned with success", ioXQ.name());
            //timerCarStatus->start(200);
        }
        else
        {
            spdlog::error("{} openned with failure", ioXQ.name());
        }
    }

    //��ȡС����е����
    double separation = 0, radius = 0;
    bool DebugFlag = false;
    //ros::param::param<double>("~wheel_separation", separation, 0.37);
    //ros::param::param<double>("~wheel_radius", radius, 0.0625);
    //ros::param::param<bool>("~debug_flag", DebugFlag, false);
	node->declare_parameter<double>("~wheel_separation", 0.37);
	node->get_parameter<double>("~wheel_separation",separation);
	
	node->declare_parameter<double>("~wheel_radius", 0.0625);
	node->get_parameter<double>("~wheel_radius",radius);
	
	node->declare_parameter<bool>("~debug_flag", false);
	node->get_parameter<bool>("~debug_flag",DebugFlag);



    //��ȡС�����Ʋ���
    double max_speed, r_min;
    max_speed = 2.0;
    r_min = 0.25;
    string cmd_topic;
    //ros::param::param<double>("~max_speed", max_speed, 2.0);
    //ros::param::param<double>("~r_min", r_min, 0.25);
    //ros::param::param<std::string>("~cmd_topic", cmd_topic, "cmd_vel");
     
	node->declare_parameter<double>("~max_speed", 2.0);
	node->get_parameter<double>("~max_speed",max_speed);
	
	node->declare_parameter<double>("~r_min", 0.0625);
	node->get_parameter<double>("~r_min",r_min);
	
	node->declare_parameter<std::string>("~cmd_topic", "cmd_vel");
	node->get_parameter<std::string>("~cmd_topic",cmd_topic);
     
    // ��ʼ��log�����ߺ�����������
    //ros::NodeHandle mNH;
    //ros::Publisher log_pub = mNH.advertise<xiaoqiang_log::LogRecord>("/xiaoqiang_log", 1, true);
    //ros::Publisher audio_pub = mNH.advertise<std_msgs::String>("/xiaoqiang_tts/text", 1, true);

    try
    {
        xqserial_server::StatusPublisher xq_status(separation, radius, &ioXQ, node);
        std::thread serial2statusThread(&xqserial_server::StatusPublisher::Update, &xq_status);
        serial2statusThread.detach();

        xqserial_server::DiffDriverController xq_diffdriver(max_speed, cmd_topic, &xq_status, &ioXQ, r_min, node);
        std::thread cmd2serialThread(&xqserial_server::DiffDriverController::run, &xq_diffdriver);
        cmd2serialThread.detach();

        // send test flag
        char debugFlagCmd[] = {(char)0xcd, (char)0xeb, (char)0xd7, (char)0x01, 'T'};
        if (DebugFlag)
        {
            //ROS_INFO_STREAM("Debug mode Enabled");
            //serial.write(debugFlagCmd, 5);
            ioXQ.write(debugFlagCmd, 5);
        }
        // send reset cmd
        char resetCmd[] = {(char)0xcd, (char)0xeb, (char)0xd7, (char)0x01, 'I'};
        //serial.write(resetCmd, 5);
        //ioXQ.write(resetCmd, 5);
        //ros::Duration(0.5).sleep();

        //ros::Rate r(100); //��������Ϊ50hz
        //����һ��100HZ��˯����ʱ
        rclcpp::WallRate loop_rate(100.0);

        while (rclcpp::ok())
        {
            if (/*ioXQ.errorStatus() || */ioXQ.opened() == false)
            {
                //ROS_ERROR_STREAM("Error: serial port closed unexpectedly");
                RCLCPP_INFO(node->get_logger(),"Error: serial port closed unexpectedly");
                break;
            }
            xq_status.Refresh(); //��ʱ����״̬
            loop_rate.sleep();
        }

    quit:
        //serial.close();
        ioXQ.close();
    }
    catch (std::exception &e)
    {
        //ROS_ERROR_STREAM("Open " << port << " failed.");
        //ROS_ERROR_STREAM("Exception: " << e.what());
        // ��鴮���豸�Ƿ����
        if (!exists(port))
        {
            //// ����������ʾ��Ϣ
            //std_msgs::String audio_msg;
            //audio_msg.data = "δ���ֵ��̴��ڣ����鴮������";
            //audio_pub.publish(audio_msg);
            //xiaoqiang_log::LogRecord log_record;
            //log_record.collection_name = "exception";
            //log_record.stamp = ros::Time::now();
            //Json::Value record;
            //record["type"] = "HARDWARE_ERROR";
            //record["info"] = "���̴����豸δ�ҵ�: " + port;
            //Json::FastWriter fastWriter;
            //log_record.record = fastWriter.write(record);
            //// ������־��Ϣ
            //log_pub.publish(log_record);
        }
        //ros::shutdown();
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::spin(node);

    //ros::shutdown();
    rclcpp::shutdown();

    return 0;
}
