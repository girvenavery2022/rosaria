#include <stdio.h>
#include <math.h>
#ifdef ADEPT_PKG
  #include <Aria.h>
  #include <ArRobotConfigPacketReader.h> // todo remove after ArRobotConfig implemented in AriaCoda
#else
  #include <Aria/Aria.h>
  #include <Aria/ArRobotConfigPacketReader.h> // todo remove after ArRobotConfig implemented in AriaCoda
#endif
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs//msg/float64.hpp"
#include "std_msgs//msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "rosaria/BumperState.hpp"
#include <rosaria/RosAriaConfig.h>

#include "LaserPublisher.h"

#include <sstream>

/** @brief Node that interfaces between ROS and mobile robot base features via ARIA library. 

    RosAriaNode will use ARIA to connect to a robot controller (configure via
    ~port parameter), either direct serial connection or over the network.  It 
    runs ARIA's robot communications cycle in a background thread, and
    as part of that cycle (a sensor interpretation task which calls RosAriaNode::publish()),
    it  publishes various topics with newly received robot
    data.  It also sends velocity commands to the robot when received in the
    cmd_vel topic, and handles dynamic_reconfigure and Service requests.

    For more information about ARIA see
    http://robots.mobilerobots.com/wiki/Aria.

    RosAria uses the roscpp client library, see http://www.ros.org/wiki/roscpp for
    information, tutorials and documentation.
*/
class RosAriaNode : public rclcpp::Node
{
  public:
    RosAriaNode(rclcpp::NodeOptions options);
    virtual ~RosAriaNode();
    
  public:
    /**
     * @brief 
     * 
     * @return int 
     */
    int setup();

    /**
     * @brief 
     * 
     * @param msg 
     */
    void cmdvel_cb(const geometry_msgs::msg::Twist::SharedPtr msg);

    /**
     * @brief 
     * 
     */
    void cmdvel_watchdog();

    /**
     * @brief 
     * 
     * @param request 
     * @param response 
     * @return true 
     * @return false 
     */
    bool enable_motors_cb(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
          std::shared_ptr<std_srvs::srv::Empty::Response> response);

    /**
     * @brief 
     * 
     * @param request 
     * @param response 
     * @return true 
     * @return false 
     */
    bool disable_motors_cb(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
          std::shared_ptr<std_srvs::srv::Empty::Response> response);
    
    /**
     * @brief 
     * 
     */
    void publish_odom();

    /**
     * @brief 
     * 
     */
    void publish_transform();

    /**
     * @brief 
     * 
     */
    void publish_bumpers();

    /**
     * @brief 
     * 
     */
    void publish_sonar();
    void publish();

    /**
     * @brief 
     * 
     */
    void readParameters();

  protected:
    // ros2 publisher, subscribers and services
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr  pose_pub;
    rclcpp::Publisher<rosaria::BumperState>::SharedPtr  bumpers_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr  sonar_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr voltage_pub;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr recharge_state_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr state_of_charge_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motors_state_pub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped> cmdvel_sub;

    // odom->base_link transform broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ros2 messages and services
    geometry_msgs::msg::TransformStamped odom_trans;
    std_msgs::msg::Int8 recharge_state;
    std_msgs::msg::Bool motors_state;
    nav_msgs::msg::Odometry position;
    rosaria::BumperState bumpers;
    std_srvs::srv::Empty enable_srv;
    std_srvs::srv::Empty disable_srv;

    // rclcpp time and timers
    rclcpp::Time veltime;
    rclcpp::TimerBase::SharedPtr cmdvel_watchdog_timer;
    rclcpp::TimerBase::SharedPtr cmdvel_timeout;

    // Pioneer robot defenition
    ArRobotConnector *conn;
    ArLaserConnector *laserConnector;
    ArRobot *robot;
    ArPose pos;
    ArFunctorC<RosAriaNode> myPublishCB;
    ArRobot::ChargeState batteryCharge;
    
    // frame ids for the pioneer
    std::string frame_id_odom;
    std::string frame_id_base;
    std::string frame_id_bumper;
    std::string frame_id_sonar;

    // flag indicating whether sonar was enabled or disabled on the robot
    bool sonar_enabled; 

    // enable and publish sonar topics. set to true when first subscriber connects, set to false when last subscriber disconnects. 
    bool publish_sonar; 

    // Debug Aria
    bool debug_aria;
    std::string aria_log_filename;
    
    // Robot Calibration Parameters (see readParameters() function)
    int TicksMM, DriftFactor, RevCount;  //If TicksMM or RevCount are <0, don't use. If DriftFactor is -99999, don't use (DriftFactor could be 0 or negative).
    
    // whether to publish aria lasers
    bool publish_aria_lasers;

    bool published_motors_state;

    std::string serial_port;
    int serial_baud;
};