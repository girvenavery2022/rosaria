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
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs//msg/float64.hpp"
#include "std_msgs//msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/empty.hpp"
#include "rosaria/BumperState.hpp"
#include "tf2/tf2.hpp"
#include "tf/transform_listener.h"  //for tf::getPrefixParam
#include <tf/transform_broadcaster.h>
#include "tf/transform_datatypes.h"
#include <dynamic_reconfigure/server.h>
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
    int Setup();
    void cmdvel_cb( const geometry_msgs::msg::TwistConstPtr &);
    void cmdvel_watchdog(const ros::TimerEvent& event);
    //void cmd_enable_motors_cb();
    //void cmd_disable_motors_cb();
    void spin();
    void publish();
    void sonarConnectCb();
    void dynamic_reconfigureCB(rosaria::RosAriaConfig &config, uint32_t level);
    void readParameters();

  protected:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr  pose_pub;
    rclcpp::Publisher<rosaria::BumperState>::SharedPtr  bumpers_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr  sonar_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr  sonar_pointcloud2_pub;
    //ros::Publisher voltage_pub;

    //ros::Publisher recharge_state_pub;
    std_msgs::msg::Int8 recharge_state;

    //ros::Publisher state_of_charge_pub;

    ros::Publisher motors_state_pub;
    std_msgs::msg::Bool motors_state;
    bool published_motors_state;

    ros::Subscriber cmdvel_sub;

    ros::ServiceServer enable_srv;
    ros::ServiceServer disable_srv;
    bool enable_motors_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool disable_motors_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    ros::Time veltime;
    ros::Timer cmdvel_watchdog_timer;
    ros::Duration cmdvel_timeout;

    std::string serial_port;
    int serial_baud;

    ArRobotConnector *conn;
    ArLaserConnector *laserConnector;
    ArRobot *robot;
    nav_msgs::msg::Odometry position;
    rosaria::BumperState bumpers;
    ArPose pos;
    ArFunctorC<RosAriaNode> myPublishCB;
    //ArRobot::ChargeState batteryCharge;

    //for odom->base_link transform
    tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::msg::TransformStamped odom_trans;
    
    std::string frame_id_odom;
    std::string frame_id_base_link;
    std::string frame_id_bumper;
    std::string frame_id_sonar;

    // flag indicating whether sonar was enabled or disabled on the robot
    bool sonar_enabled; 

    // enable and publish sonar topics. set to true when first subscriber connects, set to false when last subscriber disconnects. 
    bool publish_sonar; 
    bool publish_sonar_pointcloud2;

    // Debug Aria
    bool debug_aria;
    std::string aria_log_filename;
    
    // Robot Calibration Parameters (see readParameters() function)
    int TicksMM, DriftFactor, RevCount;  //If TicksMM or RevCount are <0, don't use. If DriftFactor is -99999, don't use (DriftFactor could be 0 or negative).
    
    // dynamic_reconfigure
    dynamic_reconfigure::Server<rosaria::RosAriaConfig> *dynamic_reconfigure_server;

    // whether to publish aria lasers
    bool publish_aria_lasers;
};