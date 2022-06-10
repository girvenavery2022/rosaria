#include "rosaria/RosAria.hpp"

RosAriaNode::RosAriaNode(rclcpp::NodeOptions options)
: Node("lidar_processor", options)
{
  pose_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom",10);
  bumpers_pub = this->create_publisher<rosaria::BumperState>("/bumper_state",10);
  sonar_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sonar", 10);

  voltage_pub = this->create_publisher<std_msgs::msg::Float64>("/battery_voltage", 10);
  recharge_state_pub = this->create_publisher<std_msgs::msg:Int8>("/battery_recharge_state", 10);
  state_of_charge_pub = this->create_publisher<std_msgs::msg::Float32>("/battery_state_of_charge", 10);
  motors_state_pub = this->create_publisher<std_msgs::msg::Bool>("/motors_state", 10);

  cmdvel_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/cmd_vel", 10,
    std::bind(&RosAriaNode::cmdvel_cb, this, std::placeholders::_1));

  // port and baud
  serial_port = this->declare_parameter("port", "/dev/ttyUSB0");
  RCLCPP_INFO(this->get_logger(), "RosAria: set port: [%s]", serial_port.c_str());

  serial_baud = this->declare_parameter("baud_rate", 0);
  if(serial_baud != 0)
    RCLCPP_INFO(this->get_logger, "RosAria: set serial port baud rate %d", serial_baud);

  // handle debugging more elegantly
  debug_aria = this->declare_parameter("debug_aria", false);
  aria_log_filename = this->declare_parameter("baud_rate", std::string("Aria.log"));

  // whether to connect to lasers using aria
  publish_aria_lasers = this->declare_parameter("publish_aria_lasers", false);

  // Get frame_ids to use.
  frame_id_odom = this->declare_parameter("odom_frame_id", std::string("odom"));
  frame_id_base = this->declare_parameter("base_frame_id", std::string("base_footprint"));
  frame_id_bumper = this->declare_parameter("bumpers_frame_id", std::string("bumpers"));
  frame_id_sonar = this->declare_parameter("sonar_frame_id", std::string("sonar"));
 
  recharge_state.data = -2;
  motors_state.data = false;
  published_motors_state = false;

  // advertise enable/disable services
  enable_srv = n.advertiseService("enable_motors", &RosAriaNode::enable_motors_cb, this);
  disable_srv = n.advertiseService("disable_motors", &RosAriaNode::disable_motors_cb, this);
  
  veltime = rclcpp->get_clock()->now()
}

RosAriaNode::~RosAriaNode()
{
  // disable motors and sonar.
  robot->disableMotors();
  robot->disableSonar();

  robot->stopRunning();
  robot->waitForRunExit();
  Aria::shutdown();
}

void RosAriaNode::readParameters()
{
  // Robot Parameters. If a parameter was given and is nonzero, set it now.
  // Otherwise, get default value for this robot (from getOrigRobotConfig()).
  // Parameter values are stored in member variables for possible later use by the user with dynamic reconfigure.
  robot->lock();
  ros::NodeHandle n_("~");
  if (n_.getParam("TicksMM", TicksMM) && TicksMM > 0)
  {
    RCLCPP_INFO("Setting robot TicksMM from ROS Parameter: %d", TicksMM);
    robot->comInt(93, TicksMM);
  }
  else
  {
    TicksMM = robot->getOrigRobotConfig()->getTicksMM();
    RCLCPP_INFO("This robot's TicksMM parameter: %d", TicksMM);
    //n_.setParam( "TicksMM", TicksMM);
  }
  
  if (n_.getParam("DriftFactor", DriftFactor) && DriftFactor != -99999)
  {
    RCLCPP_INFO("Setting robot DriftFactor from ROS Parameter: %d", DriftFactor);
    robot->comInt(89, DriftFactor);
  }
  else
  {
    DriftFactor = robot->getOrigRobotConfig()->getDriftFactor();
    RCLCPP_INFO("This robot's DriftFactor parameter: %d", DriftFactor);
    //n_.setParam( "DriftFactor", DriftFactor);
  }
  
  if (n_.getParam("RevCount", RevCount) && RevCount > 0)
  {
    RCLCPP_INFO("Setting robot RevCount from ROS Parameter: %d", RevCount);
    robot->comInt(88, RevCount);
  }
  else
  {
    RevCount = robot->getOrigRobotConfig()->getRevCount();
    RCLCPP_INFO("This robot's RevCount parameter: %d", RevCount);
    //n_.setParam( "RevCount", RevCount);
  }
  robot->unlock();
}

void RosAriaNode::dynamic_reconfigureCB(rosaria::RosAriaConfig &config, uint32_t level)
{
  //
  // Odometry Settings
  //
  robot->lock();
  if(TicksMM != config.TicksMM && config.TicksMM > 0)
  {
    RCLCPP_INFO("Setting TicksMM from Dynamic Reconfigure: %d -> %d ", TicksMM, config.TicksMM);
    TicksMM = config.TicksMM;
    robot->comInt(93, TicksMM);
  }
  
  if(DriftFactor != config.DriftFactor && config.DriftFactor != -99999) 
  {
    RCLCPP_INFO("Setting DriftFactor from Dynamic Reconfigure: %d -> %d ", DriftFactor, config.DriftFactor);
    DriftFactor = config.DriftFactor;
    robot->comInt(89, DriftFactor);
  }
  
  if(RevCount != config.RevCount && config.RevCount > 0)
  {
    RCLCPP_INFO("Setting RevCount from Dynamic Reconfigure: %d -> %d ", RevCount, config.RevCount);
    RevCount = config.RevCount;
    robot->comInt(88, RevCount);
  }
  
  //
  // Acceleration Parameters
  //
  int value;
  value = config.trans_accel * 1000;
  if(value != robot->getTransAccel() && value > 0)
  {
    RCLCPP_INFO("Setting TransAccel from Dynamic Reconfigure: %d", value);
    robot->setTransAccel(value);
  }
  
  value = config.trans_decel * 1000;
  if(value != robot->getTransDecel() && value > 0)
  {
    RCLCPP_INFO("Setting TransDecel from Dynamic Reconfigure: %d", value);
    robot->setTransDecel(value);
  } 
  
  value = config.lat_accel * 1000;
  if(value != robot->getLatAccel() && value > 0)
  {
    RCLCPP_INFO("Setting LatAccel from Dynamic Reconfigure: %d", value);
    if (robot->getAbsoluteMaxLatAccel() > 0 )
      robot->setLatAccel(value);
  }
  
  value = config.lat_decel * 1000;
  if(value != robot->getLatDecel() && value > 0)
  {
    RCLCPP_INFO("Setting LatDecel from Dynamic Reconfigure: %d", value);
    if (robot->getAbsoluteMaxLatDecel() > 0 )
      robot->setLatDecel(value);
  }
  
  value = config.rot_accel * 180/M_PI;
  if(value != robot->getRotAccel() && value > 0)
  {
    RCLCPP_INFO("Setting RotAccel from Dynamic Reconfigure: %d", value);
    robot->setRotAccel(value);
  }
  
  value = config.rot_decel * 180/M_PI;
  if(value != robot->getRotDecel() && value > 0)
  {
    RCLCPP_INFO("Setting RotDecel from Dynamic Reconfigure: %d", value);
    robot->setRotDecel(value);
  } 
  robot->unlock();
}

/// Called when another node subscribes or unsubscribes from sonar topic.
void RosAriaNode::sonarConnectCb()
{
  publish_sonar = (sonar_pub.getNumSubscribers() > 0);
  publish_sonar_pointcloud2 = (sonar_pointcloud2_pub.getNumSubscribers() > 0);
  robot->lock();
  if (publish_sonar || publish_sonar_pointcloud2)
  {
    robot->enableSonar();
    sonar_enabled = false;
  }
  else if(!publish_sonar && !publish_sonar_pointcloud2)
  {
    robot->disableSonar();
    sonar_enabled = true;
  }
  robot->unlock();
}

int RosAriaNode::Setup()
{
  // Note, various objects are allocated here which are never deleted (freed), since Setup() is only supposed to be
  // called once per instance, and these objects need to persist until the process terminates.

  robot = new ArRobot();
  ArArgumentBuilder *args = new ArArgumentBuilder(); //  never freed
  ArArgumentParser *argparser = new ArArgumentParser(args); // Warning never freed
  argparser->loadDefaultArguments(); // adds any arguments given in /etc/Aria.args.  Useful on robots with unusual serial port or baud rate (e.g. pioneer lx)

  // Now add any parameters given via ros params (see RosAriaNode constructor):

  // if serial port parameter contains a ':' character, then interpret it as hostname:tcpport
  // for wireless serial connection. Otherwise, interpret it as a serial port name.
  size_t colon_pos = serial_port.find(":");
  if (colon_pos != std::string::npos)
  {
    args->add("-remoteHost"); // pass robot's hostname/IP address to Aria
    args->add(serial_port.substr(0, colon_pos).c_str());
    args->add("-remoteRobotTcpPort"); // pass robot's TCP port to Aria
    args->add(serial_port.substr(colon_pos+1).c_str());
  }
  else
  {
    args->add("-robotPort %s", serial_port.c_str()); // pass robot's serial port to Aria
  }

  // if a baud rate was specified in baud parameter
  if(serial_baud != 0)
  {
    args->add("-robotBaud %d", serial_baud);
  }
  
  if( debug_aria )
  {
    // turn on all ARIA debugging
    args->add("-robotLogPacketsReceived"); // log received packets
    args->add("-robotLogPacketsSent"); // log sent packets
    args->add("-robotLogVelocitiesReceived"); // log received velocities
    args->add("-robotLogMovementSent");
    args->add("-robotLogMovementReceived");
    ArLog::init(ArLog::File, ArLog::Verbose, aria_log_filename.c_str(), true);
  }


  // Connect to the robot
  conn = new ArRobotConnector(argparser, robot); // warning never freed
  if (!conn->connectRobot()) {
    RCLCPP_ERROR("RosAria: ARIA could not connect to robot! (Check ~port parameter is correct, and permissions on port device, or any errors reported above)");
    return 1;
  }

  if(publish_aria_lasers)
    laserConnector = new ArLaserConnector(argparser, robot, conn);

  // causes ARIA to load various robot-specific hardware parameters from the robot parameter file in /usr/local/Aria/params
  if(!Aria::parseArgs())
  {
    RCLCPP_ERROR("RosAria: ARIA error parsing ARIA startup parameters!");
    return 1;
  }

  readParameters();

  // Start dynamic_reconfigure server
  dynamic_reconfigure_server = new dynamic_reconfigure::Server<rosaria::RosAriaConfig>;
  
  // Setup Parameter Minimums and maximums
  rosaria::RosAriaConfig dynConf_min;
  rosaria::RosAriaConfig dynConf_max;
  
  dynConf_max.trans_accel = robot->getAbsoluteMaxTransAccel() / 1000;
  dynConf_max.trans_decel = robot->getAbsoluteMaxTransDecel() / 1000;
  // TODO: Fix rqt dynamic_reconfigure gui to handle empty intervals
  // Until then, set unit length interval.
  dynConf_max.lat_accel = ((robot->getAbsoluteMaxLatAccel() > 0.0) ? robot->getAbsoluteMaxLatAccel() : 0.1) / 1000;
  dynConf_max.lat_decel = ((robot->getAbsoluteMaxLatDecel() > 0.0) ? robot->getAbsoluteMaxLatDecel() : 0.1) / 1000;
  dynConf_max.rot_accel = robot->getAbsoluteMaxRotAccel() * M_PI/180;
  dynConf_max.rot_decel = robot->getAbsoluteMaxRotDecel() * M_PI/180;

  dynConf_min.trans_accel = 0;
  dynConf_min.trans_decel = 0;
  dynConf_min.lat_accel = 0;
  dynConf_min.lat_decel = 0;
  dynConf_min.rot_accel = 0;
  dynConf_min.rot_decel = 0;
  
  dynConf_min.TicksMM     = 0;
  dynConf_max.TicksMM     = 200;
  dynConf_min.DriftFactor = -99999;
  dynConf_max.DriftFactor = 32767;
  dynConf_min.RevCount    = 0;
  dynConf_max.RevCount    = 65535;

  dynamic_reconfigure_server->setConfigMax(dynConf_max);
  dynamic_reconfigure_server->setConfigMin(dynConf_min);
  
  
  rosaria::RosAriaConfig dynConf_default;
  dynConf_default.trans_accel = robot->getTransAccel() / 1000;
  dynConf_default.trans_decel = robot->getTransDecel() / 1000;
  dynConf_default.lat_accel   = robot->getLatAccel() / 1000;
  dynConf_default.lat_decel   = robot->getLatDecel() / 1000;
  dynConf_default.rot_accel   = robot->getRotAccel() * M_PI/180;
  dynConf_default.rot_decel   = robot->getRotDecel() * M_PI/180;

  dynConf_default.TicksMM     = 0;
  dynConf_default.DriftFactor = -99999;
  dynConf_default.RevCount    = 0;
  
  dynamic_reconfigure_server->setConfigDefault(dynConf_default);
  
  dynamic_reconfigure_server->setCallback(boost::bind(&RosAriaNode::dynamic_reconfigureCB, this, _1, _2));


  // Enable the motors
  robot->enableMotors();

  // disable sonars on startup
  robot->disableSonar();

  // callback will  be called by ArRobot background processing thread for every SIP data packet received from robot
  robot->addSensorInterpTask("ROSPublishingTask", 100, &myPublishCB);

  // Initialize bumpers with robot number of bumpers
  bumpers.front_bumpers.resize(robot->getNumFrontBumpers());
  bumpers.rear_bumpers.resize(robot->getNumRearBumpers());

  // Run ArRobot background processing thread
  robot->runAsync(true);

  // connect to lasers and create publishers
  if(publish_aria_lasers)
  {
    RCLCPP_INFO("rosaria", "rosaria: Connecting to laser(s) configured in ARIA parameter file(s)...");
    if (!laserConnector->connectLasers())
    {
      ROS_FATAL("rosaria", "rosaria: Error connecting to laser(s)...");
      return 1;
    }

    robot->lock();
    const std::map<int, ArLaser*> *lasers = robot->getLaserMap();
    RCLCPP_INFO("rosaria", "rosaria: there are %lu connected lasers", lasers->size());
    for(std::map<int, ArLaser*>::const_iterator i = lasers->begin(); i != lasers->end(); ++i)
    {
      ArLaser *l = i->second;
      int ln = i->first;
      std::string tfname("laser");
      if(lasers->size() > 1 || ln > 1) // no number if only one laser which is also laser 1
        tfname += ln; 
      tfname += "_frame";
      RCLCPP_INFO("rosaria", "rosaria: Creating publisher for laser #%d named %s with tf frame name %s", ln, l->getName(), tfname.c_str());
      new LaserPublisher(l, n, true, tfname);
    }
    robot->unlock();
    RCLCPP_INFO("rosaria", "rosaria: Done creating laser publishers");
  }

  // register a watchdog for cmd_vel timeout
  double cmdvel_timeout_param = 0.6;
  n.param("cmd_vel_timeout", cmdvel_timeout_param, 0.6);
  cmdvel_timeout = ros::Duration(cmdvel_timeout_param);
  if (cmdvel_timeout_param > 0.0)
    cmdvel_watchdog_timer = n.createTimer(ros::Duration(0.1), &RosAriaNode::cmdvel_watchdog, this);

  RCLCPP_INFO("rosaria", "rosaria: Setup complete");
  return 0;
}

void RosAriaNode::publish()
{
  // Note, this is called via SensorInterpTask callback (myPublishCB, named "ROSPublishingTask"). ArRobot object 'robot' sholud not be locked or unlocked.
  pos = robot->getPose();

  rclcpp::Time now = this->get_clock()->now();
  geometry_msgs::msg::TransformStamped t;

  // Read message content and assign it to
  // corresponding tf variables
  t.header.stamp = now;
  t.header.frame_id = "world";
  t.child_frame_id = turtlename_.c_str();

  // Turtle only exists in 2D, thus we get x and y translation
  // coordinates from the message and set the z coordinate to 0
  t.transform.translation.x = msg->x;
  t.transform.translation.y = msg->y;
  t.transform.translation.z = 0.0;

  // For the same reason, turtle can only rotate around one axis
  // and this why we set rotation in x and y to 0 and obtain
  // rotation in z axis from the message
  tf2::Quaternion q;
  q.setRPY(0, 0, msg->theta);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  // Send the transformation
  tf_broadcaster_->sendTransform(t);


  tf::poseTFToMsg(tf::Transform(tf::createQuaternionFromYaw(pos.getTh()*M_PI/180), tf::Vector3(pos.getX()/1000,
    pos.getY()/1000, 0)), position.pose.pose); //Aria returns pose in mm.
  position.twist.twist.linear.x = robot->getVel()/1000.0; //Aria returns velocity in mm/s.
  position.twist.twist.linear.y = robot->getLatVel()/1000.0;
  position.twist.twist.angular.z = robot->getRotVel()*M_PI/180;
  
  position.header.frame_id = frame_id_odom;
  position.child_frame_id = frame_id_base;
  position.header.stamp = this->get_clock()->now();
  pose_pub.publish(position);

  RCLCPP_DEBUG("RosAria: publish: (time %f) pose x: %f, pose y: %f, pose angle: %f; linear vel x: %f, vel y: %f; angular vel z: %f", 
    position.header.stamp.toSec(), 
    (double)position.pose.pose.position.x,
    (double)position.pose.pose.position.y,
    (double)position.pose.pose.orientation.w,
    (double)position.twist.twist.linear.x,
    (double)position.twist.twist.linear.y,
    (double)position.twist.twist.angular.z
  );

  // publishing transform odom->base_link
  odom_trans.header.stamp = this->get_clock()->now();
  odom_trans.header.frame_id = frame_id_odom;
  odom_trans.child_frame_id = frame_id_base_link;
  
  odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(pos.getTh()*M_PI/180);

  odom_trans.transform.translation.x = pos.getX()/1000;
  odom_trans.transform.translation.y = pos.getY()/1000;
  odom_trans.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, msg->theta);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();
  
  odom_broadcaster->sendTransform(odom_trans);
  
  // getStallValue returns 2 bytes with stall bit and bumper bits, packed as (00 00 FrontBumpers RearBumpers)
  int stall = robot->getStallValue();
  unsigned char front_bumpers = (unsigned char)(stall >> 8);
  unsigned char rear_bumpers = (unsigned char)(stall);

  bumpers.header.frame_id = frame_id_bumper;
  bumpers.header.stamp = this->get_clock()->now();

  std::stringstream bumper_info(std::stringstream::out);
  // Bit 0 is for stall, next bits are for bumpers (leftmost is LSB)
  for (unsigned int i=0; i<robot->getNumFrontBumpers(); i++)
  {
    bumpers.front_bumpers[i] = (front_bumpers & (1 << (i+1))) == 0 ? 0 : 1;
    bumper_info << " " << (front_bumpers & (1 << (i+1)));
  }
  RCLCPP_DEBUG("RosAria: Front bumpers:%s", bumper_info.str().c_str());

  bumper_info.str("");
  // Rear bumpers have reverse order (rightmost is LSB)
  unsigned int numRearBumpers = robot->getNumRearBumpers();
  for (unsigned int i=0; i<numRearBumpers; i++)
  {
    bumpers.rear_bumpers[i] = (rear_bumpers & (1 << (numRearBumpers-i))) == 0 ? 0 : 1;
    bumper_info << " " << (rear_bumpers & (1 << (numRearBumpers-i)));
  }
  RCLCPP_DEBUG("RosAria: Rear bumpers:%s", bumper_info.str().c_str());
  
  bumpers_pub.publish(bumpers);

  //Publish battery information
  // TODO: Decide if BatteryVoltageNow (normalized to (0,12)V)  is a better option
  std_msgs::Float64 batteryVoltage;
  batteryVoltage.data = robot->getRealBatteryVoltageNow();
  voltage_pub.publish(batteryVoltage);

  if(robot->haveStateOfCharge())
  {
    std_msgs::Float32 soc;
    soc.data = robot->getStateOfCharge()/100.0;
    state_of_charge_pub.publish(soc);
  }

  // publish recharge state if changed
  char s = robot->getChargeState();
  if(s != recharge_state.data)
  {
    RCLCPP_INFO("RosAria: publishing new recharge state %d.", s);
    recharge_state.data = s;
    recharge_state_pub.publish(recharge_state);
  }

  // publish motors state if changed
  bool e = robot->areMotorsEnabled();
  if(e != motors_state.data || !published_motors_state)
  {
	RCLCPP_INFO("RosAria: publishing new motors state %d.", e);
	motors_state.data = e;
	motors_state_pub.publish(motors_state);
	published_motors_state = true;
  }

  // Publish sonar information, if enabled.
  if (publish_sonar || publish_sonar_pointcloud2)
  {
    sensor_msgs::PointCloud cloud;	//sonar readings.
    cloud.header.stamp = position.header.stamp;	//copy time.
    // sonar sensors relative to base_link
    cloud.header.frame_id = frame_id_sonar;
  

    std::stringstream sonar_debug_info; // Log debugging info
    sonar_debug_info << "Sonar readings: ";

    for (int i = 0; i < robot->getNumSonar(); i++) {
      ArSensorReading* reading = NULL;
      reading = robot->getSonarReading(i);
      if(!reading) {
        RCLCPP_WARN("RosAria: Did not receive a sonar reading.");
        continue;
      }
    
      // getRange() will return an integer between 0 and 5000 (5m)
      sonar_debug_info << reading->getRange() << " ";

      // local (x,y). Appears to be from the centre of the robot, since values may
      // exceed 5000. This is good, since it means we only need 1 transform.
      // x & y seem to be swapped though, i.e. if the robot is driving north
      // x is north/south and y is east/west.
      //
      //ArPose sensor = reading->getSensorPosition();  //position of sensor.
      // sonar_debug_info << "(" << reading->getLocalX() 
      //                  << ", " << reading->getLocalY()
      //                  << ") from (" << sensor.getX() << ", " 
      //                  << sensor.getY() << ") ;; " ;
    
      //add sonar readings (robot-local coordinate frame) to cloud
      geometry_msgs::Point32 p;
      p.x = reading->getLocalX() / 1000.0;
      p.y = reading->getLocalY() / 1000.0;
      p.z = 0.0;
      cloud.points.push_back(p);
    }
    RCLCPP_DEBUG(sonar_debug_info.str());
    
    // publish topic(s)

    if(publish_sonar_pointcloud2)
    {
      sensor_msgs::msg::PointCloud2 cloud2;
      if(!sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2))
      {
        RCLCPP_WARN("Error converting sonar point cloud message to point_cloud2 type before publishing! Not publishing this time.");
      }
      else
      {
        sonar_pointcloud2_pub.publish(cloud2);
      }
    }

    if(publish_sonar)
    {
      sonar_pub.publish(cloud);
    }
  } // end if sonar_enabled
}

bool RosAriaNode::enable_motors_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  RCLCPP_INFO("RosAria: Enable motors request.");
  robot->lock();
  if(robot->isEStopPressed())
      RCLCPP_WARN("RosAria: Warning: Enable motors requested, but robot also has E-Stop button pressed. Motors will not enable.");
  robot->enableMotors();
  robot->unlock();
// todo could wait and see if motors do become enabled, and send a response with an error flag if not
  return true;
}

bool RosAriaNode::disable_motors_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  RCLCPP_INFO("RosAria: Disable motors request.");
  robot->lock();
  robot->disableMotors();
  robot->unlock();
// todo could wait and see if motors do become disabled, and send a response with an error flag if not
  return true;
}

void
RosAriaNode::cmdvel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  veltime = this->get_clock()->now();
  RCLCPP_INFO( "new speed: [%0.2f,%0.2f](%0.3f)", msg->linear.x*1e3, msg->angular.z, veltime.toSec() );

  robot->lock();
  robot->setVel(msg->linear.x*1e3);
  if(robot->hasLatVel())
    robot->setLatVel(msg->linear.y*1e3);
  robot->setRotVel(msg->angular.z*180/M_PI);
  robot->unlock();
  RCLCPP_DEBUG("RosAria: sent vels to to aria (time %f): x vel %f mm/s, y vel %f mm/s, ang vel %f deg/s", veltime.toSec(),
    (double) msg->linear.x * 1e3, (double) msg->linear.y * 1e3, (double) msg->angular.z * 180/M_PI);
}

void RosAriaNode::cmdvel_watchdog(const ros::TimerEvent& event)
{
  // stop robot if no cmd_vel message was received for 0.6 seconds
  if (ros::Time::now() - veltime > ros::Duration(0.6))
  {
    robot->lock();
    robot->setVel(0.0);
    if(robot->hasLatVel())
      robot->setLatVel(0.0);
    robot->setRotVel(0.0);
    robot->unlock();
  }
}

int main( int argc, char** argv )
{
  ros::init(argc,argv, "RosAria");
  ros::NodeHandle n(std::string("~"));
  Aria::init();

  RosAriaNode *node = new RosAriaNode(n);

  if( node->Setup() != 0 )
  {
    ROS_FATAL( "RosAria: ROS node setup failed... \n" );
    return -1;
  }

  node->spin();

  delete node;

  RCLCPP_INFO( "RosAria: Quitting... \n" );
  return 0;
  
}
