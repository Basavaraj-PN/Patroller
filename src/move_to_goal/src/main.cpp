#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_srvs/Trigger.h"
#include "move_base_msgs/MoveBaseAction.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream> 
#include "yaml-cpp/yaml.h"


#include "vector"
#include <boost/lexical_cast.hpp>
std::vector<geometry_msgs::Point> landmarks;
visualization_msgs::MarkerArray marker_array;
ros::Publisher marker_pub;
ros::Publisher goal_publisher;
ros::Publisher reached_publisher;
static int current_index = 0;
static bool start_patrol = false;

sensor_msgs::Image image;

const std::string mPath1 = "src/move_to_goal/Images/image_set01/";
const std::string mPath2 = "src/move_to_goal/Images/image_set02/";


void imageCallback(const sensor_msgs::Image& msg){
  image = msg;
}

void saveImage(const std::string &Path){

    cv_bridge::CvImagePtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    static int image_count = 0;
    std::stringstream sstream;                               // added this
    sstream << Path + "image_" << image_count << ".png" ;                  // added this
    ROS_ASSERT( cv::imwrite( sstream.str(),  cv_ptr->image ) );      // added this
    image_count++;                                      // added this
    ros::param::set("/done", false);
}

void loadGoalFromYaml(const YAML::Node &config)
{

  for (YAML::const_iterator it = config["goals"].begin(); it != config["goals"].end(); ++it)
  {
    static int i = 0;
    geometry_msgs::Point p;
    p.x = it->second[0].as<float>();
    p.y = it->second[1].as<float>();
    ROS_INFO_STREAM("Adding landmark #" << i + 1 << ": ( "
                                        << p.x << ", " << p.y << ");");
    landmarks.push_back(p);
  
  visualization_msgs::Marker cylinder;

  {
    cylinder.header.frame_id = "/map";
    cylinder.header.stamp = ros::Time::now();
    cylinder.ns = "cylinder";
    cylinder.id = 2 * i;
    cylinder.type = visualization_msgs::Marker::CYLINDER;
    cylinder.action = visualization_msgs::Marker::ADD;
    cylinder.pose.orientation.w = 1.0;
    cylinder.scale.x = 0.5;
    cylinder.scale.y = 0.5;
    cylinder.scale.z = 0.01;

    cylinder.color.g = 1.0f;
    cylinder.color.a = 0.4f;
    cylinder.lifetime = ros::Duration();

    cylinder.pose.position.x = landmarks[i].x;
    cylinder.pose.position.y = landmarks[i].y;
  }
  marker_array.markers.push_back(cylinder);

  visualization_msgs::Marker text;

  {
    text.header.frame_id = "/map";
    text.header.stamp = ros::Time::now();
    text.ns = "text";
    text.id = 2 * i + 1;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;
    text.pose.orientation.w = 1.0;
    text.scale.x = 1;
    text.scale.y = 1;
    text.scale.z = 1;

    text.color.r = 1.0f;
    text.color.a = 1.0f;
    text.lifetime = ros::Duration();

    text.pose.position.x = landmarks[i].x + 0.2;
     text.pose.position.y = landmarks[i].y + 0.2;

    text.text = boost::lexical_cast<std::string>(i + 1);
  }
  marker_array.markers.push_back(text);
  i++;

  }

}

void publishNextGoal(bool first_input) {

  if ( !first_input )
  current_index++;
  if ( current_index >= landmarks.size() ){
      start_patrol = false;
      current_index = 0;
      ros::param::set("/stop", true);
  }
  else{
  geometry_msgs::PoseStamped goal;
  goal.header.stamp = ros::Time::now();
  goal.header.frame_id = "map";
  goal.pose.position.x = landmarks[current_index].x;
  goal.pose.position.y = landmarks[current_index].y;
  goal.pose.orientation.w = 1.0f;

  goal_publisher.publish(goal);
  
  }
}

void positionCallback(
    const move_base_msgs::MoveBaseActionFeedback::ConstPtr& position) {

  if ( !start_patrol ) return;

  double distance = (position->feedback.base_position.pose.position.x -landmarks[current_index].x) *
      (position->feedback.base_position.pose.position.x - landmarks[current_index].x) +
      (position->feedback.base_position.pose.position.y -landmarks[current_index].y) *
      (position->feedback.base_position.pose.position.y - landmarks[current_index].y);

  if ( distance < 0.09 ) { // 0.3m
    ros::param::set("/done", true);
    bool smach_use;
    ros::param::get("/smach_use", smach_use);
    if(!smach_use){
      saveImage(mPath1);
    }
    publishNextGoal(false);
  }
  marker_pub.publish(marker_array);
  
}

bool receiveStartCommand(std_srvs::Trigger::Request &req,
    std_srvs::Trigger::Response &res) {

  if ( landmarks.size() < 2 ) {
    ROS_ERROR("The landmark number is not correct!");
    res.success = false;
    res.message = "There are at lest have 2 landmarks.";
    return false;
  } else {
    start_patrol = true;
    res.success = true;
    res.message = "Start patrol.";
    publishNextGoal(true);
    return true;
  }
}

bool receiveStopCommand(std_srvs::Trigger::Request &req,
    std_srvs::Trigger::Response &res) {
  start_patrol = false;
  res.success = true;
  res.message = "Stop patrol.";
  return true;
}

bool receiveResetCommand(std_srvs::Trigger::Request &req,
    std_srvs::Trigger::Response &res) {
  start_patrol = false;
  res.success = true;
  res.message = "Reset patrol.";
  return true;
}

bool receiveSaveImageCommand(std_srvs::Trigger::Request &req,
    std_srvs::Trigger::Response &res) {
  saveImage(mPath2);
  res.success = true;
  res.message = "Image Saved.";
  return true;
}

int main(int argc,char** argv)
{
 
  ros::init(argc, argv, "patroller");
  ros::NodeHandle n;
  ros::param::set("/done", false);
  ros::param::set("/stop", false);
  YAML::Node config = YAML::LoadFile("src/move_to_goal/config/config.yaml");
  loadGoalFromYaml(config);
  // ros::Rate r(1);
  marker_pub = n.advertise<visualization_msgs::MarkerArray>("/landmarks", 10);
  goal_publisher = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 2);

  ros::Subscriber sub = n.subscribe("/camera/rgb/image_raw", 1, imageCallback);
  ros::Subscriber current_position_sub =
      n.subscribe("/move_base/feedback", 10, positionCallback);

  ros::ServiceServer cmd_start_server =
      n.advertiseService("StartPatrol", &receiveStartCommand);
  ros::ServiceServer cmd_stop_server =
      n.advertiseService("StopPatrol", &receiveStopCommand);
  ros::ServiceServer cmd_reset_server =
      n.advertiseService("ResetPatrol", &receiveResetCommand);
  ros::ServiceServer cmd_image_server =
      n.advertiseService("SaveImage", &receiveSaveImageCommand);


  ros::spin();
  return 0;
}
