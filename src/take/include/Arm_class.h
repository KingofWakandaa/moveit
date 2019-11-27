#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
class Arm_class
{
  public:
    pthread_mutex_t count_mutex;
    geometry_msgs::Point position;
    geometry_msgs::Quaternion orientation;
    TurtleClass();
    ~TurtleClass();
    void getPose(const geometry_msgs::PoseStampedConstPtr &pose);//CREATE A CALLBACK FUNCTION FOR THE TOPIC  position
};
