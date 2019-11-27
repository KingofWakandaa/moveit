#include "Arm_class.h"
Arm_class::Arm_class()
{
  position.x = 0;
  position.y = 0;
  position.z = 0;
  orientation.x = 0;
  orientation.y = 0;
  orientation.z = 0;
  orientation.w = 0;
}
Arm_class::~Arm_class()
{

}
void Arm_class::getPose(const geometry_msgs::PoseStampedConstPtr &pose)
{
  pthread_mutex_lock( &this->count_mutex );
  //COPY THE MSG TO A LOCAL VARIABLE
  position = pose->pose.position;
  orientation = pose->pose.orientation;
  pthread_mutex_unlock( &this->count_mutex );
}
