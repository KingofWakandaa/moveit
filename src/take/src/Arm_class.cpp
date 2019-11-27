#include "take/Arm_class.h"

void Arm_class::getPose(const geometry_msgs::PoseStampedConstPtr &pose)
{
  pthread_mutex_lock( &this->count_mutex );
  //COPY THE MSG TO A LOCAL VARIABLE
  position = pose->pose.position;
  orientation = pose->pose.orientation;
  pthread_mutex_unlock( &this->count_mutex );
}
