#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


class Shared_Control
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber teleop_sub_;
    ros::Subscriber vfh_sub_;
    ros::Publisher shared_cmd_;
    ros::Timer timer;


    geometry_msgs::Twist tele;
    geometry_msgs::Twist vfh;
    geometry_msgs::Twist cmd_vel;



public:
    Shared_Control();
    void teleCallback(const geometry_msgs::Twist::ConstPtr& tele);
    void vfhCallback(const geometry_msgs::Twist::ConstPtr& vfh);
    void operationCallback(const ros::TimerEvent&);
};

Shared_Control::Shared_Control()
{
  teleop_sub_ = nh_.subscribe("/teleop/cmd_vel", 5, &Shared_Control::teleCallback,this);
  vfh_sub_ = nh_.subscribe("/vfh/cmd_vel", 5, &Shared_Control::vfhCallback, this);
  shared_cmd_ = nh_.advertise<geometry_msgs::Twist>("/shared_control/cmd_vel", 5);
  timer = nh_.createTimer(ros::Duration(0.1), &Shared_Control::operationCallback, this);
}

void Shared_Control::teleCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  tele.linear.x = msg->linear.x;
  tele.linear.y = msg->linear.y;
  tele.linear.z = msg->linear.z;
  tele.angular.x = msg->angular.x;
  tele.angular.y = msg->angular.y;
  tele.angular.z = msg->angular.z;
  }
void Shared_Control::vfhCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  vfh.linear.x = msg->linear.x;
  vfh.linear.y = msg->linear.y;
  vfh.linear.z = msg->linear.z;
  vfh.angular.x = msg->angular.x;
  vfh.angular.y = msg->angular.y;
  vfh.angular.z = msg->angular.z;
}

void Shared_Control::operationCallback(const ros::TimerEvent&)
{
  cmd_vel.linear.x = tele.linear.x /*+ vfh.linear.x*/;
  cmd_vel.linear.y = tele.linear.y + vfh.linear.y;
  cmd_vel.linear.z = tele.linear.z + vfh.linear.z;

  cmd_vel.angular.x = tele.angular.x + vfh.angular.x;
  cmd_vel.angular.y = tele.angular.y + vfh.angular.y;
  cmd_vel.angular.z = tele.angular.z - vfh.angular.z; //

  shared_cmd_.publish(cmd_vel);
}

//void Shared_Control::cmd_velCallback(const geometry_msgs::Twist& msg){

//}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "shared_control_node");
  Shared_Control shared_control_node;
  ros::spin();

  return 0;
}
