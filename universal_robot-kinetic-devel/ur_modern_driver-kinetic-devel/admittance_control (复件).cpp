#include "ros/ros.h"
#include "<geometry_msgs/WrenchStamped.h"

typedef unsigned int uint32;
typedef int int32;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned char byte;
/*
struct RDTRecord
{
  uint32_t rdt_sequence_;
  uint32_t ft_sequence_;
  uint32_t status_;
  int32_t fx_;
  int32_t fy_;
  int32_t fz_;
  int32_t tx_;
  int32_t ty_;
  int32_t tz_;

  enum {RDT_RECORD_SIZE = 36};
  void unpack(const uint8_t *buffer);
  static uint32_t unpack32(const uint8_t *buffer);
};*/
class Localizer{
public:
    Localizer(ros:NodeHandle & nh){
        ar_sub = nh.subscribe<geometry_msgs::WrenchStamped>
        ("tf", 1, &Localizer::number_callback, this);
    
    }
    void number_callback(const geometry_msgs::WrenchStamped::ConstPtr & msg){
        last_msg_ = msg;
        uint32_t rdt_sequence_;
        uint32_t ft_sequence_;
        uint32_t status_;
        int32_t fx_;
        int32_t fy_;
        int32_t fz_;
        int32_t tx_;
        int32_t ty_;
        int32_t tz_;
        ROS_INFO_STREAM(last_msg_->fx_);
    }
    ros::Subscriber ar_sub_;
    geometry_msgs last_msg_;
};
int main(int argc, char **argv){
    int i = 0;
    ros::init(argc, argv, "admittance_controller");
    ros::NodeHandle node_obj;
    Localizer localizer(node_obj);
    ROS_INFO("admittance controller starting");
    ros::spin();
}
