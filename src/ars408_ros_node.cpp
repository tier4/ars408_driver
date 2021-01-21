#include "ars408_ros/ars408_ros_node.h"

PeContinentalArs408Node::PeContinentalArs408Node() :
    global_node_handle_(), private_node_handle_("~")
{

}

void PeContinentalArs408Node::CanFrameCallback(const can_msgs::Frame::ConstPtr& can_msg)
{

  std::cout << "[" << ros::this_node::getName() << "] CAN ID: " << std::hex << can_msg->id << " DATA: " ;

  for(size_t i=0 ; i< can_msg->dlc; i++)
  {
    std::cout << unsigned(can_msg->data[i]) << " ";
  }

  std::cout << std::endl;
}

void PeContinentalArs408Node::Run()
{
  std::string can_input_topic = "can_raw";

  subscriber_can_raw_ = global_node_handle_.subscribe(can_input_topic, 1, &PeContinentalArs408Node::CanFrameCallback, this);
  ROS_INFO_STREAM("Subscribed to " << can_input_topic);
  ros::spin();
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ars408_ros_node");

  PeContinentalArs408Node app;

  app.Run();

  return 0;
}
