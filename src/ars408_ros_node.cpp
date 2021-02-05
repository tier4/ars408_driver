#include "ars408_ros/ars408_ros_node.h"

PeContinentalArs408Node::PeContinentalArs408Node() :
    global_node_handle_(), private_node_handle_("~"), ars408_parser_()
{

}

void PeContinentalArs408Node::CanFrameCallback(const can_msgs::Frame::ConstPtr& can_msg)
{
  if (!can_msg->data.empty())
  {
//    std::vector<uint8_t> data;
//    for (uint8_t i=0; i < can_msg->dlc; i++)
//    {
//      data.emplace_back(can_msg->data[i]);
//    }

    ars408_parser_.Parse(can_msg->id, can_msg->data, can_msg->dlc);
  }
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
