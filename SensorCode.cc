#include <gz/msgs/twist.pb.h>
#include <gz/msgs/laserscan.pb.h>
#include <gz/transport/Node.hh>

std::string topic_pub = "/pos";   //publish to this topic
gz::transport::Node node;
auto pub = node.Advertise<gz::msgs::Twist>(topic_pub);

void cb(const gz::msgs::LaserScan &_msg)
{
  gz::msgs::Twist data;

  bool allMore = true;
  for (int i = 0; i < _msg.ranges_size(); i++)
  {
    if (_msg.ranges(i) < 1.0)
    {
      allMore = false;
      break;
    }
  }
  if (allMore) //if all bigger than one
  {
    data.mutable_linear()->set_x(0.5);
    data.mutable_angular()->set_z(0.0);
  }
  else
  {
    data.mutable_linear()->set_x(0.0);
    data.mutable_angular()->set_z(0.5);
  }
  pub.Publish(data);
}

int main(int argc, char **argv)
{
  std::string topic_sub = "/imu";   // subscribe to this topic
  // Subscribe to a topic by registering a callback.
  if (!node.Subscribe(topic_sub, cb))
  {
    std::cerr << "Error subscribing to topic [" << topic_sub << "]" << std::endl;
    return -1;
  }
  printf("hello world\n");
  // Zzzzzz.
  gz::transport::waitForShutdown();

  return 0;
}
