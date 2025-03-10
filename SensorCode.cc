#include <gz/msgs.hh>
#include <gz/transport.hh>

std::string topic_pub = "/pos";   //publish to this topic
gz::transport::Node node;
auto pub = node.Advertise<gz::msgs::Twist>(topic_pub);

void cb(const gz::msgs::IMU &_msg)
{

  gz::msgs::Twist data;
  double posx = _msg.orientation().x();
  double posy = _msg.orientation().y();
  double posz = _msg.orientation().z();


  printf("x=%f, y=%f, z=%f\n", posx, posy, posz);

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
  // Zzzzzz.
  gz::transport::waitForShutdown();

  return 0;
}
