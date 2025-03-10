#include <gz/msgs/twist.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/transport/Node.hh>

std::string topic_pub = "/pos";   //publish to this topic
gz::transport::Node node;
auto pub = node.Advertise<gz::msgs::Twist>(topic_pub);

double velx = 0;
double vely = 0;
double velz = 0;

double posx = 0;
double posy = 0;
double posz = 0;

double dt = 0.01;

void cb(const gz::msgs::IMU &_msg)
{

  gz::msgs::Twist data;
  double orix = _msg.orientation().x();
  double oriy = _msg.orientation().y();
  double oriz = _msg.orientation().z();

  double linax = _msg.linear_acceleration().x();
  double linay = _msg.linear_acceleration().y();
  double linaz = _msg.linear_acceleration().z();

  velx=velx+linax * dt;

  posx = posx + velx * dt;



  printf("Orientation:\nx=%f, y=%f, z=%f\n", orix, oriy, oriz);
  printf("Lin Acc:\nx=%f, y=%f, z=%f\n", linax, linay, linaz);
  printf("Lin Vel:\nx=%f, y=%f, z=%f\n", velx, vely, velz);
  printf("Lin pos:\nx=%f, y=%f, z=%f\n\n\n", posx, posy, posz);
  printf("------------------------------------\n");



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
