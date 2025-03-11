#include <gz/msgs.hh>
#include <gz/transport.hh>

std::string topic_pub = "/pos";   //publish to this topic
gz::transport::Node node;
auto pub = node.Advertise<gz::msgs::Twist>(topic_pub);

double velx = 0;
double vely = 0;
double velz = 0;

double posx = 0;
double posy = 0;
double posz = 0;

double angpx = 0;
double angpy = 0;
double angpz = 0;

double dt = 0.01;
double timestamp = 0;

void cb(const gz::msgs::IMU &_msg)
{

  gz::msgs::Twist data;
  double orix = _msg.orientation().x();
  double oriy = _msg.orientation().y();
  double oriz = _msg.orientation().z();

  double linax = _msg.linear_acceleration().x();
  double linay = _msg.linear_acceleration().y();
  double linaz = _msg.linear_acceleration().z();

  double angvx = _msg.angular_velocity().x();
  double angvy = _msg.angular_velocity().y();
  double angvz = _msg.angular_velocity().z();


  velx = velx + linax * dt;
  vely = vely + linay * dt;
  velz = velz + linaz * dt;

  posx = posx + velx * dt;
  posy = posy + vely * dt;
  posz = posz + velz * dt;

  angpx = angpx + angvx * dt;
  angpy = angpy + angvy * dt;
  angpz = angpz + angvz * dt;

  timestamp = timestamp + dt;

  printf("Time =%f s\n", timestamp);
  printf("Orientation:\nx=%f, y=%f, z=%f\n", orix, oriy, oriz);
  printf("Lin Acc:\nx=%f, y=%f, z=%f\n", linax, linay, linaz);
  printf("Lin Vel:\nx=%f, y=%f, z=%f\n", velx, vely, velz);
  printf("Lin Pos:\nx=%f, y=%f, z=%f\n", posx, posy, posz);
  printf("Ang Vel:\nx=%f, y=%f, z=%f\n", angvx, angvy, angvz);
  printf("Ang pos:\nx=%f, y=%f, z=%f\n", angpx, angpy, angpz);
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
