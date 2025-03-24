#ifndef FORCE_TEST
#define FORCE_TEST


#include <gz/msgs.hh>
#include <gz/transport.hh>

std::string topic_pub = "/pos";   //publish to this topic
gz::transport::Node node;
auto pub = node.Advertise<gz::msgs::Twist>(topic_pub);

class Vector{
  public: double x, y, z;
};

class face{
public: Vector v1, v2, v3 ,v4;
};


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

int facec;
Vector * comp;
Vector * normalp;

Vector Forward;
Vector Up;

Vector OrientToNormal(double, double, double, double);
double * dist(Vector);
Vector estcomf(double*, Vector);


void cb(const gz::msgs::IMU &_msg)
{

  gz::msgs::Twist data;
  double orix = _msg.orientation().x();
  double oriy = _msg.orientation().y();
  double oriz = _msg.orientation().z();
  double oriw = _msg.orientation().w();

  Vector ori = OrientToNormal(orix, oriy, oriz, oriw);
  Up = ori;
  Forward = OrienttoForward(orix, oriy, oriz, oriw);

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

  Vector estpos;
  estpos.x = posx;
  estpos.y = posy;
  estpos.z = posz;

  double * disp;
  disp = dist(estpos);
  Vector estcom;
  estcom = estcomf(disp, ori);


  printf("Time =%f s\n", timestamp);
  printf("Orientation:\n x=%f, y=%f, z=%f\n", ori.x, ori.y, ori.z);
  //printf("Lin Acc:\nx=%f, y=%f, z=%f\n", linax, linay, linaz);
 // printf("Lin Vel:\nx=%f, y=%f, z=%f\n", velx, vely, velz);
  printf("Lin Pos:\nx=%f, y=%f, z=%f\n", estpos.x, estpos.y, estpos.z);
 // printf("Ang Vel:\nx=%f, y=%f, z=%f\n", angvx, angvy, angvz);
 // printf("Ang pos:\nx=%f, y=%f, z=%f\n", angpx, angpy, angpz);
  printf("On face: x=%f, y=%f. z=%f\n", estcom.x, estcom.y,estcom.z);
  printf("------------------------------------\n");


  pub.Publish(data);
}

Vector* GetVertices(){
  FILE * file = fopen("boat.obj", "r");
  if(file == NULL){
    printf("CANT OPEN FILE\n");
    exit(1);
  }
  int count = 0;
  while(1){
    char lineHeader[128];
    int res = fscanf(file, "%s", lineHeader);
    if (res == EOF) break;
    if (strcmp (lineHeader, "v") == 0){
      count++;
    }
  }
  Vector * verticesp = (Vector*)malloc(count * sizeof(Vector));
  count = 0;
  rewind(file);
  while(1){
    char lineHeader[128];
    int res = fscanf(file, "%s", lineHeader);
    if (res == EOF) break;
    if (strcmp (lineHeader, "v") == 0){
      Vector vertex;
      fscanf(file, "%lf %lf %lf\n", &vertex.x, &vertex.y, &vertex.z);
      verticesp[count]=vertex;
      count++;
    }
  }
  fclose(file);
  return verticesp;
}

face * GetFaces(Vector* verticesp){
  FILE * file = fopen("boat.obj", "r");
  if(file == NULL){
    printf("CANT OPEN FILE\n");
    exit(1);
  }
  int count = 0;
  while(1){
    char lineHeader[128];
    int res = fscanf(file, "%s", lineHeader);
    if (res == EOF) break;
    if (strcmp (lineHeader, "f") == 0){
      count++;
    }
  }
  face * facep = (face*)malloc(count * sizeof(face));
  count = 0;
  rewind(file);
  while(1){
    char lineHeader[128];
    int res = fscanf(file, "%s", lineHeader);
    if (res == EOF) break;
    if (strcmp (lineHeader, "f") == 0){
      face facer;
      int vertex1, vertex2, vertex3, vertex4;
      int dump;
      fscanf(file, "%d/%d/%d %d/%d/%d %d/%d/%d %d/%d/%d\n", &vertex1, &dump, &dump, &vertex2, &dump, &dump, &vertex3, &dump, &dump, &vertex4, &dump, &dump);
      facer.v1=verticesp[vertex1-1];
      facer.v2=verticesp[vertex2-1];
      facer.v3=verticesp[vertex3-1];
      facer.v4=verticesp[vertex4-1];
      facep[count]=facer;
      count++;
    }
  }
  facec = count;
  return facep;
}

Vector * GetCenterOfMass(face* facep){
  Vector * comp = (Vector*)malloc(facec * sizeof(Vector));
  face facer;
  Vector com;
  for(int i = 0; i<facec; i++){
    facer = facep[i];
    com.x = (facer.v1.x + facer.v2.x + facer.v3.x + facer.v4.x)/4;
    com.y = (facer.v1.y + facer.v2.y + facer.v3.y + facer.v4.y)/4;
    com.z = (facer.v1.z + facer.v2.z + facer.v3.z + facer.v4.z)/4;
    comp[i] = com;
  }
  return comp;
}

double Distance(Vector v1, Vector v2){
  double dist = sqrt(pow(v1.x-v2.x,2)+pow(v1.y-v2.y,2)+pow(v1.z-v2.z,2));
  return dist;
}

Vector * GetNormals(face* facep){
  Vector * normalp = (Vector*)malloc(facec * sizeof(Vector));
  face facer;
  Vector normal, v1,v2,v3,v4;
  double length;
  for(int i = 0; i<facec; i++){
    facer = facep[i];
    v1=facer.v1;
    v2=facer.v2;
    v3=facer.v3;
    v4=facer.v4;
    normal.x = (v1.y-v2.y)*(v1.z+v2.z)+(v2.y-v3.y)*(v2.z+v3.z)+(v3.y-v4.y)*(v3.z+v4.z)+(v4.y-v1.y)*(v4.z+v1.z);
    normal.y = (v1.z-v2.z)*(v1.x+v2.x)+(v2.z-v3.z)*(v2.x+v3.x)+(v3.z-v4.z)*(v3.x+v4.x)+(v4.z-v1.z)*(v4.x+v1.x);
    normal.z = (v1.x-v2.x)*(v1.y+v2.y)+(v2.x-v3.x)*(v2.y+v3.y)+(v3.x-v4.x)*(v3.y+v4.y)+(v4.x-v1.x)*(v4.y+v1.y);
    length = sqrt(pow(normal.x,2)+pow(normal.y,2)+pow(normal.z,2));
    normal.x=normal.x/length;
    normal.y=normal.y/length;
    normal.z=normal.z/length;
    normalp[i]=normal;
  }
  return normalp;
}
Vector OrientToNormal(double x, double y, double z, double w){
  Vector normal;
  normal.x = 2*(x*z+w*y);
  normal.y = 2*(y*z-w*x);
  normal.z = 1-2*(x*x+y*y);
  return normal;
}

Vector OrienttoForward(double x, double y, double z, double w){
  Vector forward;
  forward.x = 2*(x*y - z*w);
  forward.y = 1-2*(x*x + z*z);
  forward.z = 2*(z*y + x*w);
  return forward;
}

double * dist(Vector posest){
  double * distp = (double*)malloc(facec*sizeof(double));
  for(int i=0; i<facec; i++){
    distp[i] = Distance(comp[i], posest);
  }
  return distp;
}

Vector estcomf(double * distp, Vector norm){
  double heuristic[facec];
  int maxind = -1;
  double maxheur = 0;
  for(int i = 0; i<facec; i++){
    heuristic[i] = 1/distp[i] + 0.1/(Distance(norm, normalp[i])+5);
    if(heuristic[i]>maxheur){
      maxind = i;
      maxheur = heuristic[i];
    }
  }
  if(maxind==-1){
    exit(1);
  }
  return comp[maxind];
}



int init_localization(int argc, char **argv)
{
  std::string topic_sub = "/imu";   // subscribe to this topic
  // Subscribe to a topic by registering a callback.
  if (!node.Subscribe(topic_sub, cb))
  {
    std::cerr << "Error subscribing to topic [" << topic_sub << "]" << std::endl;
    return -1;
  }
  // Zzzzzz.

  Vector * verticesp;
  verticesp = GetVertices();
  face * facep;

  facep = GetFaces(verticesp);

  comp = GetCenterOfMass(facep);
  normalp = GetNormals(facep );

  gz::transport::waitForShutdown();
  free(verticesp);
  return 0;
}

#endif