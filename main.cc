#include <gz/msgs.hh>
#include <gz/transport.hh>
#include <gz/physics.hh>
//#include <gz/client.hh>
//#include <gz/sim/Link.hh>
#include <iostream>
#include <chrono>
#include <thread>

class Vector {
public: double x, y, z;
};

class face {
public: Vector v1, v2, v3, v4;
};

gz::transport::Node node;

double dt = 0.1;
double timestamp = 0;

int facec;
Vector* comp;
Vector* normalp;

Vector Forward;
Vector Up;


	double posx,posy, posz;
	double posxl,posyl, poszl;

Vector OrientToNormal(double, double, double, double);
Vector OrientToForward(double, double, double, double);
double* dist(Vector);
Vector estcomf(double*, Vector);


void cb(const gz::msgs::Odometry& _msg)
{
//	gz::physics::WorldPtr world = gz::physics::World();
//	gz::physics::LinkPtr link = world.GetLink("robotlink");
//	double x = link->WorldPose().Pos().X();
	//msg.mutable_entity()->set_id(9);

	double orix = _msg.pose().orientation().x();
	double oriy = _msg.pose().orientation().y();
	double oriz = _msg.pose().orientation().z();
	double oriw = _msg.pose().orientation().w();

	Vector ori = OrientToNormal(orix, oriy, oriz, oriw);
	Up = ori;
	Forward = OrientToForward(orix, oriy, oriz, oriw);

	posxl = posx;
	posyl = posy;
	poszl = posz;

	double velx, vely, velz;



	posx = _msg.pose().position().x();
	posy = _msg.pose().position().y();
	posz = _msg.pose().position().z();

	velx = (posx-posxl)/0.1;
	vely = (posy-posyl)/0.1;
	velz = (posz-poszl)/0.1;


	timestamp = timestamp + dt;

	Vector estpos;
	estpos.x = posx;
	estpos.y = posy;
	estpos.z = posz;

	double* disp;
	disp = dist(estpos);
	Vector estcom;
	estcom = estcomf(disp, ori);


	printf("Time =%f s\n", timestamp);
	printf("Orientation:\n x=%f, y=%f, z=%f\n", ori.x, ori.y, ori.z);
//	printf("Lin Acc:\nx=%f, y=%f, z=%f\n", linax, linay, linaz);
    printf("Lin Vel:\nx=%f, y=%f, z=%f\n", velx, vely, velz);
	printf("Lin Pos:\nx=%f, y=%f, z=%f\n", posx, posy, posz);
	// printf("Ang Vel:\nx=%f, y=%f, z=%f\n", angvx, angvy, angvz);
	// printf("Ang pos:\nx=%f, y=%f, z=%f\n", angpx, angpy, angpz);
//	printf("On face: x=%f, y=%f. z=%f\n", estcom.x, estcom.y, estcom.z);
	printf("------------------------------------\n");
	if(timestamp>199){
		printf("Found a crack at %f, %f, %f", estpos.x, estpos.y, estpos.z);
		exit(1);
	}
}

Vector* GetVertices() {
	FILE* file = fopen("boat.obj", "r");
    std::cout << "opened" << std::endl;
	if (file == NULL) {
        std::cout << "CANT OPEN FILE" << std::endl;
		exit(1);
	}
	int count = 0;
	while (1) {
		char lineHeader[128];
		int res = fscanf(file, "%s", lineHeader);
		if (res == EOF) break;
		if (strcmp(lineHeader, "v") == 0) {
			count++;
		}
	}
	Vector* verticesp = (Vector*)malloc(count * sizeof(Vector));
	count = 0;
	rewind(file);
	while (1) {
		char lineHeader[128];
		int res = fscanf(file, "%s", lineHeader);
		if (res == EOF) break;
		if (strcmp(lineHeader, "v") == 0) {
			Vector vertex;
			fscanf(file, "%lf %lf %lf\n", &vertex.x, &vertex.y, &vertex.z);
			verticesp[count] = vertex;
			count++;
		}
	}
	fclose(file);
    std::cout << "closed" << std::endl;
	return verticesp;
}

face* GetFaces(Vector* verticesp) {
	FILE* file = fopen("boat.obj", "r");
	if (file == NULL) {
		printf("CANT OPEN FILE\n");
		exit(1);
	}
	int count = 0;
	while (1) {
		char lineHeader[128];
		int res = fscanf(file, "%s", lineHeader);
		if (res == EOF) break;
		if (strcmp(lineHeader, "f") == 0) {
			count++;
		}
	}
	face* facep = (face*)malloc(count * sizeof(face));
	count = 0;
	rewind(file);
	while (1) {
		char lineHeader[128];
		int res = fscanf(file, "%s", lineHeader);
		if (res == EOF) break;
		if (strcmp(lineHeader, "f") == 0) {
			face facer;
			int vertex1, vertex2, vertex3, vertex4;
			int dump;
			fscanf(file, "%d/%d/%d %d/%d/%d %d/%d/%d %d/%d/%d\n", &vertex1, &dump, &dump, &vertex2, &dump, &dump, &vertex3, &dump, &dump, &vertex4, &dump, &dump);
			facer.v1 = verticesp[vertex1 - 1];
			facer.v2 = verticesp[vertex2 - 1];
			facer.v3 = verticesp[vertex3 - 1];
			facer.v4 = verticesp[vertex4 - 1];
			facep[count] = facer;
			count++;
		}
	}
	facec = count;
    fclose(file);
	return facep;
}

Vector* GetCenterOfMass(face* facep) {
	Vector* comp = (Vector*)malloc(facec * sizeof(Vector));
	face facer;
	Vector com;
	for (int i = 0; i < facec; i++) {
		facer = facep[i];
		com.x = (facer.v1.x + facer.v2.x + facer.v3.x + facer.v4.x) / 4;
		com.y = (facer.v1.y + facer.v2.y + facer.v3.y + facer.v4.y) / 4;
		com.z = (facer.v1.z + facer.v2.z + facer.v3.z + facer.v4.z) / 4;
		comp[i] = com;
	}
	return comp;
}

double Distance(Vector v1, Vector v2) {
	double dist = sqrt(pow(v1.x - v2.x, 2) + pow(v1.y - v2.y, 2) + pow(v1.z - v2.z, 2));
	return dist;
}

Vector* GetNormals(face* facep) {
	Vector* normalp = (Vector*)malloc(facec * sizeof(Vector));
	face facer;
	Vector normal, v1, v2, v3, v4;
	double length;
	for (int i = 0; i < facec; i++) {
		facer = facep[i];
		v1 = facer.v1;
		v2 = facer.v2;
		v3 = facer.v3;
		v4 = facer.v4;
		normal.x = (v1.y - v2.y) * (v1.z + v2.z) + (v2.y - v3.y) * (v2.z + v3.z) + (v3.y - v4.y) * (v3.z + v4.z) + (v4.y - v1.y) * (v4.z + v1.z);
		normal.y = (v1.z - v2.z) * (v1.x + v2.x) + (v2.z - v3.z) * (v2.x + v3.x) + (v3.z - v4.z) * (v3.x + v4.x) + (v4.z - v1.z) * (v4.x + v1.x);
		normal.z = (v1.x - v2.x) * (v1.y + v2.y) + (v2.x - v3.x) * (v2.y + v3.y) + (v3.x - v4.x) * (v3.y + v4.y) + (v4.x - v1.x) * (v4.y + v1.y);
		length = sqrt(pow(normal.x, 2) + pow(normal.y, 2) + pow(normal.z, 2));
		normal.x = normal.x / length;
		normal.y = normal.y / length;
		normal.z = normal.z / length;
		normalp[i] = normal;
	}
	return normalp;
}
Vector OrientToForward(double x, double y, double z, double w) {
	Vector forward;
	forward.x = 1 - 2 * (y * y + z * z);
	forward.y = 2 * (x * y + w * z);
	forward.z = 2 * (x * z - y * w);
	return forward;
}

Vector OrientToNormal(double x, double y, double z, double w) {
	Vector normal;
	normal.x = - 2 * (x * y - z * w);
	normal.y = -1 + 2 * (x * x + z * z);
	normal.z = -2 * (z * y + x * w);
	return normal;
}

double* dist(Vector posest) {
	double* distp = (double*)malloc(facec * sizeof(double));
	for (int i = 0; i < facec; i++) {
		distp[i] = Distance(comp[i], posest);
	}
	return distp;
}

Vector estcomf(double* distp, Vector norm) {
	double heuristic[facec];
	int maxind = -1;
	double maxheur = 0;
	for (int i = 0; i < facec; i++) {
		heuristic[i] = 1 / distp[i] + 0.1 / (Distance(norm, normalp[i]) + 5);
		if (heuristic[i] > maxheur) {
			maxind = i;
			maxheur = heuristic[i];
		}
	}
	if (maxind == -1) {
		exit(1);
	}
	return comp[maxind];
}



int init_localization()
{	
    std::cout << "init localization" << std::endl;
	std::string topic_sub = "/model/robot/odometry";   // subscribe to this topic
	// Subscribe to a topic by registering a callback.
	if (!node.Subscribe(topic_sub, cb))
	{
		std::cerr << "Error subscribing to topic [" << topic_sub << "]" << std::endl;
		return -1;
	}
	std::cout << "subscribed" << std::endl;// Zzzzzz.

	Vector* verticesp;
	verticesp = GetVertices();
	face* facep;
	std::cout << "dick" << std::endl;

	facep = GetFaces(verticesp);
	std::cout << "dick 2" << std::endl;

	comp = GetCenterOfMass(facep);
	normalp = GetNormals(facep);
    
	return 0;
}

std::string topic_force = "/world/world_test/wrench";
// Create a publisher on the EntityWrench topic
gz::transport::Node::Publisher pub_force;

Vector ResForce;

int init_force()
{
	std::cout << "Starting force publisher..." << std::endl;

	// Define the topic for applying force
	std::cout << "Attempting to advertise topic: " << topic_force << std::endl;

	// Create a publisher on the EntityWrench topic
	pub_force = node.Advertise<gz::msgs::EntityWrench>(topic_force);

	// Wait for a connection
	std::cout << "Waiting for connection on topic: " << topic_force << std::endl;
	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	if (!pub_force)
	{
		std::cerr << "Error: Could not create publisher on topic: " << topic_force << std::endl;
		return -1;
	}

	std::cout << "Publisher created successfully." << std::endl;

	ResForce.x = 0;
	ResForce.y = 0;
	ResForce.z = 0;

	//// Publish the force message immediately to ensure the topic shows up
	//for (int i = 0; i < 5; ++i)
	//{
	//    std::cout << "Publishing force... Iteration: " << i + 1 << std::endl;
	//    pub.Publish(msg);
	//    
	//    // Sleep for a second before sending the next message
	//    std::this_thread::sleep_for(std::chrono::seconds(1));
	//}

	//// Keep the node alive for a while to ensure messages are processed
	//std::this_thread::sleep_for(std::chrono::seconds(2));

	return 0;
}

void AddForce(Vector force)
{
	ResForce.x += force.x;
	ResForce.y += force.y;
	ResForce.z += force.z;
}


void PublishForce()
{
	// Create and configure the EntityWrench message
	gz::msgs::EntityWrench msg;
	msg.mutable_entity()->set_id(9);  // Set the entity ID to 9 (your robot's ID)

	// Set the force values
	msg.mutable_wrench()->mutable_force()->set_x(ResForce.x);//ResForce.x);  // 100N in X direction
	msg.mutable_wrench()->mutable_force()->set_y(ResForce.y);//ResForce.y);    // 0N in Y direction
	msg.mutable_wrench()->mutable_force()->set_z(ResForce.z);//ResForce.z); // 10,000N in Z direction

	// Set torque values (0 here, but can be modified)
	msg.mutable_wrench()->mutable_torque()->set_x(0.0);
	msg.mutable_wrench()->mutable_torque()->set_y(0.0);
	msg.mutable_wrench()->mutable_torque()->set_z(0.0);

	pub_force.Publish(msg);

	ResForce.x = 0;
	ResForce.y = 0;
	ResForce.z = 0;
}


int main(int argc, char** argv)
{
	std::cout << "Starting" << std::endl;

	//gz::client::setup(argc, argv)
	init_force();
	init_localization();

	std::cout << "Init finished" << std::endl;

	while (true)
	{
		std::cout << "Publishing force" << std::endl;
		Vector Force;
		Force.x = Up.x * -1000;
		Force.y = Up.y * -1000;
		Force.z = Up.z * -1000;
		AddForce(Force);
		Force.x = Forward.x * 2000;
		Force.y = Forward.y * 2000;
		Force.z = Forward.z * 2000;
		AddForce(Force);
        PublishForce();
		// Sleep for a second before sending the next message
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	// Keep the node alive for a while to ensure messages are processed
	std::this_thread::sleep_for(std::chrono::seconds(2));

	return 0;
}

