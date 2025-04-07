#include <gz/msgs.hh>
#include <gz/transport.hh>
#include <gz/physics.hh>
#include <thread>
#include <cstdio>
#include <ctime>
#include <cmath>
#include <cstdlib>
#include <string>


// Custom 3D vector class
class Vector {
public: double x, y, z;
};

// Class for representing a face with a max of 4 vertices
class face {
public: Vector v1, v2, v3, v4;
};

// Node use for communication with the sim
gz::transport::Node node;

// variables needed to keep sync with the sim
bool CanStartTick = false;
float SimStartTime = 0.0f;

// State for the robot state machine
enum MoveState {
    begin,
    straight,
    turning,
	stop
};

// Robot movement variables
MoveState RobotMoveState = begin;
float MoveStrength = 2000.0f;
float MoveTime = 9.0f;
float StartMoveTime = 0.0f;
float RotateStrength = -250.0f;
Vector OldForward;
bool IsRotatingAround = false;
float WaitTime = 0.0f;
bool NeedsToStop = false;
float MaxSpeed = 2.0f;

// Variables used for localization
double velx = 0;
double vely = 0;
double velz = 0;
double vel = 0.0f;

double angpx = 0;
double angpy = 0;
double angpz = 0;

double posx,posy,posz;
double posxl,posyl,poszl;

double dt = 0.1;
double timestamp = 0;

int facec;
Vector* comp;
Vector* normalp;

// Robot up and forward vectors
Vector Forward;
Vector Up;

bool FoundCrack = false;

// Specify some functions
Vector OrientToNormal(double, double, double, double);
Vector OrientToForward(double, double, double, double);
double* dist(Vector);
Vector estcomf(double*, Vector);


void cb(const gz::msgs::Odometry& _msg)
{
	// Set to true to signal that a tick has happend
    CanStartTick = true;

	// Get the quaternion of the robot orientation 
	double orix = _msg.pose().orientation().x();
	double oriy = _msg.pose().orientation().y();
	double oriz = _msg.pose().orientation().z();
	double oriw = _msg.pose().orientation().w();

	// Get the normal and forward vectors
	Vector ori = OrientToNormal(orix, oriy, oriz, oriw);
	Up = ori;
	Forward = OrientToForward(orix, oriy, oriz, oriw);

	// Save the last position
	posxl = posx;
	posyl = posy;
	poszl = posz;

	// Get the new position
	posx = _msg.pose().position().x();
	posy = _msg.pose().position().y();
	posz = _msg.pose().position().z();

	// Save the velocity
	velx = (posx-posxl)/0.1;
	vely = (posy-posyl)/0.1;
	velz = (posz-poszl)/0.1;
	vel = sqrt(pow(velx,2.0f)+pow(vely,2.0f)+pow(velz,2.0f));

	timestamp = timestamp + dt;

	// Update the estimate position
	Vector estpos;
	estpos.x = posx;
	estpos.y = posy;
	estpos.z = posz;

	double* disp;
	disp = dist(estpos);
	Vector estcom;
	estcom = estcomf(disp, ori);

	float time = ((float)clock()/CLOCKS_PER_SEC) - SimStartTime;

	// Check if a crack is found
	if(time>45.0f && !FoundCrack){
		printf("\n------------------------------------\n");
		printf("\nFound a crack:\nx=%.3f, y=%.3f, z=%.3f\n", estpos.x, estpos.y, estpos.z);
		printf("\n------------------------------------\n");

		// Have the robot slow down and wait for 0.5 sec
		FoundCrack = true;
		NeedsToStop = true;
		WaitTime = 0.5f;

		// Write the crack location to a txt file
		FILE  * file = fopen("./data/crack2/location","w");
		fprintf(file,"Crack found at position:x=%f, y=%f, z=%f",estpos.x,estpos.y,estpos.z);
		//fclose(file); // Does not write to the file for some reason
	}
}

// Get the vertices from the .obj file
Vector* GetVertices() {
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
	return verticesp;
}

// Get the faces from the .obj file
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

// Returns a pointer to the vector of the center of mass of the robot
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

// Returns the distance between two vectors
double Distance(Vector v1, Vector v2) {
	double dist = sqrt(pow(v1.x - v2.x, 2) + pow(v1.y - v2.y, 2) + pow(v1.z - v2.z, 2));
	return dist;
}

// Returns a pointer to the normal vector of the inputed face
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

// Return the robot forward vector
Vector OrientToForward(double x, double y, double z, double w) {
	Vector forward;
	forward.x = 1 - 2 * (y * y + z * z);
	forward.y = 2 * (x * y + w * z);
	forward.z = 2 * (x * z - y * w);

	return forward;
}

// Return the robot normal vector `(aka robot up vector)
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

// Initialize the variables for localization and setup the communication with the sim
int init_localization()
{	
	printf("Init localization...\n");
	std::string topic_sub = "/model/robot/odometry";   // subscribe to this topic
	// Subscribe to a topic by registering a callback.
	if (!node.Subscribe(topic_sub, cb))
	{
		printf("Error subscribing to topic [%s]", topic_sub.c_str());
		exit(1);
	}

	Vector* verticesp;
	verticesp = GetVertices();
	face* facep;

	facep = GetFaces(verticesp);

	comp = GetCenterOfMass(facep);
	normalp = GetNormals(facep);
    
	return 0;
}

// Specify a topic and create a publisher
std::string topic_force = "/world/world_test/wrench";
gz::transport::Node::Publisher pub_force;

// Resulting torque and force. Used for publishing to the simulation
Vector ResForce;
Vector ResTorque;

// Initialize the force parameters and setup the connection with the sim
int init_force()
{
	printf("Init force...\n");


	// Create a publisher on the EntityWrench topic
	pub_force = node.Advertise<gz::msgs::EntityWrench>(topic_force);

	// Wait for a connection
	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	if (!pub_force)
	{
		printf("Error: Could not create publisher on topic: %s" ,topic_force.c_str());
		exit(1);
	}

	ResForce.x = 0;
	ResForce.y = 0;
	ResForce.z = 0;

	return 0;
}

// Add a force to the robot
void AddForce(Vector force)
{
	ResForce.x += force.x;
	ResForce.y += force.y;
	ResForce.z += force.z;
}

// Add torque to the robot
void addTorque(Vector torque)
{
	ResTorque.x += torque.x;
	ResTorque.y += torque.y;
	ResTorque.z += torque.z;
}

// Publish the forces to the specified topic, which is read by the sim
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
	msg.mutable_wrench()->mutable_torque()->set_x(ResTorque.x);
	msg.mutable_wrench()->mutable_torque()->set_y(ResTorque.y);
	msg.mutable_wrench()->mutable_torque()->set_z(ResTorque.z);

	// Publish the message to the topic
	pub_force.Publish(msg);

	// Reset the resulting force and torque
	ResForce.x = 0;
	ResForce.y = 0;
	ResForce.z = 0;

	ResTorque.x = 0;
	ResTorque.y = 0;
	ResTorque.z = 0;
}

// Return a vector of the magnetic force
Vector GetMagneticForce() 
{
    Vector _force;
	float _strength = 1500.0f;
	if (RobotMoveState == turning && !NeedsToStop)
		_strength = 100.0f;

    _force.x = -Up.x * _strength;
    _force.y = -Up.y * _strength;
    _force.z = -Up.z * _strength;
    
    return _force;  
}

// Algorithm to move the robot over the ship hull
void RobotMove ()
{
	// Slow down the robot
	if (NeedsToStop)
	{
		if (vel < 0.1f) 
		{
			NeedsToStop = false;
        	StartMoveTime = ((float) clock())/CLOCKS_PER_SEC; 	
			return;
		}

		Vector force;
		force.x = -Forward.x * MoveStrength * vel;
		force.y = -Forward.y * MoveStrength * vel;
		force.z = -Forward.z * MoveStrength * vel;

		AddForce(force);
		return;
	}

	// Let the robot wait
    if ((((float) clock())/CLOCKS_PER_SEC - StartMoveTime) < WaitTime) 
	{	
		return;
	}

	WaitTime = 0.0f;

	// If a crack is found the program exits
	if (FoundCrack) exit(1);

    switch (RobotMoveState) 
	{
    case begin: // Called before a straight part. It sets up the star time 
        StartMoveTime = ((float) clock())/CLOCKS_PER_SEC; 
        RobotMoveState = straight;
        break;
    case straight: // Move for a certain amount of time
        if ((((float) clock())/CLOCKS_PER_SEC - StartMoveTime) >= MoveTime) 
        {
            RobotMoveState = turning;

			if (vel > 0.1f) NeedsToStop = true;

			// Setup variable for the robot to wait
            StartMoveTime = ((float) clock())/CLOCKS_PER_SEC; 
			WaitTime = 1.0f;

            OldForward = Forward;
            break;
        }

		// Move forwards
        Vector ForwardForce;
        ForwardForce.x = Forward.x * MoveStrength * (MaxSpeed - vel);
        ForwardForce.y = Forward.y * MoveStrength * (MaxSpeed - vel);
        ForwardForce.z = Forward.z * MoveStrength * (MaxSpeed - vel);
        AddForce(ForwardForce);
        break;
    case turning:
        if ((OldForward.x * Forward.x + OldForward.y * Forward.y + OldForward.z * Forward.z) < 0.05f)
        {
			// Setup variables for the robot to wait
			StartMoveTime = ((float) clock())/CLOCKS_PER_SEC; 
			WaitTime = 1.0f;

			// Have to the robot move 0.5 secs if it already rotated once
			if (IsRotatingAround)
			{
				MoveTime = 9.0f;
				RotateStrength *= -1.0f;
				IsRotatingAround = false;
			} 
			else 
			{
				MoveTime = 0.5f;
				IsRotatingAround = true;
			}

			// Apply a torque in the opposite direction
        	Vector NegTorque;
        	NegTorque.x = -Up.x * RotateStrength; 
       	 	NegTorque.y = -Up.y * RotateStrength; 
        	NegTorque.z = -Up.z * RotateStrength; 
        	addTorque(NegTorque);

			// Robot goes into begin to start with a straight movement
            RobotMoveState = begin;
            break;
        }

		// Rotate the robot
        Vector Torque;
        Torque.x = Up.x * RotateStrength; 
        Torque.y = Up.y * RotateStrength; 
        Torque.z = Up.z * RotateStrength; 
        addTorque(Torque);

        break;
	case stop: // Do not move the robot
		exit(1);
		break;
    }
}

// Print the data about position and forces on the terminal
void PrintData()
{
	if (FoundCrack) return;
	printf("\n--------------|Data|----------------\n");
	printf("Time = %.3f s\n\n", ((float)clock()/CLOCKS_PER_SEC) - SimStartTime);
    printf("Applied forces:\nx=%.3f, y=%.3f, z=%.3f\n", ResForce.x, ResForce.y, ResForce.z);
    printf("Applied torque:\nx=%.3f, y=%.3f, z=%.3f\n\n", ResTorque.x, ResTorque.y, ResTorque.z);
	printf("Orientation:\nx=%.3f, y=%.3f, z=%.3f\n", Up.x, Up.y, Up.z);
    printf("Lin Vel:\nx=%.3f, y=%.3f, z=%.3f\n", velx, vely, velz);
	printf("Lin Pos:\nx=%.3f, y=%.3f, z=%.3f\n", posx, posy, posz);
	printf("\n------------------------------------\n");
}

// Initiatlize the RobotClient
void init ()
{
    printf("Starting...\n\n");
	init_force();
	init_localization();

	printf("\nInit finished\n\n");
}

// Repeatedly run to apply force and move the robot
void run()
{
    RobotMove();

    // Add the force created by the magnet
    AddForce(GetMagneticForce());

	PrintData();

    // Add the force to the robot
    PublishForce();
}

int main()
{	
	// Initialize the code
    init();

    // Wait for the sim to start
	printf("Waiting for start of simulation...\n");
    while(!CanStartTick) {}
    SimStartTime = ((float)clock())/CLOCKS_PER_SEC; 

    // Looping behavior
    while (true)
    { 
        // Wait to start a tick
        while(!CanStartTick) {}

        run();

        // Set the bool back to false in order to wait for the next tick
        CanStartTick = false;
    }

	return 0;
}

