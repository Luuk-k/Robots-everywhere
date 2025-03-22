#include <gz/transport/Node.hh>
#include <gz/msgs/entity_wrench.pb.h>
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    std::cout << "Starting force publisher..." << std::endl;

    // Initialize Gazebo Transport Node
    gz::transport::Node node;

    // Define the topic for applying force
    std::string topic = "/world/world_test/wrench";
    std::cout << "Attempting to advertise topic: " << topic << std::endl;

    // Create a publisher on the EntityWrench topic
    gz::transport::Node::Publisher pub = node.Advertise<gz::msgs::EntityWrench>(topic);

    // Wait for a connection
    std::cout << "Waiting for connection on topic: " << topic << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    if (!pub)
    {
        std::cerr << "Error: Could not create publisher on topic: " << topic << std::endl;
        return -1;
    }

    std::cout << "Publisher created successfully. Publishing force..." << std::endl;

    // Create and configure the EntityWrench message
    gz::msgs::EntityWrench msg;
    msg.mutable_entity()->set_id(9);  // Set the entity ID to 9 (your robot's ID)

    // Set the force values
    msg.mutable_wrench()->mutable_force()->set_x(100.0);  // 100N in X direction
    msg.mutable_wrench()->mutable_force()->set_y(0.0);    // 0N in Y direction
    msg.mutable_wrench()->mutable_force()->set_z(10000.0); // 10,000N in Z direction

    // Set torque values (0 here, but can be modified)
    msg.mutable_wrench()->mutable_torque()->set_x(0.0);
    msg.mutable_wrench()->mutable_torque()->set_y(0.0);
    msg.mutable_wrench()->mutable_torque()->set_z(0.0);

    // Publish the force message immediately to ensure the topic shows up
    for (int i = 0; i < 5; ++i)
    {
        std::cout << "Publishing force... Iteration: " << i + 1 << std::endl;
        pub.Publish(msg);
        
        // Sleep for a second before sending the next message
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::cout << "Finished publishing." << std::endl;

    // Keep the node alive for a while to ensure messages are processed
    std::this_thread::sleep_for(std::chrono::seconds(5));

    return 0;
}

