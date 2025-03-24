#include <gz/transport/Node.hh>
#include <gz/msgs/entity_wrench.pb.h>
#include <iostream>
#include <chrono>
#include <thread>

#include <ForceTest.cc>
#include <SensorCode.cc>

int main()
{
	std::cout << "Starting" << std::endl;

	init_force();
    init_localization();

	std::cout << "Init finished" << std::endl;

	//while (true)
	//{
	//	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	//}
    // Publish the force message immediately to ensure the topic shows up
    for (int i = 0; i < 5; ++i)
    {
        std::cout << "Publishing force... Iteration: " << i + 1 << std::endl;
        AddForce(Vector{Forward.x*1000.0f, Forward.y * 1000.0f, Forward.z * 1000.0f});
        
        // Sleep for a second before sending the next message
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Keep the node alive for a while to ensure messages are processed
    std::this_thread::sleep_for(std::chrono::seconds(2));

    
    return 0;
}

