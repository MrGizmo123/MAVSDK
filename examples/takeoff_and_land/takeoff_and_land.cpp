//
// Simple example to demonstrate how takeoff and land using MAVSDK.
//

#include <vector>
#include <limits>
#include <chrono>
#include <cstdint>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <iostream>
#include <future>
#include <memory>
#include <thread>
#include <mavsdk/plugins/mocap/mocap.h>

using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

void usage(const std::string& bin_name)
{
    std::cerr << "Usage : " << bin_name << " <pixhawk_connection_url> <forward_connection_1> <forward_connection_2>\n"
              << "Connection URL format should be :\n"
              << " For TCP server: tcpin://<our_ip>:<port>\n"
              << " For TCP client: tcpout://<remote_ip>:<port>\n"
              << " For UDP server: udp://<our_ip>:<port>\n"
              << " For UDP client: udp://<remote_ip>:<port>\n"
              << " For Serial : serial://</path/to/serial/dev>:<baudrate>]\n"
              << " For example, to connect to the simulator use URL: udpin://0.0.0.0:14540";
}

int main(int argc, char** argv)
{
    if (argc != 4) {
        usage(argv[0]);
        return 1;
    }

    Mavsdk mavsdk{Mavsdk::Configuration{ComponentType::GroundStation}};
    ConnectionResult connection_result = mavsdk.add_any_connection(argv[1], ForwardingOption::ForwardingOn);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Pixhawk Connection failed: " << connection_result << '\n';
        return 1;
    }

    connection_result = mavsdk.add_any_connection(argv[2], ForwardingOption::ForwardingOn);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Forwarding Connection 1 failed: " << connection_result << '\n';
        return 1;
    }

    connection_result = mavsdk.add_any_connection(argv[3], ForwardingOption::ForwardingOn);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Forwarding Connection 2 failed: " << connection_result << '\n';
        return 1;
    }




    auto system = mavsdk.first_autopilot(20.0);
    if (!system) {
        std::cerr << "Timed out waiting for system\n";
        return 1;
    }

    // Instantiate plugins.
    auto telemetry = Telemetry{system.value()};
    auto action = Action{system.value()};
    
    auto mocap = Mocap{system.value()};

    // We want to listen to the altitude of the drone at 1 Hz.
    // const auto set_rate_result = telemetry.set_rate_position(1.0);
    // if (set_rate_result != Telemetry::Result::Success) {
    //     std::cerr << "Setting rate failed: " << set_rate_result << '\n';
    //     return 1;
    // }

    // // Set up callback to monitor altitude while the vehicle is in flight
    // telemetry.subscribe_position([](Telemetry::Position position) {
    //   std::cout << "Altitude: " << position.relative_altitude_m << " m\n";
    // });

    //mocap.set_gps_global_origin(origin);
    // // Check until vehicle is ready to arm
    // while (telemetry.health_all_ok() != true) {
    // 	std::cout << telemetry << std::endl;
    // 	std::cout << "Vehicle is getting ready to arm\n";
    //     sleep_for(seconds(1));
    // }

    // Arm vehicle
    // std::cout << "Arming...\n";
    // const Action::Result arm_result = action.arm();

    // if (arm_result != Action::Result::Success) {
    //     std::cerr << "Arming failed: " << arm_result << '\n';
    //     return 1;
    // }

    std::cout << "sending mocap" << std::endl;
    
    while (true)
    {
      Mocap::PositionBody pos_body = {
	.x_m = 0,
	.y_m = 0,
	.z_m = 0
      };

      Mocap::AngleBody ang_body = {
	.roll_rad = 0,
	.pitch_rad = 0,
	.yaw_rad = 0,
      };

      float not_a_number = std::numeric_limits<float>::quiet_NaN();
      std::vector<float> cov_matrix = { not_a_number };
      
      Mocap::VisionPositionEstimate est = {
	.time_usec = 0,
	.position_body = pos_body,
	.angle_body = ang_body,
	.pose_covariance = cov_matrix
      };
      
      
      mocap.set_vision_position_estimate(est);

      sleep_for(std::chrono::milliseconds(17));
      
      // std::cout << "sending done" << std::endl;
    }
    
    return -1;
}
