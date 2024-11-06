#include <rclcpp/rclcpp.hpp>
#include <mujoco/mujoco.h>
#include <string>

#include <contact_estimation_testbed/contact_estimation_testbed.hpp>


ContactEstimationTestbed::ContactEstimationTestbed(): Node("contact_estimation_testbed")
{
    RCLCPP_INFO(this->get_logger(), "Hello, world!");
}


int main(int argc, char *argv[])
{

    // Load the MuJoCo model from XML
    std::string model_path = "/home/dyuman/Documents/TUD/contact_testbed_ros2_ws/src/contact-estimation-testbed/contact_estimation_testbed/urdf/silver_badger_testbed.urdf";
    mjModel* m = mj_loadXML(model_path.c_str(), nullptr, nullptr, 0);
    if (!m) {
        std::cerr << "Could not load MuJoCo model." << std::endl;
        return 1;
    }

    // Create a data structure for the simulation
    mjData* d = mj_makeData(m);

    // Simulation loop
    std::cout << "Starting MuJoCo simulation..." << std::endl;
    for (int i = 0; i < 1000; ++i) {  // Run 1000 simulation steps
        // Advance the simulation by one time step
        mj_step(m, d);

        // Print the position of the first joint as an example
        std::cout << "Step " << i << ": joint position = " << d->qpos[0] << std::endl;

        // Sleep to slow down the loop (optional)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Free MuJoCo data and deactivate
    mj_deleteData(d);
    mj_deleteModel(m);


    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ContactEstimationTestbed>());
    rclcpp::shutdown();
    return 0;
}