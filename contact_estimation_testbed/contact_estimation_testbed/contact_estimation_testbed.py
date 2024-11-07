import os
import mujoco
import glfw
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped



class ContactEstimationTestbed(Node):
    def __init__(self):
        super().__init__("contact_estimation_testbed")

        # Initialize GLFW
        if not glfw.init():
            self.get_logger().error("Could not initialize GLFW")
            return

        # Load MuJoCo model
        self.declare_parameter("mujoco_file", "")
        mujoco_file = self.get_parameter("mujoco_file").get_parameter_value().string_value

        if not mujoco_file or not os.path.exists(mujoco_file):
            self.get_logger().error("MuJoCo file path is invalid.")
            return
        self.model = mujoco.MjModel.from_xml_path(mujoco_file)
        self.data = mujoco.MjData(self.model)

        # Create a GLFW window for visualization
        self.window = glfw.create_window(1200, 900, "MuJoCo Contact Force Testbed", None, None)
        if not self.window:
            self.get_logger().error("Could not create GLFW window")
            glfw.terminate()
            return

        glfw.make_context_current(self.window)
        glfw.swap_interval(1)

        # Set up MuJoCo visualization scene and context
        self.scene = mujoco.MjvScene(self.model, maxgeom=10000)
        self.mujoco_context = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150)

        # Set up the camera
        self.cam = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(self.cam)
        self.cam.distance = 2.5

        # Create a timer to update the simulation as fast as possible for best rendering
        self.timer = self.create_timer(0.0, self.simulation_step)

        # Initialize the additional required objects
        self.opt = mujoco.MjvOption()      # Render options
        self.pert = mujoco.MjvPerturb()    # For user perturbations
        mujoco.mjv_defaultOption(self.opt)
        mujoco.mjv_defaultPerturb(self.pert)

        # Initialize wrench storage for each foot (set to None initially)
        self.wrenches = {
            "front_left": None,
            "front_right": None,
            "rear_left": None,
            "rear_right": None
        }

        # Create subscribers for each foot's wrench topic
        self.create_subscription(WrenchStamped, "/front_left_foot/wrench", self.front_left_wrench_callback, 10)
        self.create_subscription(WrenchStamped, "/front_right_foot/wrench", self.front_right_wrench_callback, 10)
        self.create_subscription(WrenchStamped, "/rear_left_foot/wrench", self.rear_left_wrench_callback, 10)
        self.create_subscription(WrenchStamped, "/rear_right_foot/wrench", self.rear_right_wrench_callback, 10)

    def front_left_wrench_callback(self, msg):
        self.wrenches["front_left"] = msg

    def front_right_wrench_callback(self, msg):
        self.wrenches["front_right"] = msg

    def rear_left_wrench_callback(self, msg):
        self.wrenches["rear_left"] = msg

    def rear_right_wrench_callback(self, msg):
        self.wrenches["rear_right"] = msg

    def apply_wrench_to_body(self, body_name, wrench_msg):
        if wrench_msg is not None:
            # Apply the wrench once
            force = [wrench_msg.wrench.force.x, wrench_msg.wrench.force.y, wrench_msg.wrench.force.z]
            torque = [wrench_msg.wrench.torque.x, wrench_msg.wrench.torque.y, wrench_msg.wrench.torque.z]
            body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
            if body_id != -1:
                # Apply force and torque to the specified body in the MuJoCo simulation
                mujoco.mj_applyFT(self.model, self.data, force, torque, body_id, self.data.qfrc_applied)

            # Reset the wrench after applying it once
            self.wrenches[body_name] = None

    def simulation_step(self):
        # Apply wrenches to each foot, then reset them
        self.apply_wrench_to_body("front_left_foot", self.wrenches["front_left"])
        self.apply_wrench_to_body("front_right_foot", self.wrenches["front_right"])
        self.apply_wrench_to_body("rear_left_foot", self.wrenches["rear_left"])
        self.apply_wrench_to_body("rear_right_foot", self.wrenches["rear_right"])

        # Step the MuJoCo simulation
        mujoco.mj_step(self.model, self.data)

        # Render the scene
        width, height = glfw.get_framebuffer_size(self.window)
        viewport = mujoco.MjrRect(0, 0, width, height)

        # Update the scene with the correct arguments
        mujoco.mjv_updateScene(self.model, self.data, self.opt, self.pert, self.cam, mujoco.mjtCatBit.mjCAT_ALL, self.scene)
        mujoco.mjr_render(viewport, self.scene, self.mujoco_context)

        # Swap buffers and poll GLFW events
        glfw.swap_buffers(self.window)
        glfw.poll_events()

        # Check if the window should close
        if glfw.window_should_close(self.window):
            self.get_logger().info("Window closed by user")
            self.destroy_node()

    def __del__(self):
        # Clean up GLFW and MuJoCo resources on exit
        if self.window:
            glfw.destroy_window(self.window)
        glfw.terminate()


def main(args=None):
    rclpy.init(args=args)
    mujoco_node = ContactEstimationTestbed()
    try:
        rclpy.spin(mujoco_node)
    except KeyboardInterrupt:
        pass
    finally:
        mujoco_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
