import os
import mujoco
import glfw
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState

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
        self.sim_timer = self.create_timer(0.001, self.simulation_step)

        # Initialize the additional required objects
        self.opt = mujoco.MjvOption()      # Render options
        self.pert = mujoco.MjvPerturb()    # For user perturbations
        mujoco.mjv_defaultOption(self.opt)
        mujoco.mjv_defaultPerturb(self.pert)

        # Initialize wrench storage for each foot (set to None initially)
        self.wrenches = {
            "fl_foot": None,
            "fr_foot": None,
            "rl_foot": None,
            "rr_foot": None
        }

        self.feet_pos = {
            "fl_foot": [0.0054, 0.200, 0],
            "fr_foot": [0.0054, -0.200, 0],
            "rl_foot": [0.0054, 0.200, 0],
            "rr_foot": [0.0054, -0.200, 0]
        }

        self.nominal_joint_positions = {
            'fr_j0': -0.1,
            'fr_j1': 0.8,
            'fr_j2': -1.5,
            'fl_j0': 0.1,
            'fl_j1': -0.8,
            'fl_j2': 1.5,
            'rl_j0': -0.1,
            'rl_j1': -1.0,
            'rl_j2': 1.5,
            'rr_j0': 0.1,
            'rr_j1': 1.0,
            'rr_j2': -1.5,
            'sp_j0': 0.0
        }

        self.set_nominal_joint_positions()

        # Create subscribers for each foot's wrench topic
        self.create_subscription(WrenchStamped, "/front_left_foot/wrench", self.front_left_wrench_callback, 10)
        self.create_subscription(WrenchStamped, "/front_right_foot/wrench", self.front_right_wrench_callback, 10)
        self.create_subscription(WrenchStamped, "/rear_left_foot/wrench", self.rear_left_wrench_callback, 10)
        self.create_subscription(WrenchStamped, "/rear_right_foot/wrench", self.rear_right_wrench_callback, 10)

        # JointState publisher for publishing joint states
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)

    def front_left_wrench_callback(self, msg):
        self.wrenches["fl_foot"] = msg

    def front_right_wrench_callback(self, msg):
        self.wrenches["fr_foot"] = msg

    def rear_left_wrench_callback(self, msg):
        self.wrenches["rl_foot"] = msg

    def rear_right_wrench_callback(self, msg):
        self.wrenches["rr_foot"] = msg

    def apply_wrench_to_body(self, body_name, wrench_msg, site_name):
        if wrench_msg is not None:
            force = [wrench_msg.wrench.force.x, wrench_msg.wrench.force.y, wrench_msg.wrench.force.z]
            torque = [wrench_msg.wrench.torque.x, wrench_msg.wrench.torque.y, wrench_msg.wrench.torque.z]
            body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
            if body_id != -1:
                mujoco.mj_applyFT(self.model, self.data, force, torque, self.feet_pos[site_name], body_id, self.data.qfrc_applied)
            self.wrenches[body_name] = None

    def simulation_step(self):
        # Apply wrenches to each foot, then reset them
        self.apply_wrench_to_body("fl_l2", self.wrenches["fl_foot"], "fl_foot")
        self.apply_wrench_to_body("fr_l2", self.wrenches["fr_foot"], "fr_foot")
        self.apply_wrench_to_body("rl_l2", self.wrenches["rl_foot"], "rl_foot")
        self.apply_wrench_to_body("rr_l2", self.wrenches["rr_foot"], "rr_foot")

        # Step the MuJoCo simulation
        mujoco.mj_step(self.model, self.data)

        # Publish joint states
        self.publish_joint_states()

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

    def publish_joint_states(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        joint_state_msg.name = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(self.model.njnt)]
        joint_state_msg.position = self.data.qpos[:self.model.njnt]
        joint_state_msg.velocity = self.data.qvel[:self.model.njnt]
        joint_state_msg.effort = self.data.qfrc_applied[:self.model.njnt]

        # Publish the JointState message
        self.joint_state_publisher.publish(joint_state_msg)

    def set_nominal_joint_positions(self):
        for joint_name, position in self.nominal_joint_positions.items():
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            joint_qpos_addr = self.model.jnt_qposadr[joint_id]
            self.data.qpos[joint_qpos_addr] = position
        mujoco.mj_forward(self.model, self.data)

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
