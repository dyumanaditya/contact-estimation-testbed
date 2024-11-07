import os
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import numpy as np

class JointStatePlotter(Node):
    def __init__(self):
        super().__init__('joint_state_plotter')

        # Load configuration from YAML file
        # Load joints to plot from YAML file
        self.declare_parameter("yaml_file", "")
        yaml_file_path = self.get_parameter("yaml_file").get_parameter_value().string_value
        config = self.load_config_from_yaml(yaml_file_path)

        # Extract joints and plot types from the config
        self.joints_to_plot = config.get('joints', [])
        self.plots_to_display = config.get('plots', [])

        if not self.joints_to_plot:
            self.get_logger().error("No joints specified for plotting in the YAML file.")
            return
        if not self.plots_to_display:
            self.get_logger().error("No plot types specified in the YAML file.")
            return

        # Subscribe to the joint states topic
        self.subscription = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)

        # Initialize data storage for joint states
        self.positions = {joint: [] for joint in self.joints_to_plot}
        self.velocities = {joint: [] for joint in self.joints_to_plot}
        self.efforts = {joint: [] for joint in self.joints_to_plot}

        # Set up subplots based on selected plot types
        num_plots = len(self.plots_to_display)
        self.fig, self.axes = plt.subplots(num_plots, 1, figsize=(20, 5 * num_plots))
        if num_plots == 1:
            self.axes = [self.axes]  # Ensure axes is always a list

        # Title for each subplot based on the selected plot types
        for ax, plot_type in zip(self.axes, self.plots_to_display):
            ax.set_title(f'Joint {plot_type.capitalize()}')
            ax.set_xlabel("Time Step")
            if plot_type == 'position':
                ax.set_ylabel("Position (rad)")
            elif plot_type == 'velocity':
                ax.set_ylabel("Velocity (rad/s)")
            elif plot_type == 'effort':
                ax.set_ylabel("Effort (Nm)")

        self.plot_initialized = False

        # Enable non-blocking interactive mode
        self.plot_timer = self.create_timer(0.1, self.update_plot)  # Update plot every 0.1 seconds

    def load_config_from_yaml(self, yaml_file_path):
        """Load the joint names and plot types from a YAML file."""
        if not os.path.exists(yaml_file_path):
            self.get_logger().error(f"YAML file {yaml_file_path} not found.")
            return {}

        with open(yaml_file_path, 'r') as file:
            try:
                return yaml.safe_load(file)
            except yaml.YAMLError as exc:
                self.get_logger().error(f"Error reading YAML file: {exc}")
                return {}

    def joint_state_callback(self, msg):
        # Filter only the joints specified in the YAML file
        for i, name in enumerate(msg.name):
            if name in self.joints_to_plot:
                
                self.positions[name].append(msg.position[i])
                self.velocities[name].append(msg.velocity[i])
                self.efforts[name].append(msg.effort[i])

                # Limit the length of data to avoid excessive memory usage
                max_len = 100  # Set maximum number of time steps to keep
                if len(self.positions[name]) > max_len:
                    self.positions[name].pop(0)
                    self.velocities[name].pop(0)
                    self.efforts[name].pop(0)

        self.plot_initialized = True

    def update_plot(self):
        if not self.plot_initialized:
            return  # Wait until the first joint state message arrives

        # Clear and update each specified plot
        for ax, plot_type in zip(self.axes, self.plots_to_display):
            ax.cla()  # Clear the axis

            # Plot the data for each specified joint
            if plot_type == 'position':
                for joint in self.joints_to_plot:
                    ax.plot(self.positions[joint], label=joint)
            elif plot_type == 'velocity':
                for joint in self.joints_to_plot:
                    ax.plot(self.velocities[joint], label=joint)
            elif plot_type == 'effort':
                for joint in self.joints_to_plot:
                    ax.plot(self.efforts[joint], label=joint)

            # Add labels and legend
            ax.set_title(f'Joint {plot_type.capitalize()}')
            ax.set_xlabel("Time Step")
            if plot_type == 'position':
                ax.set_ylabel("Position (rad)")
            elif plot_type == 'velocity':
                ax.set_ylabel("Velocity (rad/s)")
            elif plot_type == 'effort':
                ax.set_ylabel("Effort (Nm)")
            ax.legend(loc='upper right')

        # Redraw the plot
        plt.draw()
        plt.pause(0.001)  # Small pause to allow the plot to update

    def save_plot(self):
        """Save the final plot to a file before shutdown."""
        self.fig.savefig("joint_states_plot.png")  # Save as PNG, change as needed
        self.get_logger().info("Final plot saved as 'joint_states_plot.png'")

    def destroy_node(self):
        # Save the plot before shutting down
        self.save_plot()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    plotter = JointStatePlotter()

    try:
        rclpy.spin(plotter)
    except KeyboardInterrupt:
        pass
    finally:
        plotter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
