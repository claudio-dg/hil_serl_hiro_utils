from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Percorso dinamico al file YAML
    params_file = PathJoinSubstitution([
        FindPackageShare("hil_serl_hiro_utils"),
        "config",
        "state_bridge_params.yaml"
    ])

    # Nodo StateBridgeNode
    state_bridge_node = Node(
        package="hil_serl_hiro_utils",
        executable="StateBridgeNode.py",
        name="state_bridge_node",
        parameters=[params_file],
        output="screen",
    )

    return LaunchDescription([
        state_bridge_node
    ])