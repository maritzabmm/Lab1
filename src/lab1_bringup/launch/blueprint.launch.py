from launch import LaunchDescription
from launch_ros.actions import Node

# exactly this name!
def generate_launch_description():

    #instantiate a LaunchDescription object
    ld = LaunchDescription()

    first_node = Node(

        package = 'task3',     # package name

        executable = 'service',  # node's executable name

        output = 'screen', # log = redirects the output to a log file based on log level,
                    # screen = Explicitly prints subscriber output to the terminal

        # parameters=[{'log_level': 'WARN'}] # Set the log level as a parameter
        # OR
        # arguments=['--ros-args', '--log-level', 'simple_publisher:=WARN']

        # parameters = [{'x1': 1.0,
        #             'y1': 1.0,
        #             'x2': 2.0,
        #             'y2': 2.0}]  # declare parameters of the node <parameter_name> <default value>
                                        # (if the value is not string, omit quotes
    )

    second_node = Node(
        ## same structure
        package = 'task3',     # package name

        executable = 'client',  # node's executable name

        output = 'screen', # log = redirects the output to a log file based on log level,
                    # screen = Explicitly prints subscriber output to the terminal
    )

    # add as much nodes needed for your application

    # launch description
    ld.add_action(first_node)
    ld.add_action(second_node)

    return ld