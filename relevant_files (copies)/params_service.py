import rclpy
from rclpy.node import Node

from adre_ros.commons.transformations.messages_conversion import dictionary_to_params
from adre_lib.parameters.parameters_parser import ParametersParserYAML

from communication_interfaces.srv import Params


class ParametersService(Node):
    """
    Handles the spawning of the ego vehicle and its sensors

    Derive from this class and implement method sensors()
    """

    def __init__(self, srv_name):
        super().__init__(srv_name)

        # Declare parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ('topic_parameters', "/params/param_srv"),
                ('parameter_file', "''")
            ]
        )

        # Get parameters
        parameter_file = self.get_parameter("parameter_file").get_parameter_value().string_value
        topic_parameters = self.get_parameter("topic_parameters").get_parameter_value().string_value

        self.params = ParametersParserYAML(parameter_file)
        self.srv_vehicle = self.create_service(Params, topic_parameters, self.get_parameters)

    def get_parameters(self, request, response):
        """
        Get parameter information from the ego vehicle
        """
        response = dictionary_to_params(self.params.read(request.groups))
        return response


def main():
    """
    main function
    """
    srv_name = 'params_service'
    rclpy.init()
    static_parameters_srv = ParametersService(srv_name)
    static_parameters_srv.get_logger().info("\033[1m{}\033[0m Node Active".format(srv_name))
    rclpy.spin(static_parameters_srv)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
