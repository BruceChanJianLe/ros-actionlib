#include "exp/exp_server.hpp"

const std::string ROSNodeName {"exp_server_node"};

int main(int argc, char ** argv)
{
    // Initialize ROS node
    ros::init(argc, argv, ROSNodeName);

    exp_server::exp_server node(ros::this_node::getName());

    node.start();

    ros::spin();

    return 0;
}