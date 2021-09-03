#include "exp/exp_client.hpp"

const std::string ROSNodeName {"exp_client_node"};


int main(int argc, char ** argv)
{
    // Initialize Ros node
    ros::init(argc, argv, ROSNodeName);

    exp_client::exp_client node("exp_server_node");

    node.start();

    return 0;
}