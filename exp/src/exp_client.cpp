#include "exp/exp_client.hpp"


namespace exp_client
{
    exp_client::exp_client(std::string server_name)
    :   relative_nh_(ros::NodeHandle()),
        private_nh_(ros::NodeHandle("~"))
    {
        act_clt_ = std::make_shared<actionlib::SimpleActionClient<exp_msgs::ExpAction>> (
            server_name,
            true
        );

        goal_.target_pose.pose.position.x = 10.0;
        goal_.target_pose.pose.position.y = 5.0;
    }


    exp_client::~exp_client()
    {
        stop();
    }


    void exp_client::start()
    {
        // Wait for sever to be online, ture if connected in stipulated time
        if(!act_clt_->waitForServer(ros::Duration(30.0)))
        {
            ROS_INFO_STREAM(
                ros::this_node::getName() <<
                " server not available!"
            );
        }
        else
        {
            // Send goal to server
            act_clt_->sendGoal(goal_);

            // Wait for result for a certain amount of time
            if(act_clt_->waitForResult(ros::Duration(100.0)))
            {
                actionlib::SimpleClientGoalState state = act_clt_->getState();
                ROS_INFO_STREAM(
                    ros::this_node::getName() << 
                    " Action finished: " << state.toString().c_str()
                );
            }
            else
            {
                actionlib::SimpleClientGoalState state = act_clt_->getState();
                ROS_INFO_STREAM(
                    ros::this_node::getName() << 
                    " Action did not finish in time: " << state.toString().c_str()
                );
            }
        }

    }


    void exp_client::stop()
    {
        // Cancel all request to server
        act_clt_->cancelAllGoals();
    }
} // namespace exp_client
