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
            // if(act_clt_->waitForResult(ros::Duration(100.0)))
            // Or wait for indefinited amount of time
            if(act_clt_->waitForResult(ros::Duration(1.0)))
            {
                auto res = act_clt_->getState();
                
                // Handle server's state
                switch (res.state_)
                {
                case actionlib::SimpleClientGoalState::PREEMPTED:
                    ROS_INFO_STREAM(
                        ros::this_node::getName()
                        << " "
                        << __func__
                        << " server state is "
                        << res.toString().c_str()
                    );
                    break;

                case actionlib::SimpleClientGoalState::ABORTED:
                    ROS_INFO_STREAM(
                        ros::this_node::getName()
                        << " "
                        << __func__
                        << " server state is "
                        << res.toString().c_str()
                    );
                    break;

                case actionlib::SimpleClientGoalState::SUCCEEDED:
                    ROS_INFO_STREAM(
                        ros::this_node::getName()
                        << " "
                        << __func__
                        << " server state is "
                        << res.toString().c_str()
                    );
                    break;

                default:
                    ROS_INFO_STREAM(
                        ros::this_node::getName()
                        << " "
                        << __func__
                        << " server default state!"
                    );
                    break;
                };
            }
            else
            {
                ROS_INFO_STREAM(
                    ros::this_node::getName()
                    << " "
                    << __func__
                    << " canceling goal as intended."
                );
                act_clt_->cancelAllGoals();
            }
        }

    }


    void exp_client::stop()
    {
        // Cancel all request to server
        act_clt_->cancelAllGoals();
    }
} // namespace exp_client
