#include "exp/exp_server.hpp"


namespace exp_server
{
    exp_server::exp_server(std::string action_name)
    :   name_(action_name),
        private_nh_(ros::NodeHandle("~")),
        relative_nh_(ros::NodeHandle())
    {
        // Initialize action server
        act_srv_ = std::make_shared<actionlib::SimpleActionServer<exp_msgs::ExpAction>> (
            relative_nh_,
            action_name,
            [this](const exp_msgs::ExpGoalConstPtr & goal)
            {
                this->actionServerCB(goal);
            },
            false
        );

    }

    exp_server::~exp_server()
    {
        stop();
    }


    void exp_server::start()
    {
        act_srv_->start();
    }


    void exp_server::stop()
    {
        act_srv_->shutdown();
    }


    void exp_server::actionServerCB(const exp_msgs::ExpGoalConstPtr & goal)
    {
        ROS_INFO_STREAM(
            ros::this_node::getName() << " " << __func__ <<
            " action server request has been received."
        );

        geometry_msgs::PoseStamped begining_pose;
        begining_pose.pose.position.x = 0.0;
        begining_pose.pose.position.y = 0.0;

        // Success flag
        bool isok = false;

        ros::Rate r(5);
        // Carry out goal request
        while(ros::ok())
        {
            if(!act_srv_->isPreemptRequested())
            {
                if(
                    (int) goal->target_pose.pose.position.x == (int) begining_pose.pose.position.x
                    && (int) goal->target_pose.pose.position.y == (int) begining_pose.pose.position.y
                )
                {
                    isok = true;
                    break;
                }
                else if(
                    (int) goal->target_pose.pose.position.x < (int) begining_pose.pose.position.x
                    && (int) goal->target_pose.pose.position.y < (int) begining_pose.pose.position.y
                )
                {
                    break;
                }
                else
                {
                    if((int) goal->target_pose.pose.position.x > (int) begining_pose.pose.position.x)
                        begining_pose.pose.position.x += 1.0;
                    if((int) goal->target_pose.pose.position.y > (int) begining_pose.pose.position.y)
                        begining_pose.pose.position.y += 1.0;

                    // Update feedback with current pose
                    feedback_.feedback.base_position = begining_pose;

                    act_srv_->publishFeedback(feedback_.feedback);
                    ROS_INFO_STREAM(
                        ros::this_node::getName() << " " << __func__ << 
                        " current x pose: " << (int) begining_pose.pose.position.x << 
                        " current y pose: " << (int) begining_pose.pose.position.y 
                    );
                }
                r.sleep();
            }
            else
            {
                ROS_INFO_STREAM(
                    ros::this_node::getName() <<
                    " server has been preempted."
                );
                // Set server as preempted
                act_srv_->setPreempted();
                return;
            }
        }

        // Determine if service call was successful
        if(isok)
        {
            act_srv_->setSucceeded(result_.result, "Successful service call");
        }
        else
        {
            act_srv_->setAborted(result_.result, "Failed service call");
        }
    }
} // namespace exp_server
