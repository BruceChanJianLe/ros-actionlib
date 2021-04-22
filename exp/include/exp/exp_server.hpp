#include <ros/ros.h>

#include <exp_msgs/ExpAction.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>

#include <string>
#include <memory>

namespace exp_server
{
    class exp_server
    {
    private:
        /// Action Server
        std::shared_ptr<actionlib::SimpleActionServer<exp_msgs::ExpAction>> act_srv_;
        // actionlib::SimpleActionServer<exp_msgs::ExpAction> act_srv_;

        /// Feedback
        exp_msgs::ExpActionFeedback feedback_;

        /// Result
        exp_msgs::ExpActionResult result_;

        /// Name
        std::string name_;

        /// ROS Node Handles
        ros::NodeHandle relative_nh_;
        ros::NodeHandle private_nh_;

        /// Action server callback
        void actionServerCB(const exp_msgs::ExpGoalConstPtr &);

        /// Stop action server
        void stop();

    public:
        /// Constructor
        exp_server(std::string);

        /// Destructor
        ~exp_server();

        /// Start action server
        void start();

    };

} // namespace exp_server
