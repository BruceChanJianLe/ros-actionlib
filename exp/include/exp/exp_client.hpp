#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <exp_msgs/ExpAction.h>

#include <string>
#include <memory>


namespace exp_client
{
    class exp_client
    {
    private:
        /// ROS node handles
        ros::NodeHandle relative_nh_;
        ros::NodeHandle private_nh_;

        /// Action client pointer
        std::shared_ptr<actionlib::SimpleActionClient<exp_msgs::ExpAction>> act_clt_;

        /// Action goal
        exp_msgs::ExpGoal goal_;

        /// Stop action client
        void stop();

    public:
        /// Constructor
        exp_client(std::string);

        /// Destructor
        ~exp_client();

        /// Start action client
        void start();
    };


} // namespace exp_client
