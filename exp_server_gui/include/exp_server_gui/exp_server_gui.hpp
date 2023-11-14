#pragma once


#include <ros/ros.h>

#include <rqt_gui_cpp/plugin.h>
#include <exp_server_gui/ui_exp_server_gui.h>
#include <QWidget>

#include <mbf_msgs/MoveBaseAction.h>
#include <mbf_msgs/MoveBaseActionResult.h>
#include <mbf_msgs/MoveBaseGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>

namespace exp_server
{
    enum serverState
    {
        PREEMPTED,
        ABORTED,
        SUCCEEDED,
        IDLE,
        CANCEL
    };

    class exp_server_gui : public rqt_gui_cpp::Plugin
    {
        Q_OBJECT
        public:
            exp_server_gui();
            ~exp_server_gui();

            virtual void initPlugin(qt_gui_cpp::PluginContext & context) override;
            virtual void shutdownPlugin() override;
            virtual void saveSettings(
                qt_gui_cpp::Settings & plugin_settings,
                qt_gui_cpp::Settings & instance_settings
            ) const override;
            virtual void restoreSettings(
                const qt_gui_cpp::Settings & plugin_settings,
                const qt_gui_cpp::Settings & instance_settings
            ) override;

        private:
            Ui::gui ui_;
            QWidget* widget_;

        protected:
            /// Action Client
            std::shared_ptr<actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>> act_client_;

            /// Feedback
            mbf_msgs::MoveBaseFeedback feedback_;

            /// Result
            mbf_msgs::MoveBaseResult result_;

            /// Name
            std::string name_;

            /// ROS Node Handles
            ros::NodeHandle relative_nh_;
            ros::NodeHandle private_nh_;

            /// Stop action server
            void stop();

            /// Current State
            serverState state_;

            bool alternate_goals_;

        private Q_SLOTS:
            // Qt callback function
            void buttonSendGoal();
            void buttonCancelGoal();
            void buttonCancelAllGoal();
            void buttonStopTrackingGoal();
    };

} // namespace exp_server
