#pragma once


#include <ros/ros.h>

#include <rqt_gui_cpp/plugin.h>
#include <exp_server_gui/ui_exp_server_gui.h>
#include <QWidget>

#include <exp_msgs/ExpAction.h>
#include <actionlib/server/simple_action_server.h>
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
            QWidget * widget_;

        protected:
            /// Action Server
            std::shared_ptr<actionlib::SimpleActionServer<exp_msgs::ExpAction>> act_srv_;

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

            /// Current State
            serverState state_;

        private Q_SLOTS:
            // Qt callback function
            void buttonPreemptedCallback();
            void buttonAbortedCallback();
            void buttonSucceededCallback();
    };

} // namespace exp_server
