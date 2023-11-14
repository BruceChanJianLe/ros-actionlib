#include "exp_server_gui/exp_server_gui.hpp"
#include <pluginlib/class_list_macros.h>
#include <QStringList>


PLUGINLIB_EXPORT_CLASS(exp_server::exp_server_gui, rqt_gui_cpp::Plugin);

namespace exp_server
{
    exp_server_gui::exp_server_gui()
    :   rqt_gui_cpp::Plugin()
    ,   widget_(0)
    ,   state_(exp_server::serverState::IDLE)
    ,   alternate_goals_{false}
    {
        setObjectName("mbf_client_gui");
    }

    exp_server_gui::~exp_server_gui()
    {
        stop();
    }

    void exp_server_gui::initPlugin(qt_gui_cpp::PluginContext & context)
    {
        // Access standalone command line arguments
        QStringList argv = context.argv();

        // Create QWidget
        widget_ = new QWidget();

        // Extend the widget with all attributes and children from UI
        ui_.setupUi(widget_);

        // Add widget to the user interface
        context.addWidget(widget_);

        // Initialize ROS related things
        // Initialize action server
        act_client_ = std::make_shared<actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>>(
            getNodeHandle(),
            "/move_base_flex/move_base",
            true);

        // Connect Qt Widget
        connect(ui_.buttonSendGoal, SIGNAL(pressed()), this, SLOT(buttonSendGoal()));
        connect(ui_.buttonCancelGoal, SIGNAL(pressed()), this, SLOT(buttonCancelGoal()));
        connect(ui_.buttonCancelAllGoal, SIGNAL(pressed()), this, SLOT(buttonCancelAllGoal()));
        connect(ui_.buttonStopTrackingGoal, SIGNAL(pressed()), this, SLOT(buttonStopTrackingGoal()));

        // Start action server
        ROS_INFO("Waiting for mbf action server...");
        act_client_->waitForServer();
        ROS_INFO("mbf action server has started");
    }

    void exp_server_gui::shutdownPlugin()
    {
    }

    void exp_server_gui::saveSettings(
        qt_gui_cpp::Settings & plugin_settings,
        qt_gui_cpp::Settings & instance_settings
    ) const
    {
    }

    void exp_server_gui::restoreSettings(
        const qt_gui_cpp::Settings & plugin_settings,
        const qt_gui_cpp::Settings & instance_settings
    )
    {
    }

    void exp_server_gui::stop()
    {
        if (widget_ != nullptr)
            delete widget_;
    }

    void exp_server_gui::buttonSendGoal()
    {
        mbf_msgs::MoveBaseGoal goal;
        goal.controller = "modethree";
        goal.planner = "modeone";
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        if (alternate_goals_)
        {
            goal.target_pose.pose.position.x = 5;
            alternate_goals_ = !alternate_goals_;
        }
        else
        {
            goal.target_pose.pose.position.x = 0;
            alternate_goals_ = !alternate_goals_;
        }
        goal.target_pose.pose.position.y = 0;
        goal.target_pose.pose.position.z = 0;
        goal.target_pose.pose.orientation.x = 0;
        goal.target_pose.pose.orientation.y = 0;
        goal.target_pose.pose.orientation.z = 0;
        goal.target_pose.pose.orientation.w = 1;

        {
            actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> new_act_client(
            getNodeHandle(),
            "/move_base_flex/move_base",
            true);
            new_act_client.sendGoal(goal);
        }
        // act_client_->sendGoal(goal);

        ROS_INFO_STREAM(
            ros::this_node::getName()
            << " "
            << __func__
            << " client has send goal with x: "
            << goal.target_pose.pose.position.x
            << "."
        );
    }

    void exp_server_gui::buttonCancelGoal()
    {
        act_client_->cancelGoal();
        ROS_INFO_STREAM(
            ros::this_node::getName()
            << " "
            << __func__
            << " client has cancelled the goal."
        );
    }

    void exp_server_gui::buttonCancelAllGoal()
    {
        act_client_->cancelAllGoals();
        ROS_INFO_STREAM(
            ros::this_node::getName()
            << " "
            << __func__
            << " client has cancelled all the goals."
        );
    }

    void exp_server_gui::buttonStopTrackingGoal()
    {
        act_client_->stopTrackingGoal();
        ROS_INFO_STREAM(
            ros::this_node::getName()
            << " "
            << __func__
            << " client has stopped tracking the goal."
        );
    }

} // namespace exp_server
