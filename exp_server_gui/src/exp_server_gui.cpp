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
    {
        setObjectName("exp_server_gui");
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
        act_srv_ = std::make_shared<actionlib::SimpleActionServer<exp_msgs::ExpAction>> (
            getNodeHandle(),
            "/exp_server_node",
            [this](const exp_msgs::ExpGoalConstPtr & goal)
            {
                this->actionServerCB(goal);
            },
            false
        );

        // Connect Qt Widget
        connect(ui_.buttonPreempted, SIGNAL(pressed()), this, SLOT(buttonPreemptedCallback()));
        connect(ui_.buttonAborted, SIGNAL(pressed()), this, SLOT(buttonAbortedCallback()));
        connect(ui_.buttonSucceeded, SIGNAL(pressed()), this, SLOT(buttonSucceededCallback()));

        // Start action server
        act_srv_->start();
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

    void exp_server_gui::actionServerCB(const exp_msgs::ExpGoalConstPtr & goal)
    {
        // Set status text
        ui_.current_status->setText("   ACTIVE");

        ROS_INFO_STREAM(
            ros::this_node::getName() << " " << __func__ <<
            " action server request has been received."
        );

        // Success flag
        bool isok = false;

        ros::Rate r(5);
        // Carry out goal request
        while(ros::ok())
        {
            if(!act_srv_->isPreemptRequested())
            {
                if(state_ != exp_server::serverState::IDLE)
                    break;
            }
            else
            {
                state_ = exp_server::serverState::CANCEL;
                ui_.current_status->setText("   CANCELLED BY CLIENT.");
                break;
            }
            r.sleep();
        }

        switch (state_)
        {
        case exp_server::serverState::PREEMPTED:
            act_srv_->setPreempted(result_.result, " action server state: PREEMPTED!");
            ROS_WARN_STREAM(
                ros::this_node::getName()
                << " "
                << __func__
                << " server responded with PREEMPTED."
            );
            break;

        case exp_server::serverState::ABORTED:
            act_srv_->setAborted(result_.result, " action server state: PREEMPTED!");
            ROS_ERROR_STREAM(
                ros::this_node::getName()
                << " "
                << __func__
                << " server responded with ABORTED."
            );
            break;

        case exp_server::serverState::SUCCEEDED:
            act_srv_->setSucceeded(result_.result, " action server state: SUCCEEDED!");
            ROS_INFO_STREAM(
                ros::this_node::getName()
                << " "
                << __func__
                << " server responded with SUCCEEDED."
            );
            break;

        case exp_server::serverState::CANCEL:
            act_srv_->setPreempted(result_.result, " action cancelled by client.");
            ROS_INFO_STREAM(
                ros::this_node::getName()
                << " "
                << __func__
                << " action cancelled by client."
            );
            break;

        default:
            act_srv_->setAborted(result_.result, " DEFAULT action server state: ABORTED!!!");
            ROS_FATAL_STREAM(
                ros::this_node::getName()
                << " "
                << __func__
                << " server responded with ABORTED."
            );
            break;
        }

        // Reset State
        state_ = exp_server::serverState::IDLE;

        // Set status text
        ui_.current_status->setText("     IDLE");
    }

    void exp_server_gui::stop()
    {
        act_srv_->shutdown();
        if (widget_ != nullptr)
            delete widget_;
    }

    void exp_server_gui::buttonPreemptedCallback()
    {
        if(state_ == exp_server::serverState::IDLE)
            state_ = exp_server::serverState::PREEMPTED;
    }

    void exp_server_gui::buttonAbortedCallback()
    {
        if(state_ == exp_server::serverState::IDLE)
            state_ = exp_server::serverState::ABORTED;
    }

    void exp_server_gui::buttonSucceededCallback()
    {
        if(state_ == exp_server::serverState::IDLE)
            state_ = exp_server::serverState::SUCCEEDED;
    }

} // namespace exp_server
