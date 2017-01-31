#include <stdio.h>
#include <ros/ros.h>
#include <vector>
#include <deque>
#include <string>
#include <sstream>
#include <signal.h>

#include "continual_planning_executive/symbolicState.h"
#include "continual_planning_executive/stateCreator.h"
#include "continual_planning_executive/goalCreator.h"
#include "continual_planning_executive/plannerInterface.h"
#include "continual_planning_executive/load_plugins.h"
#include "continual_planning_executive/planExecutor.h"
#include "continual_planning_executive/continualPlanning.h"

#include <continual_planning_msgs/SetContinualPlanningControl.h>
#include <continual_planning_msgs/PlanningAction.h>

#include <actionlib/server/simple_action_server.h>

#include <ros/package.h>

using std::string;

class ContinualPlanningServer
{
public:
  ContinualPlanningServer(const string& name) :
      as_(nh_, name, false), action_name_(name)
  {
    planning_.reset(new ContinualPlanning());
    loop_rate_ = ros::Rate(10);
    as_.start();
  }

  void preemptCB()
  {
    // TODO cancel current actions or planning
  }

  void executeCB(const continual_planning_msgs::PlanningGoalConstPtr &goal)
  {
    bool success = true;
    planning_->reset();

    // TODO set goal

    // start executing the action
    while (ros::ok())
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested())
      {
        ROS_INFO_STREAM(action_name_<<": Preempted");
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      planning_state_ = planning_->loop();
      if(planning_state_ != ContinualPlanning::Running) {
          break;
      }

      loop_rate_.sleep();
      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
    }

    if (success)
    {
//      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }

protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<continual_planning_msgs::PlanningAction> as_;
  string action_name_;
  continual_planning_msgs::PlanningFeedback feedback_;
  continual_planning_msgs::PlanningResult result_;

  ContinualPlanning::Ptr planning_;
  continual_planning_msgs::ContinualPlanningStatus planning_state_;
  ros::Rate loop_rate_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "continual_planning_server");

  ContinualPlanningServer planning_server(ros::this_node::getName());
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();

  return 0;
}

