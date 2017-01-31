/*
 * execution_monitor.cpp
 *
 *  Created on: Jan 25, 2017
 *      Author: andreas
 */

#include <continual_planning_executive/execution_monitor.h>
#include <ros/console.h>

bool ExecutedActivityComparator::operator()(const continual_planning_msgs::ExecutedActivity& a,
    const continual_planning_msgs::ExecutedActivity& b)
{
  if (a.action.name < b.action.name)
    return true;
  else if (a.action.name > b.action.name)
    return false;

  if (a.action.duration < b.action.duration)
    return true;
  else if (a.action.duration > b.action.duration)
    return false;

  if (a.action.start_time < b.action.start_time)
    return true;
  else if (a.action.start_time > b.action.start_time)
    return false;

  if (a.action.parameters.size() < b.action.parameters.size())
    return true;
  else if (a.action.parameters.size() > b.action.parameters.size())
    return false;

  // parameters.size == b.action.parameters.size
  for (unsigned int i = 0; i < a.action.parameters.size(); i++)
  {
    if (a.action.parameters[i] < b.action.parameters[i])
      return true;
    else if (a.action.parameters[i] > b.action.parameters[i])
      return false;
  }
  return false;
}

ExecutionMonitor::ExecutionMonitor()
{
  planning_activity_.name = "planning";
}

ExecutionMonitor::~ExecutionMonitor()
{
}

continual_planning_msgs::ExecutedActivity ExecutionMonitor::createActivity(const DurativeAction& action) const
{
  continual_planning_msgs::ExecutedActivity a;
  a.action.name = action.name;
  a.action.parameters = action.parameters;
  a.action.start_time = action.startTime;
  a.action.duration = action.duration;
  return a;
}

void ExecutionMonitor::startActivity(const DurativeAction& action)
{
  active_actions_.insert(createActivity(action));
}

void ExecutionMonitor::finishActivity(const DurativeAction& action, bool success, const std::string& explanation)
{
  ActivitySet::const_iterator it = active_actions_.find(createActivity(action));
  if (it == active_actions_.end())
  {
    ROS_WARN_STREAM("expected activity "<<action<<" has no tracking entry. skipping...");
    return;
  }

  finished_actions_.push_back(*it);
  continual_planning_msgs::ExecutedActivity& activity = finished_actions_.back();
  activity.duration = ros::Time::now() - activity.start_time;
  activity.success = success;
  activity.explanation = explanation;

  active_actions_.erase(it);
}

void ExecutionMonitor::startPlanning()
{
  startActivity(planning_activity_);
}

void ExecutionMonitor::finishPlanning(bool success, const std::string& explanation)
{
  finishActivity(planning_activity_, success, explanation);
}
