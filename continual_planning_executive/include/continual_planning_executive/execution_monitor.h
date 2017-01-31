/*
 * execution_monitor.h
 *
 *  Created on: Jan 25, 2017
 *      Author: andreas
 */

#ifndef SRC_EXECUTION_MONITOR_H_
#define SRC_EXECUTION_MONITOR_H_

#include <continual_planning_executive/plan.h>
#include <continual_planning_msgs/ExecutedActivity.h>

struct ExecutedActivityComparator
{
  bool operator()(const continual_planning_msgs::ExecutedActivity& a, const continual_planning_msgs::ExecutedActivity& b);
};

typedef std::set<continual_planning_msgs::ExecutedActivity, ExecutedActivityComparator> ActivitySet;
class ExecutionMonitor
{
public:
  ExecutionMonitor();
  virtual ~ExecutionMonitor();

  continual_planning_msgs::ExecutedActivity createActivity(const DurativeAction& a) const;
  void startActivity(const DurativeAction& action);
  void finishActivity(const DurativeAction& action, bool success, const std::string& explanation);
  void startPlanning();
  void finishPlanning(bool success, const std::string& explanation);

private:
  ActivitySet active_actions_;
  std::vector<continual_planning_msgs::ExecutedActivity> finished_actions_;
  DurativeAction planning_activity_;
};

#endif /* SRC_EXECUTION_MONITOR_H_ */
