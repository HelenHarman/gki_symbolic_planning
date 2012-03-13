#ifndef PLANNER_PARAMETERS_H
#define PLANNER_PARAMETERS_H

#include <string>
using namespace std;
#include "best_first_search.h"

class PlannerParameters
{
   public:
      PlannerParameters();
      ~PlannerParameters();
 
      /// Read parameters from ROS and cmd line (cmd line overrides ROS parameters).
      bool readParameters(int argc, char** argv);

      void dump() const;

   public:
      bool anytime_search;          ///< Perform anytime search (don't stop at first plan)
      bool disallow_concurrent_actions;     ///< Do NOT allow any concurrent actions, i.e. do NOT plan temporally
      
      int timeout_if_plan_found;          ///< Timeout if a plan was found (0 - inf).
      int timeout_while_no_plan_found;    ///< Timeout while no plan found (0 - inf).
      
      bool greedy;                  ///< Perform greedy search
      bool lazy_evaluation;         ///< Lazy heuristic evaluation
      bool verbose;                 ///< Verbose outputs
      bool insert_let_time_pass_only_when_running_operators_not_empty;

      int lazy_state_module_evaluation;    ///< if > 0 evaluate modules lazy, if < 0 determine automatically

      /// if true, is_applicable checks for valid duration from cost modules
      /** if false the cost module for an applicable operator should always provide a valid cost
       *  for applicable operators (possibly in conjunction with an appropriate (pre-)condition module)
       */
      bool use_cost_modules_for_applicability;

      bool cyclic_cg_heuristic;                    ///< Use cyclic_cg heuristic
      bool cyclic_cg_preferred_operators;          ///< Use cyclic_cg heuristic preferred operators
      bool makespan_heuristic;                     ///< Use makespan heuristic
      bool makespan_heuristic_preferred_operators; ///< Use makespan heuristic preferred operators
      bool no_heuristic;                           ///< Use the no heuristic

      bool cg_heuristic_zero_cost_waiting_transitions;  ///< If false, scheduled effects are accounted
      bool cg_heuristic_fire_waiting_transitions_only_if_local_problems_matches_state;
 
      /// Possible definitions of "g"
      enum GValues {
         GMakespan,               ///< g values by makespan (timestamp + longest duration of running operators
         GCost,                   ///< g values by path cost
         GTimestamp,              ///< g values by timestamp
         GWeighted                ///< g values weighted by  w * makespan + (1-w) * pathcost
      };
      enum GValues g_values;      ///< How g values are calculated - Default: Timestamp
      double g_weight;            ///< The weight w for GWeighted

      BestFirstSearchEngine::QueueManagementMode queueManagementMode;

      bool use_known_by_logical_state_only;         ///< Enable tss known filtering (might crop search space!)

      /** when enabled: if two plans have the same makespan, consider one better 
          if it has lower number of subgoals, otherwise same */
      bool use_subgoals_to_break_makespan_ties;     
 
      bool reschedule_plans;        ///< Use scheduler to reschedule found plans
      bool epsilonize_internally;   ///< add eps_time when applying an operator
      bool epsilonize_externally;   ///< Add epsilon steps in between plan steps by calling epsilonize_plan.
      bool keep_original_plans;     ///< Store non-epsilonized plans as "...orig" files (if epsilonize_externally on)

      string plan_name;             ///< File prefix for outputting plans
      string planMonitorFileName;   ///< Filename for monitoring (if set, implies monitoring mode)

      bool monitoring_verify_timestamps;     ///< During monitoring only accept the monitored plan if the timestamps match the original one.
   
   protected:
      /// Read parameters from param server (ros must be initialized).
      bool readROSParameters();
      /// Read parameters from command line.
      bool readCmdLineParameters(int argc, char** argv);

      void printUsage() const;
};

#endif

