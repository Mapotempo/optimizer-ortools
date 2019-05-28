#ifndef OR_TOOLS_TUTORIALS_CPLUSPLUS_LIMITS_H
#define OR_TOOLS_TUTORIALS_CPLUSPLUS_LIMITS_H

#include <chrono>
#include <iomanip>
#include <ostream>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include "./ortools_result.pb.h"

#include "./tsptw_data_dt.h"

#include "ortools/base/bitmap.h"
#include "ortools/base/file.h"
#include "ortools/base/logging.h"

#include "ortools/constraint_solver/constraint_solver.h"

DEFINE_int64(time_limit_in_ms, 0, "Time limit in ms, no option means no limit.");
DEFINE_int64(no_solution_improvement_limit, -1, "Iterations whitout improvement");
DEFINE_int64(minimum_duration, -1, "Initial time whitout improvement in ms");
DEFINE_int64(init_duration, -1, "Maximum duration to find a first solution");
DEFINE_int64(time_out_multiplier, 2, "Multiplier for the nexts time out");
DEFINE_int64(vehicle_limit, 0, "Define the maximum number of vehicle");
DEFINE_int64(solver_parameter, -1, "Force a particular behavior");
DEFINE_bool(only_first_solution, false, "Compute only the first solution");
DEFINE_bool(balance, false, "Route balancing");
DEFINE_bool(nearby, false, "Short segment priority");
#ifdef DEBUG
DEFINE_bool(debug, true, "debug display");
#else
DEFINE_bool(debug, false, "debug display");
#endif
DEFINE_bool(intermediate_solutions, false, "display intermediate solutions");
DEFINE_string(routing_search_parameters,
              "", /* An example of how we can override the default settings */
              // "first_solution_strategy:ALL_UNPERFORMED"
              // "local_search_operators {"
              // "  use_path_lns:BOOL_TRUE"
              // "  use_inactive_lns:BOOL_TRUE"
              // "}",
              "Text proto RoutingSearchParameters (possibly partial) that will "
              "override the DefaultRoutingSearchParameters()");

const char* kTime     = "time";
const char* kDistance = "distance";

namespace operations_research {
namespace {

//  Don't use this class within a MakeLimit factory method!
class NoImprovementLimit : public SearchLimit {
public:
  NoImprovementLimit(Solver* const solver, IntVar* const objective_var,
                     int64 solution_nbr_tolerance, double time_out, int64 time_out_coef,
                     int64 init_duration, const bool minimize = true)
      : SearchLimit(solver)
      , solver_(solver)
      , start_time_(absl::GetCurrentTimeNanos())
      , solution_nbr_tolerance_(solution_nbr_tolerance)
      , minimize_(minimize)
      , limit_reached_(false)
      , first_solution_(true)
      , initial_time_out_(time_out)
      , time_out_(time_out)
      , time_out_coef_(time_out_coef)
      , init_duration_(init_duration)
      , nbr_solutions_with_no_better_obj_(0)
      , prototype_(new Assignment(solver_)) {
    if (minimize_) {
      best_result_ = kint64max;
    } else {
      best_result_ = kint64min;
    }

    CHECK_NOTNULL(objective_var);
    prototype_->AddObjective(objective_var);
  }

  virtual void Init() {
    nbr_solutions_with_no_better_obj_ = 0;
    limit_reached_                    = false;
    if (minimize_) {
      best_result_ = kint64max;
    } else {
      best_result_ = kint64min;
    }
  }

  //  Returns true if limit is reached, false otherwise.
  virtual bool Check() {
    if (!first_solution_ &&
        ((nbr_solutions_with_no_better_obj_ > solution_nbr_tolerance_ &&
          solution_nbr_tolerance_ > 0) ||
         (1e-6 * (absl::GetCurrentTimeNanos() - start_time_) > time_out_ &&
          initial_time_out_ > 0))) {
      limit_reached_ = true;
    } else if (first_solution_ && init_duration_ > 0 &&
               1e-6 * (absl::GetCurrentTimeNanos() - start_time_) > init_duration_) {
      limit_reached_ = true;
    }
    // VLOG(2) << "NoImprovementLimit's limit reached? " << limit_reached_;
    return limit_reached_;
  }

  virtual bool AtSolution() {
    prototype_->Store();

    const IntVar* objective = prototype_->Objective();
    if (minimize_ && objective->Min() * 1.01 < best_result_) {
      if (first_solution_) {
        first_solution_ = false;
      }
      best_result_                      = objective->Min();
      nbr_solutions_with_no_better_obj_ = 0;
      if (initial_time_out_ > 0)
        time_out_ =
            std::max(initial_time_out_,
                     time_out_coef_ * 1e-6 * (absl::GetCurrentTimeNanos() - start_time_));
    } else if (!minimize_) {
      std::cout << "Maximization is not implemented!" << std::endl;
    }

    ++nbr_solutions_with_no_better_obj_;

    return true;
  }

  virtual void Copy(const SearchLimit* const limit) {
    const NoImprovementLimit* const copy_limit =
        reinterpret_cast<const NoImprovementLimit* const>(limit);

    best_result_                      = copy_limit->best_result_;
    solution_nbr_tolerance_           = copy_limit->solution_nbr_tolerance_;
    minimize_                         = copy_limit->minimize_;
    initial_time_out_                 = copy_limit->initial_time_out_;
    time_out_                         = copy_limit->time_out_;
    limit_reached_                    = copy_limit->limit_reached_;
    first_solution_                   = copy_limit->first_solution_;
    time_out_coef_                    = copy_limit->time_out_coef_;
    init_duration_                    = copy_limit->init_duration_;
    start_time_                       = copy_limit->start_time_;
    nbr_solutions_with_no_better_obj_ = copy_limit->nbr_solutions_with_no_better_obj_;
  }

  // Allocates a clone of the limit
  virtual SearchLimit* MakeClone() const {
    // we don't to copy the variables
    return solver_->RevAlloc(new NoImprovementLimit(solver_, prototype_->Objective(),
                                                    solution_nbr_tolerance_, time_out_,
                                                    time_out_coef_, minimize_));
  }

  virtual std::string DebugString() const {
    return absl::StrFormat("NoImprovementLimit(crossed = %i)", limit_reached_);
  }

private:
  Solver* const solver_;
  int64 best_result_;
  double start_time_;
  int64 solution_nbr_tolerance_;
  bool minimize_;
  bool limit_reached_;
  bool first_solution_;
  double initial_time_out_;
  double time_out_;
  int64 time_out_coef_;
  int64 init_duration_;
  int64 nbr_solutions_with_no_better_obj_;
  std::unique_ptr<Assignment> prototype_;
};

} // namespace

NoImprovementLimit*
MakeNoImprovementLimit(Solver* const solver, IntVar* const objective_var,
                       const int64 solution_nbr_tolerance, const double time_out,
                       const int64 time_out_coef, const int64 init_duration,
                       const bool minimize = true) {
  return solver->RevAlloc(new NoImprovementLimit(solver, objective_var,
                                                 solution_nbr_tolerance, time_out,
                                                 time_out_coef, init_duration, minimize));
}

namespace {

//  Don't use this class within a MakeLimit factory method!
class LoggerMonitor : public SearchLimit {
public:
  LoggerMonitor(const TSPTWDataDT& data, RoutingModel* routing,
                RoutingIndexManager* manager, int64 min_start, int64 size_matrix,
                bool debug, bool intermediate, ortools_result::Result* result,
                std::string filename, const bool minimize = true)
      : SearchLimit(routing->solver())
      , data_(data)
      , routing_(routing)
      , manager_(manager)
      , solver_(routing->solver())
      , start_time_(absl::GetCurrentTimeNanos())
      , min_start_(min_start)
      , size_matrix_(size_matrix)
      , minimize_(minimize)
      , limit_reached_(false)
      , debug_(debug)
      , intermediate_(intermediate)
      , pow_(0)
      , iteration_counter_(0)
      , prototype_(new Assignment(solver_))
      , filename_(filename)
      , result_(result) {
    if (minimize_) {
      best_result_ = kint64max;
    } else {
      best_result_ = kint64min;
    }

    CHECK_NOTNULL(routing->CostVar());
    prototype_->AddObjective(routing->CostVar());
  }

  virtual void Init() {
    iteration_counter_ = 0;
    pow_               = 0;
    if (minimize_) {
      best_result_ = kint64max;
    } else {
      best_result_ = kint64min;
    }
  }

  //  Returns true if limit is reached, false otherwise.
  virtual bool Check() {
    // VLOG(2) << "NoImprovementLimit's limit reached? " << limit_reached_;

    return false;
  }

  inline double GetSpanCostForVehicleForDimension(int vehicle,
                                                  const std::string& dimension_name) {
    if (routing_->GetMutableDimension(dimension_name) == nullptr)
      return 0;
    return (routing_->GetMutableDimension(dimension_name)
                ->CumulVar(routing_->End(vehicle))
                ->Min() -
            routing_->GetMutableDimension(dimension_name)
                ->CumulVar(routing_->Start(vehicle))
                ->Max()) *
           routing_->GetMutableDimension(dimension_name)
               ->GetSpanCostCoefficientForVehicle(vehicle) /
           1000000.0;
  }

  virtual bool AtSolution() {
    prototype_->Store();
    bool new_best = false;

    const IntVar* objective = prototype_->Objective();
    if (minimize_ && objective->Min() * 1.01 < best_result_) {
      best_result_ = objective->Min();
      if (intermediate_) {
        if (result_->routes_size() > 0)
          result_->clear_routes();

        // int current_break = 0;

        double total_fake_time_cost(0.0), total_fake_distance_cost(0.0),
            total_time_cost(0.0), total_distance_cost(0.0), total_time_balance_cost(0.0),
            total_distance_balance_cost(0.0), total_time_without_wait_cost(0.0),
            total_value_cost(0.0), total_vehicle_fixed_cost(0.0),
            total_time_order_cost(0.0), total_distance_order_cost(0.0);

        int nbr_routes(0), nbr_services_served(0);

        for (int route_nbr = 0; route_nbr < routing_->vehicles(); route_nbr++) {
          // int route_break = 0;
          ortools_result::Route* route = result_->add_routes();
          int previous_index           = -1;
          bool vehicle_used            = false;
          for (int64 index = routing_->Start(route_nbr); !routing_->IsEnd(index);
               index       = routing_->NextVar(index)->Value()) {
            ortools_result::Activity* activity       = route->add_activities();
            RoutingIndexManager::NodeIndex nodeIndex = manager_->IndexToNode(index);
            activity->set_index(data_.ProblemIndex(nodeIndex));
            activity->set_start_time(
                routing_->GetMutableDimension(kTime)->CumulVar(index)->Min());
            activity->set_current_distance(
                routing_->GetMutableDimension(kDistance)->CumulVar(index)->Min());
            if (previous_index == -1)
              activity->set_type("start");
            else {
              if (index >= data_.SizeMissions()) {
                activity->set_type("break");
                activity->set_index(int64(nodeIndex.value() - data_.SizeMissions()));
              } else {
                ++nbr_services_served;
                vehicle_used = true;
                activity->set_type("service");
                activity->set_index(data_.ProblemIndex(nodeIndex));
                activity->set_alternative(data_.AlternativeIndex(nodeIndex));
              }
            }
            for (std::size_t q = 0;
                 q < data_.Quantities(RoutingIndexManager::NodeIndex(0)).size(); ++q) {
              double exchange =
                  routing_->GetMutableDimension("quantity" + std::to_string(q))
                      ->CumulVar(index)
                      ->Min();
              activity->add_quantities(exchange);
            }
            previous_index = index;
          }
          ortools_result::Activity* end_activity = route->add_activities();
          RoutingIndexManager::NodeIndex nodeIndex =
              manager_->IndexToNode(routing_->End(route_nbr));
          end_activity->set_index(data_.ProblemIndex(nodeIndex));
          end_activity->set_start_time(routing_->GetMutableDimension(kTime)
                                           ->CumulVar(routing_->End(route_nbr))
                                           ->Min());
          end_activity->set_current_distance(routing_->GetMutableDimension(kDistance)
                                                 ->CumulVar(routing_->End(route_nbr))
                                                 ->Min());
          end_activity->set_type("end");

          if (FLAGS_nearby) {
            total_time_order_cost +=
                GetSpanCostForVehicleForDimension(route_nbr, "time_order");
            total_distance_order_cost +=
                GetSpanCostForVehicleForDimension(route_nbr, "distance_order");
          }

          if (FLAGS_debug) {
            if (vehicle_used) {
              ++nbr_routes;
              total_vehicle_fixed_cost +=
                  routing_->GetFixedCostOfVehicle(route_nbr) / 1000000.0;
            }
            total_time_cost += GetSpanCostForVehicleForDimension(route_nbr, "time");
            total_distance_cost +=
                GetSpanCostForVehicleForDimension(route_nbr, "distance");
            total_time_balance_cost +=
                GetSpanCostForVehicleForDimension(route_nbr, "time_balance");
            total_distance_balance_cost +=
                GetSpanCostForVehicleForDimension(route_nbr, "distance_balance");
            total_fake_time_cost +=
                GetSpanCostForVehicleForDimension(route_nbr, "fake_time");
            total_fake_distance_cost +=
                GetSpanCostForVehicleForDimension(route_nbr, "fake_distance");
            total_time_without_wait_cost +=
                GetSpanCostForVehicleForDimension(route_nbr, "time_without_wait");
            total_value_cost += GetSpanCostForVehicleForDimension(route_nbr, "value");
          }
        }

        result_->set_cost(best_result_ / 1000000.0 -
                          (total_time_order_cost + total_distance_order_cost));
        result_->set_duration(1e-9 * (absl::GetCurrentTimeNanos() - start_time_));
        result_->set_iterations(iteration_counter_);

        std::fstream output(filename_,
                            std::ios::out | std::ios::trunc | std::ios::binary);
        if (!result_->SerializeToOstream(&output)) {
          std::cout << "Failed to write result." << std::endl;
          return false;
        }
        output.close();

        std::cout << "Iteration : " << result_->iterations()
                  << " Cost : " << result_->cost() << " Time : " << result_->duration()
                  << std::endl;

        if (FLAGS_debug) {
          std::cout.precision(15);
          std::cout << "Cost breakdown:"
                    << "\n nbr_services_served: " << nbr_services_served
                    << "\n nbr_routes: " << nbr_routes
                    << "\n total_vehicle_fixed_cost:     " << total_vehicle_fixed_cost
                    << "\n total_time_cost:              " << total_time_cost
                    << "\n total_distance_cost:          " << total_distance_cost
                    << "\n total_time_balance_cost:      " << total_time_balance_cost
                    << "\n total_distance_balance_cost:  " << total_distance_balance_cost
                    << "\n total_fake_time_cost:         " << total_fake_time_cost
                    << "\n total_fake_distance_cost:     " << total_fake_distance_cost
                    << "\n total_time_without_wait_cost: " << total_time_without_wait_cost
                    << "\n total_value_cost:             " << total_value_cost
                    << "\n Cost substracted from the results but used in optimization "
                       "(due to nearby flag):"
                    << "\n total_time_order_cost:        " << total_time_order_cost
                    << "\n total_distance_order_cost:    " << total_distance_order_cost
                    << std::endl;
        }
      }
      new_best = true;
    } else if (!minimize_) {
      std::cout << "Maximization is not implemented!" << std::endl;
    }

    if (debug_ && new_best) {
      std::cout << "min start : " << min_start_ << std::endl;
      for (RoutingIndexManager::NodeIndex i(0); i < data_.SizeMatrix() - 1; ++i) {
        int64 index         = manager_->NodeToIndex(i);
        IntVar* cumul_var   = routing_->GetMutableDimension(kTime)->CumulVar(index);
        IntVar* transit_var = routing_->GetMutableDimension(kTime)->TransitVar(index);
        IntVar* slack_var   = routing_->GetMutableDimension(kTime)->SlackVar(index);
        IntVar* const vehicle_var = routing_->VehicleVar(index);
        if (vehicle_var->Bound() && cumul_var->Bound() && transit_var->Bound() &&
            slack_var->Bound()) {
          std::cout << "Node " << i << " index " << index << " [" << vehicle_var->Value()
                    << "] |";
          std::cout << (cumul_var->Value() - min_start_) << " + " << transit_var->Value()
                    << " -> " << slack_var->Value() << std::endl;
        }
      }
      std::cout << "-----------" << std::endl;
    }

    ++iteration_counter_;
    if (iteration_counter_ >= std::pow(2, pow_)) {
      std::cout << "Iteration : " << iteration_counter_ << std::endl;
      ++pow_;
    }
    return true;
  }

  virtual void Copy(const SearchLimit* const limit) {
    const LoggerMonitor* const copy_limit =
        reinterpret_cast<const LoggerMonitor* const>(limit);

    best_result_       = copy_limit->best_result_;
    iteration_counter_ = copy_limit->iteration_counter_;
    start_time_        = copy_limit->start_time_;
    size_matrix_       = copy_limit->size_matrix_;
    result_            = copy_limit->result_;
    minimize_          = copy_limit->minimize_;
    limit_reached_     = copy_limit->limit_reached_;
  }

  // Allocates a clone of the limit
  virtual SearchLimit* MakeClone() const {
    // we don't to copy the variables
    return solver_->RevAlloc(new LoggerMonitor(data_, routing_, manager_, min_start_,
                                               size_matrix_, debug_, intermediate_,
                                               result_, filename_, minimize_));
  }

  virtual std::string DebugString() const {
    return absl::StrFormat("LoggerMonitor(crossed = %i)", limit_reached_);
  }

  std::vector<double> GetFinalScore() {
    return {(best_result_ / 1000000.0),
            1e-9 * (absl::GetCurrentTimeNanos() - start_time_),
            (double)iteration_counter_};
  }

  // void GetFinalLog() {
  //   std::cout << "Final Iteration : " << iteration_counter_ << " Cost : " <<
  //   best_result_ / 1000000.0 << " Time : " << 1e-9 * (base::GetCurrentTimeNanos() -
  //   start_time_) << std::endl;
  // }

private:
  const TSPTWDataDT& data_;
  RoutingModel* routing_;
  RoutingIndexManager* manager_;
  Solver* const solver_;
  int64 best_result_;
  double start_time_;
  int64 min_start_;
  int64 size_matrix_;
  bool minimize_;
  bool limit_reached_;
  bool debug_;
  bool intermediate_;
  int64 pow_;
  int64 iteration_counter_;
  std::unique_ptr<Assignment> prototype_;
  std::string filename_;
  ortools_result::Result* result_;
};

} // namespace

LoggerMonitor* MakeLoggerMonitor(const TSPTWDataDT& data, RoutingModel* routing,
                                 RoutingIndexManager* manager, int64 min_start,
                                 int64 size_matrix, bool debug, bool intermediate,
                                 ortools_result::Result* result, std::string filename,
                                 const bool minimize = true) {
  return routing->solver()->RevAlloc(new LoggerMonitor(data, routing, manager, min_start,
                                                       size_matrix, debug, intermediate,
                                                       result, filename, minimize));
}
} //  namespace operations_research

#endif //  OR_TOOLS_TUTORIALS_CPLUSPLUS_LIMITS_H
