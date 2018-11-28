#ifndef OR_TOOLS_TUTORIALS_CPLUSPLUS_LIMITS_H
#define OR_TOOLS_TUTORIALS_CPLUSPLUS_LIMITS_H

#include <ostream>
#include <chrono>
#include <iomanip>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <vector>

#include "ortools/base/bitmap.h"
#include "ortools/base/logging.h"
#include "ortools/base/file.h"
#include "ortools/base/split.h"
#include "ortools/base/filelinereader.h"
#include "ortools/base/join.h"
#include "ortools/base/strtoint.h"

#include "ortools/constraint_solver/constraint_solver.h"

#include "ortools_result.pb.h"

namespace operations_research {
namespace {

//  Don't use this class within a MakeLimit factory method!
class NoImprovementLimit : public SearchLimit {
  public:
    NoImprovementLimit(Solver * const solver, IntVar * const objective_var, int64 solution_nbr_tolerance, double time_out, int64 time_out_coef, const bool minimize = true) :
    SearchLimit(solver),
      solver_(solver), prototype_(new Assignment(solver_)),
      solution_nbr_tolerance_(solution_nbr_tolerance),
      start_time_(base::GetCurrentTimeNanos()),
      nbr_solutions_with_no_better_obj_(0),
      minimize_(minimize),
      initial_time_out_(time_out),
      time_out_(time_out),
      time_out_coef_(time_out_coef),
      first_solution_(true),
      limit_reached_(false) {
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
    limit_reached_ = false;
    if (minimize_) {
      best_result_ = kint64max;
    } else {
      best_result_ = kint64min;
    }
  }

  //  Returns true if limit is reached, false otherwise.
  virtual bool Check() {
    if (!first_solution_ && (nbr_solutions_with_no_better_obj_ > solution_nbr_tolerance_ && solution_nbr_tolerance_ > 0 || 1e-6 * (base::GetCurrentTimeNanos() - start_time_) > time_out_ && initial_time_out_ > 0)) {
      limit_reached_ = true;
    }
    //VLOG(2) << "NoImprovementLimit's limit reached? " << limit_reached_;

    return limit_reached_;
  }

  virtual bool AtSolution() {

    prototype_->Store();

    const IntVar* objective = prototype_->Objective();
    if (minimize_ && objective->Min() * 1.01 < best_result_) {
      if (first_solution_) {
        first_solution_ = false;
      }
      best_result_ = objective->Min();
      nbr_solutions_with_no_better_obj_ = 0;
      if (initial_time_out_ > 0) time_out_ = std::max(initial_time_out_ - 1e-6 * (base::GetCurrentTimeNanos() - start_time_), time_out_coef_ * 1e-6 * (base::GetCurrentTimeNanos() - start_time_));
    } else if (!minimize_ && objective->Max() * 0.99 > best_result_) {
      if (first_solution_) {
        first_solution_ = false;
      }
      best_result_ = objective->Max();
      nbr_solutions_with_no_better_obj_ = 0;
      if (initial_time_out_ > 0) time_out_ = std::max(initial_time_out_ - 1e-6 * (base::GetCurrentTimeNanos() - start_time_), time_out_coef_ * 1e-6 * (base::GetCurrentTimeNanos() - start_time_));
    }

    ++nbr_solutions_with_no_better_obj_;

    return true;
  }

  virtual void Copy(const SearchLimit* const limit) {
    const NoImprovementLimit* const copy_limit =
    reinterpret_cast<const NoImprovementLimit* const>(limit);

    best_result_ = copy_limit->best_result_;
    solution_nbr_tolerance_ = copy_limit->solution_nbr_tolerance_;
    minimize_ = copy_limit->minimize_;
    initial_time_out_= copy_limit->initial_time_out_;
    time_out_ = copy_limit->time_out_;
    limit_reached_ = copy_limit->limit_reached_;
    first_solution_ = copy_limit->first_solution_;
    time_out_coef_ = copy_limit->time_out_coef_;
    start_time_ = copy_limit->start_time_;
    nbr_solutions_with_no_better_obj_ = copy_limit->nbr_solutions_with_no_better_obj_;
  }

  // Allocates a clone of the limit
  virtual SearchLimit* MakeClone() const {
    // we don't to copy the variables
    return solver_->RevAlloc(new NoImprovementLimit(solver_, prototype_->Objective(), solution_nbr_tolerance_, time_out_, time_out_coef_, minimize_));
  }

  virtual std::string DebugString() const {
    return StringPrintf("NoImprovementLimit(crossed = %i)", limit_reached_);
  }

  private:
    Solver * const solver_;
    int64 best_result_;
    double start_time_;
    int64 solution_nbr_tolerance_;
    bool minimize_;
    bool limit_reached_;
    bool first_solution_;
    double initial_time_out_;
    double time_out_;
    int64 time_out_coef_;
    int64 nbr_solutions_with_no_better_obj_;
    std::unique_ptr<Assignment> prototype_;
};

} // namespace


NoImprovementLimit * MakeNoImprovementLimit(Solver * const solver, IntVar * const objective_var, const int64 solution_nbr_tolerance, const double time_out, const int64 time_out_coef, const bool minimize = true) {
  return solver->RevAlloc(new NoImprovementLimit(solver, objective_var, solution_nbr_tolerance, time_out, time_out_coef, minimize));
}

namespace {

//  Don't use this class within a MakeLimit factory method!
class LoggerMonitor : public SearchLimit {
  public:
    LoggerMonitor(const TSPTWDataDT &data, RoutingModel * routing, int64 min_start, int64 size_matrix, bool debug, bool intermediate, ortools_result::Result* result, std::string filename, const bool minimize = true) :
    data_(data),
    routing_(routing),
    SearchLimit(routing->solver()),
    solver_(routing->solver()), prototype_(new Assignment(solver_)),
    iteration_counter_(0),
    start_time_(base::GetCurrentTimeNanos()),
    pow_(0),
    min_start_(min_start),
    size_matrix_(size_matrix),
    debug_(debug),
    intermediate_(intermediate),
    result_(result),
    filename_(filename),
    minimize_(minimize) {
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
    pow_ = 0;
    if (minimize_) {
      best_result_ = kint64max;
    } else {
      best_result_ = kint64min;
    }
  }

  //  Returns true if limit is reached, false otherwise.
  virtual bool Check() {
    //VLOG(2) << "NoImprovementLimit's limit reached? " << limit_reached_;

    return false;
  }

  virtual bool AtSolution() {

    prototype_->Store();
    bool new_best = false;

    const IntVar* objective = prototype_->Objective();
    if (minimize_ && objective->Min() * 1.01 < best_result_) {
      best_result_ = objective->Min();
      if (intermediate_) {
        if (result_->routes_size() > 0) result_->clear_routes();
        result_->set_cost((int64)(best_result_ / 1000.0));
        result_->set_iterations(iteration_counter_);
        int current_break = 0;

        for (int route_nbr = 0; route_nbr < routing_->vehicles(); route_nbr++) {
          int route_break = 0;
          ortools_result::Route* route = result_->add_routes();
          int previous_index = -1;
          for (int64 index = routing_->Start(route_nbr); !routing_->IsEnd(index); index = routing_->NextVar(index)->Value()) {
            ortools_result::Activity* activity = route->add_activities();
            RoutingModel::NodeIndex nodeIndex = routing_->IndexToNode(index);
            activity->set_index(data_.ProblemIndex(nodeIndex));
            activity->set_start_time(routing_->GetMutableDimension("time")->CumulVar(index)->Min());
            activity->set_current_distance(routing_->GetMutableDimension("distance")->CumulVar(index)->Min());
            if (previous_index == -1) activity->set_type("start");
            else {
               if (index >= data_.SizeMissions()) {
                activity->set_type("break");
                activity->set_index(int64 (nodeIndex.value() - data_.SizeMissions()));
              } else {
                activity->set_type("service");
                activity->set_index(data_.ProblemIndex(nodeIndex));
                activity->set_alternative(data_.AlternativeIndex(nodeIndex));
                for (int64 q = 0 ; q < data_.Quantities(RoutingModel::NodeIndex(0)).size(); ++q) {
                  double exchange = routing_->GetMutableDimension("quantity" + std::to_string(q))->CumulVar(index)->Min();
                  activity->add_quantities(exchange/1000.);
                }
              }
            }
            previous_index = index;
          }
          ortools_result::Activity* end_activity = route->add_activities();
          RoutingModel::NodeIndex nodeIndex = routing_->IndexToNode(routing_->End(route_nbr));
          end_activity->set_index(data_.ProblemIndex(nodeIndex));
          end_activity->set_start_time(routing_->GetMutableDimension("time")->CumulVar(routing_->End(route_nbr))->Min());
          end_activity->set_current_distance(routing_->GetMutableDimension("distance")->CumulVar(routing_->End(route_nbr))->Min());
          end_activity->set_type("end");
        }
        std::fstream output(filename_, std::ios::out | std::ios::trunc | std::ios::binary);
        if (!result_->SerializeToOstream(&output)) {
          std::cout << "Failed to write result." << std::endl;
          return false;
        }
        output.close();
        std::cout << "Iteration : " << iteration_counter_ << " Cost : " << (int64)(best_result_ / 1000.0) << " Time : " << 1e-9 * (base::GetCurrentTimeNanos() - start_time_) << std::endl;
      }
      new_best = true;
    } else if (!minimize_ && objective->Max() * 0.99 > best_result_) {
      best_result_ = objective->Max();
      if (intermediate_) {
        if (result_->routes_size() > 0) result_->clear_routes();
        int current_break = 0;
        for (int route_nbr = 0; route_nbr < routing_->vehicles(); route_nbr++) {
          int route_break = 0;
          ortools_result::Route* route = result_->add_routes();
          int previous_index = -1;
          for (int64 index = routing_->Start(route_nbr); !routing_->IsEnd(index); index = routing_->NextVar(index)->Value()) {
            ortools_result::Activity* activity = route->add_activities();
            RoutingModel::NodeIndex nodeIndex = routing_->IndexToNode(index);
            activity->set_index(data_.ProblemIndex(nodeIndex));
            activity->set_start_time(routing_->GetMutableDimension("time")->CumulVar(index)->Min());
            if (previous_index == -1) activity->set_type("start");
            else {
               if (index >= data_.SizeMissions()) {
                activity->set_type("break");
                activity->set_index(int64 (nodeIndex.value() - data_.SizeMissions()));
              } else {
                activity->set_type("service");
                activity->set_index(data_.ProblemIndex(nodeIndex));
                activity->set_alternative(data_.AlternativeIndex(nodeIndex));
              }
              for (int64 q = 0 ; q < data_.Quantities(RoutingModel::NodeIndex(0)).size(); ++q) {
                double exchange = routing_->GetMutableDimension("quantity" + std::to_string(q))->CumulVar(index)->Min();
                activity->add_quantities(exchange/1000.);
              }
            }
            previous_index = index;
          }
          ortools_result::Activity* end_activity = route->add_activities();
          RoutingModel::NodeIndex nodeIndex = routing_->IndexToNode(routing_->End(route_nbr));
          end_activity->set_index(data_.ProblemIndex(nodeIndex));
          end_activity->set_start_time(routing_->GetMutableDimension("time")->CumulVar(routing_->End(route_nbr))->Min());
          end_activity->set_type("end");
        }
        std::fstream output(filename_, std::ios::out | std::ios::trunc | std::ios::binary);
        if (!result_->SerializeToOstream(&output)) {
          std::cout << "Failed to write result." << std::endl;
          return false;
        }
        output.close();
        std::cout << "Iteration : " << iteration_counter_ << " Cost : " << (int64)(best_result_ / 1000.0) << " Time : " << 1e-9 * (base::GetCurrentTimeNanos() - start_time_) << std::endl;
      }
      new_best = true;
    }

    if (debug_ && new_best) {
      std::cout << "min start : " << min_start_ << std::endl;
      for (RoutingModel::NodeIndex i(0); i < data_.SizeMatrix() - 1; ++i) {
          int64 index = routing_->NodeToIndex(i);
          IntVar *cumul_var = routing_->CumulVar(index, "time");
          IntVar *transit_var = routing_->TransitVar(index, "time");
          IntVar *slack_var = routing_->SlackVar(index, "time");
          IntVar *const vehicle_var = routing_->VehicleVar(index);
          if (vehicle_var->Bound() && cumul_var->Bound() && transit_var->Bound() && slack_var->Bound()) {
            std::cout << "Node " << i << " index " << index << " ["<< vehicle_var->Value() << "] |";
            std::cout << (cumul_var->Value() - min_start_) << " + " << transit_var->Value() << " -> " << slack_var->Value() << std::endl;
          }
      }
      std::cout << "-----------" << std::endl;
    }

    ++iteration_counter_;
    if(iteration_counter_ >= std::pow(2,pow_)) {
      std::cout << "Iteration : " << iteration_counter_ << std::endl;
      ++pow_;
    }
    return true;
  }

  virtual void Copy(const SearchLimit* const limit) {
    const LoggerMonitor* const copy_limit =
    reinterpret_cast<const LoggerMonitor* const>(limit);

    best_result_ = copy_limit->best_result_;
    iteration_counter_ = copy_limit->iteration_counter_;
    start_time_ = copy_limit->start_time_;
    size_matrix_ = copy_limit->size_matrix_;
    result_ = copy_limit->result_;
    minimize_ = copy_limit->minimize_;
    limit_reached_ = copy_limit->limit_reached_;
  }

  // Allocates a clone of the limit
  virtual SearchLimit* MakeClone() const {
    // we don't to copy the variables
    return solver_->RevAlloc(new LoggerMonitor(data_, routing_, min_start_, size_matrix_, debug_, intermediate_, result_, filename_, minimize_));
  }

  virtual std::string DebugString() const {
    return StringPrintf("LoggerMonitor(crossed = %i)", limit_reached_);
  }

  std::vector<double> GetFinalScore() {
    return {(best_result_ / 1000.0), 1e-9 * (base::GetCurrentTimeNanos() - start_time_), (double)iteration_counter_};
  }

  void GetFinalLog() {
    std::cout << "Final Iteration : " << iteration_counter_ << " Cost : " << (int64)(best_result_ / 1000.0) << " Time : " << 1e-9 * (base::GetCurrentTimeNanos() - start_time_) << std::endl;
  }

  private:
    const TSPTWDataDT &data_;
    RoutingModel * routing_;
    Solver * const solver_;
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

LoggerMonitor * MakeLoggerMonitor(const TSPTWDataDT &data, RoutingModel * routing, int64 min_start, int64 size_matrix, bool debug, bool intermediate, ortools_result::Result* result, std::string filename, const bool minimize = true) {
  return routing->solver()->RevAlloc(new LoggerMonitor(data, routing, min_start, size_matrix, debug, intermediate, result, filename, minimize));
}
}  //  namespace operations_research

#endif //  OR_TOOLS_TUTORIALS_CPLUSPLUS_LIMITS_H
