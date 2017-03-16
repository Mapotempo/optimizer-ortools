#ifndef OR_TOOLS_TUTORIALS_CPLUSPLUS_LIMITS_H
#define OR_TOOLS_TUTORIALS_CPLUSPLUS_LIMITS_H

#include <ostream>
#include <chrono>
#include <iomanip>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>

#include "base/bitmap.h"
#include "base/logging.h"
#include "base/file.h"
#include "base/split.h"
#include "base/filelinereader.h"
#include "base/join.h"
#include "base/strtoint.h"

#include "constraint_solver/constraint_solver.h"

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
      previous_time_(start_time_),
      initial_time_out_(time_out),
      time_out_(10*time_out),
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
    previous_time_ = base::GetCurrentTimeNanos();
    limit_reached_ = false;
    if (minimize_) {
      best_result_ = kint64max;
    } else {
      best_result_ = kint64min;
    }
  }

  //  Returns true if limit is reached, false otherwise.
  virtual bool Check() {
    if (!first_solution_ && nbr_solutions_with_no_better_obj_ > solution_nbr_tolerance_ || 1e-6 * (base::GetCurrentTimeNanos() - previous_time_) > time_out_) {
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
      previous_time_ = base::GetCurrentTimeNanos();
      nbr_solutions_with_no_better_obj_ = 0;
      time_out_ = std::max(initial_time_out_ - 1e-6 * (base::GetCurrentTimeNanos() - start_time_), time_out_coef_ * 1e-6 * (base::GetCurrentTimeNanos() - start_time_));
    } else if (!minimize_ && objective->Max() * 0.99 > best_result_) {
      if (first_solution_) {
        first_solution_ = false;
      }
      best_result_ = objective->Max();
      previous_time_ = base::GetCurrentTimeNanos();
      nbr_solutions_with_no_better_obj_ = 0;
      time_out_ = std::max(initial_time_out_ - 1e-6 * (base::GetCurrentTimeNanos() - start_time_), time_out_coef_ * 1e-6 * (base::GetCurrentTimeNanos() - start_time_));
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
    previous_time_ = copy_limit->previous_time_;
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
    double previous_time_;
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
    LoggerMonitor(const TSPTWDataDT &data, RoutingModel * routing, int64 min_start, int64 size_matrix, vector<IntVar*> breaks, bool debug, const bool minimize = true) :
    data_(data),
    routing_(routing),
    SearchLimit(routing->solver()),
    solver_(routing->solver()), prototype_(new Assignment(solver_)),
    iteration_counter_(0),
    start_time_(base::GetCurrentTimeNanos()),
    pow_(0),
    min_start_(min_start),
    size_matrix_(size_matrix),
    breaks_(breaks),
    debug_(debug),
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
      std::cout << "Iteration : " << iteration_counter_ << " Cost : " << best_result_ / 1000.0 << " Time : " << 1e-9 * (base::GetCurrentTimeNanos() - start_time_) << std::endl;
      int current_break = 0;
      for (int route_nbr = 0; route_nbr < routing_->vehicles(); route_nbr++) {
        int previous_index = -1;
        for (int64 index = routing_->Start(route_nbr); !routing_->IsEnd(index); index = routing_->NextVar(index)->Value()) {
          RoutingModel::NodeIndex nodeIndex = routing_->IndexToNode(index);
          std::cout << data_.MatrixIndex(nodeIndex);
          if (previous_index != -1)
            std::cout << "[" << routing_->GetMutableDimension("time")->CumulVar(index)->Min()/100 << "]";
          std::cout << ",";
          if (current_break < data_.Rests().size() && data_.Vehicles().at(route_nbr)->break_size > 0 && breaks_[current_break]->Value() == index) {
            std::cout << size_matrix_ + current_break << ",";
            current_break++;
          }
          previous_index = index;
        }
        if (current_break < data_.Rests().size() && data_.Vehicles().at(route_nbr)->break_size > 0 && breaks_[current_break]->Value() == routing_->End(route_nbr)) {
            std::cout << size_matrix_ + current_break << ",";
            current_break++;
        }
        std::cout << data_.MatrixIndex(routing_->IndexToNode(routing_->End(route_nbr))) << ";";
      }
      std::cout << std::endl;

      new_best = true;
    } else if (!minimize_ && objective->Max() * 0.99 > best_result_) {
      best_result_ = objective->Max();
      std::cout << "Iteration : " << iteration_counter_ << " Cost : " << best_result_ / 1000.0 << " Time : " << 1e-9 * (base::GetCurrentTimeNanos() - start_time_) << std::endl;
      int current_break = 0;
      for (int route_nbr = 0; route_nbr < routing_->vehicles(); route_nbr++) {
        for (int64 index = routing_->Start(route_nbr); !routing_->IsEnd(index); index = routing_->NextVar(index)->Value()) {
          RoutingModel::NodeIndex nodeIndex = routing_->IndexToNode(index);
          std::cout << data_.MatrixIndex(nodeIndex) << ",";
          if (current_break < data_.Rests().size() && data_.Vehicles().at(route_nbr)->break_size > 0 && breaks_[current_break]->Value() == index) {
            std::cout << size_matrix_ + current_break << ",";
            current_break++;
          }
        }
        if (current_break < data_.Rests().size() && data_.Vehicles().at(route_nbr)->break_size > 0 && breaks_[current_break]->Value() == routing_->End(route_nbr)) {
            std::cout << size_matrix_ + current_break << ",";
            current_break++;
        }
        std::cout << data_.MatrixIndex(routing_->IndexToNode(routing_->End(route_nbr))) << ";";
      }
      std::cout << std::endl;
      new_best = true;
    }

    if (debug_ && new_best) {
      std::cout << "min start : " << min_start_/100 << std::endl;
      for (RoutingModel::NodeIndex i(0); i < data_.SizeMatrix() - 1; ++i) {
          int64 index = routing_->NodeToIndex(i);
          IntVar *cumul_var = routing_->CumulVar(index, "time");
          IntVar *transit_var = routing_->TransitVar(index, "time");
          IntVar *slack_var = routing_->SlackVar(index, "time");
          IntVar *const vehicle_var = routing_->VehicleVar(index);
          if (vehicle_var->Bound() && cumul_var->Bound() && transit_var->Bound() && slack_var->Bound()) {
            std::cout << "Node " << i << " index " << index << " ["<< vehicle_var->Value() << "] |";
            std::cout << (cumul_var->Value() - min_start_)/100 << " + " << transit_var->Value()/100 << " -> " << slack_var->Value()/100 << std::endl;
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
    breaks_ = copy_limit->breaks_;
    minimize_ = copy_limit->minimize_;
    limit_reached_ = copy_limit->limit_reached_;
  }

  // Allocates a clone of the limit
  virtual SearchLimit* MakeClone() const {
    // we don't to copy the variables
    return solver_->RevAlloc(new LoggerMonitor(data_, routing_, min_start_, size_matrix_, breaks_, debug_, minimize_));
  }

  virtual std::string DebugString() const {
    return StringPrintf("LoggerMonitor(crossed = %i)", limit_reached_);
  }

  void GetFinalLog() {
      std::cout << "Final Iteration : " << iteration_counter_ << " Cost : " << best_result_ / 1000.0 << " Time : " << 1e-9 * (base::GetCurrentTimeNanos() - start_time_) << std::endl;
  }

  private:
    const TSPTWDataDT &data_;
    RoutingModel * routing_;
    Solver * const solver_;
    int64 best_result_;
    double start_time_;
    int64 min_start_;
    int64 size_matrix_;
    vector<IntVar*> breaks_;
    bool minimize_;
    bool limit_reached_;
    bool debug_;
    int64 pow_;
    int64 iteration_counter_;
    std::unique_ptr<Assignment> prototype_;
};

} // namespace

LoggerMonitor * MakeLoggerMonitor(const TSPTWDataDT &data, RoutingModel * routing, int64 min_start, int64 size_matrix, vector<IntVar*> breaks, bool debug, const bool minimize = true) {
  return routing->solver()->RevAlloc(new LoggerMonitor(data, routing, min_start, size_matrix, breaks, debug, minimize));
}
}  //  namespace operations_research

#endif //  OR_TOOLS_TUTORIALS_CPLUSPLUS_LIMITS_H
