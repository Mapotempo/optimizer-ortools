#ifndef OR_TOOLS_TUTORIALS_CPLUSPLUS_LIMITS_H
#define OR_TOOLS_TUTORIALS_CPLUSPLUS_LIMITS_H

#include <ostream>
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
    NoImprovementLimit(Solver * const solver, IntVar * const objective_var, int solution_nbr_tolerance, const bool minimize = true) :
    SearchLimit(solver),
      solver_(solver), prototype_(new Assignment(solver_)),
      solution_nbr_tolerance_(solution_nbr_tolerance),
      nbr_solutions_with_no_better_obj_(0),
      iteration_counter_(0),
      minimize_(minimize),
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
    //VLOG(2) << "NoImprovementLimit's limit reached? " << limit_reached_;

    return limit_reached_;
  }

  virtual bool AtSolution() {
    ++nbr_solutions_with_no_better_obj_;
    ++iteration_counter_;

    prototype_->Store();

    const IntVar* objective = prototype_->Objective();
    if (minimize_ && objective->Min() < best_result_) {
      best_result_ = objective->Min();
      std::cout << "Iteration : " << iteration_counter_ << " Cost : " << best_result_ << std::endl;
      nbr_solutions_with_no_better_obj_ = 0;
    } else if (!minimize_ && objective->Max() > best_result_) {
      best_result_ = objective->Max();
      std::cout << "Iteration : " << iteration_counter_ << " Cost : " << best_result_ << std::endl;
      nbr_solutions_with_no_better_obj_ = 0;
    }

    if (nbr_solutions_with_no_better_obj_ > solution_nbr_tolerance_) {
      limit_reached_ = true;
    }
    return true;
  }

  virtual void Copy(const SearchLimit* const limit) {
    const NoImprovementLimit* const copy_limit =
    reinterpret_cast<const NoImprovementLimit* const>(limit);

    best_result_ = copy_limit->best_result_;
    solution_nbr_tolerance_ = copy_limit->solution_nbr_tolerance_;
    minimize_ = copy_limit->minimize_;
    limit_reached_ = copy_limit->limit_reached_;
    nbr_solutions_with_no_better_obj_ = copy_limit->nbr_solutions_with_no_better_obj_;
  }

  // Allocates a clone of the limit
  virtual SearchLimit* MakeClone() const {
    // we don't to copy the variables
    return solver_->RevAlloc(new NoImprovementLimit(solver_, prototype_->Objective(), solution_nbr_tolerance_, minimize_));
  }

  virtual std::string DebugString() const {
    return StringPrintf("NoImprovementLimit(crossed = %i)", limit_reached_);
  }
  
  private:
    Solver * const solver_;
    int64 best_result_;
    int solution_nbr_tolerance_;
    bool minimize_;
    bool limit_reached_;
    int iteration_counter_;
    int nbr_solutions_with_no_better_obj_;
    std::unique_ptr<Assignment> prototype_;
};

} // namespace

NoImprovementLimit * MakeNoImprovementLimit(Solver * const solver, IntVar * const objective_var, const int solution_nbr_tolerance, const bool minimize = true) {
  return solver->RevAlloc(new NoImprovementLimit(solver, objective_var, solution_nbr_tolerance, minimize));
}
}  //  namespace operations_research

#endif //  OR_TOOLS_TUTORIALS_CPLUSPLUS_LIMITS_H