// Copyright © Mapotempo, 2013-2015
//
// This file is part of Mapotempo.
//
// Mapotempo is free software. You can redistribute it and/or
// modify since you respect the terms of the GNU Affero General
// Public License as published by the Free Software Foundation,
// either version 3 of the License, or (at your option) any later version.
//
// Mapotempo is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE.  See the Licenses for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with Mapotempo. If not, see:
// <http://www.gnu.org/licenses/agpl.html>
//
#include <iostream>

#include "base/commandlineflags.h"
#include "constraint_solver/routing.h"
#include "base/join.h"
#include "base/timer.h"
#include <base/callback.h>

#include "tsptw_data_dt.h"
#include "tsptw_solution_dt.h"
#include "limits.h"
#include "routing_common/routing_common_flags.h"

#include "constraint_solver/routing.h"
#include "constraint_solver/routing_flags.h"

DEFINE_int64(time_limit_in_ms, -1, "Time limit in ms, 0 means no limit.");
DEFINE_int64(soft_upper_bound, 3, "Soft upper bound multiplicator, 0 means hard limit.");
DEFINE_bool(nearby, false, "short segment priority");
DEFINE_int64(no_solution_improvement_limit, -1,"Iterations whitout improvement");
DEFINE_int64(initial_time_out_no_solution_improvement, 30, "Initial time whitout improvement");
DEFINE_int64(time_out_multiplier, 2, "Multiplier for the nexts time out");

namespace operations_research {

void TWBuilder(const TSPTWDataDT &data, RoutingModel &routing, Solver *solver, int64 begin_index, int64 size) {
  for (RoutingModel::NodeIndex i(begin_index); i < begin_index + size; ++i) {

    int64 const first_ready = data.FirstTWReadyTime(i);
    int64 const first_due = data.FirstTWDueTime(i);
    int64 const second_ready = data.SecondTWReadyTime(i);
    int64 const second_due = data.SecondTWDueTime(i);

    if (first_ready > -2147483648 || first_due < 2147483647) {
      int64 index = routing.NodeToIndex(i);
      IntVar *const cumul_var = routing.CumulVar(index, "time");

      if (first_ready > -2147483648) {
        cumul_var->SetMin(first_ready);
      }

      if (second_ready > -2147483648) {
        IntVar* const cost_var = solver->MakeSum(
          solver->MakeConditionalExpression(solver->MakeIsLessOrEqualCstVar(cumul_var, second_ready), solver->MakeSemiContinuousExpr(solver->MakeSum(cumul_var, -first_due), 0, FLAGS_soft_upper_bound), 0),
          solver->MakeConditionalExpression(solver->MakeIsGreaterOrEqualCstVar(cumul_var, second_due), solver->MakeSemiContinuousExpr(solver->MakeSum(cumul_var, -second_due), 0, FLAGS_soft_upper_bound), 0)
        )->Var();
        routing.AddVariableMinimizedByFinalizer(cost_var);
      } else if (first_due < 2147483647) {
        if (FLAGS_soft_upper_bound > 0) {
          routing.SetCumulVarSoftUpperBound(i, "time", first_due, FLAGS_soft_upper_bound);
        } else {
          routing.SetCumulVarSoftUpperBound(i, "time", first_due, 10000000);
        }
      }
    }

    std::vector<RoutingModel::NodeIndex> *vect = new std::vector<RoutingModel::NodeIndex>(1);
    (*vect)[0] = i;
    routing.AddDisjunction(*vect, std::pow(2, 56));
  }
}

void TSPTWSolver(const TSPTWDataDT &data) {

  const int size = data.Size();
  const int size_matrix = data.SizeMatrix();
  const int size_rest = data.SizeRest();

  std::vector<std::pair<RoutingModel::NodeIndex, RoutingModel::NodeIndex>> *start_ends = new std::vector<std::pair<RoutingModel::NodeIndex, RoutingModel::NodeIndex>>(1);
  (*start_ends)[0] = std::make_pair(data.Start(), data.Stop());
  RoutingModel routing(size, 1, *start_ends);

  const int64 horizon = data.Horizon();
  routing.AddDimension(NewPermanentCallback(&data, &TSPTWDataDT::TimePlusServiceTime), horizon, horizon, true, "time");
  routing.AddDimension(NewPermanentCallback(&data, &TSPTWDataDT::Distance), 0, LLONG_MAX, true, "distance");
  if (FLAGS_nearby) {
    routing.AddDimension(NewPermanentCallback(&data, &TSPTWDataDT::TimeOrder), horizon, horizon, true, "order");
    routing.GetMutableDimension("order")->SetSpanCostCoefficientForAllVehicles(1);
  }

  routing.GetMutableDimension("time")->SetSpanCostCoefficientForAllVehicles(5);


  Solver *solver = routing.solver();

  //  Setting visit time windows
  TWBuilder(data, routing, solver, 1, size_matrix - 2);

  // Setting rest time windows
  TWBuilder(data, routing, solver, size_matrix, size_rest);

  RoutingSearchParameters parameters = BuildSearchParametersFromFlags();

  // Search strategy
  // parameters.set_first_solution_strategy(FirstSolutionStrategy::FIRST_UNBOUND_MIN_VALUE); // Default
  // parameters.set_first_solution_strategy(FirstSolutionStrategy::PATH_CHEAPEST_ARC);
  // parameters.set_first_solution_strategy(FirstSolutionStrategy::PATH_MOST_CONSTRAINED_ARC);
  // parameters.set_first_solution_strategy(FirstSolutionStrategy::CHRISTOFIDES);
  // parameters.set_first_solution_strategy(FirstSolutionStrategy::ALL_UNPERFORMED);
  // parameters.set_first_solution_strategy(FirstSolutionStrategy::BEST_INSERTION);
  // parameters.set_first_solution_strategy(FirstSolutionStrategy::ROUTING_BEST_INSERTION);
  // parameters.set_first_solution_strategy(FirstSolutionStrategy::PARALLEL_CHEAPEST_INSERTION);
  // parameters.set_first_solution_strategy(FirstSolutionStrategy::LOCAL_CHEAPEST_INSERTION);
  // parameters.set_first_solution_strategy(FirstSolutionStrategy::GLOBAL_CHEAPEST_ARC);
  // parameters.set_first_solution_strategy(FirstSolutionStrategy::LOCAL_CHEAPEST_ARC);
  // parameters.set_first_solution_strategy(FirstSolutionStrategy::ROUTING_BEST_INSERTION);

  // parameters.set_metaheuristic(LocalSearchMetaheuristic::ROUTING_GREEDY_DESCENT);
  parameters.set_local_search_metaheuristic(LocalSearchMetaheuristic::GUIDED_LOCAL_SEARCH);
  // parameters.set_guided_local_search_lambda_coefficient(0.5);
  // parameters.set_metaheuristic(LocalSearchMetaheuristic::ROUTING_SIMULATED_ANNEALING);
  // parameters.set_metaheuristic(LocalSearchMetaheuristic::ROUTING_TABU_SEARCH);

  // routing.SetCommandLineOption("routing_no_lns", "true");

  if (FLAGS_time_limit_in_ms > 0) {
    parameters.set_time_limit_ms(FLAGS_time_limit_in_ms);
  }

  routing.CloseModelWithParameters(parameters);

  LoggerMonitor * const logger = MakeLoggerMonitor(routing.solver(), routing.CostVar(), true);
  routing.AddSearchMonitor(logger);

  if (data.Size() > 3) {
    if (FLAGS_no_solution_improvement_limit > 0) {
      NoImprovementLimit * const no_improvement_limit = MakeNoImprovementLimit(routing.solver(), routing.CostVar(), FLAGS_no_solution_improvement_limit, FLAGS_initial_time_out_no_solution_improvement, FLAGS_time_out_multiplier, true);
      routing.AddSearchMonitor(no_improvement_limit);
    }
  } else {
    SearchLimit * const limit = solver->MakeLimit(kint64max,kint64max,kint64max,1);
    routing.AddSearchMonitor(limit);
  }

  const Assignment *solution = routing.SolveWithParameters(parameters);

  if (solution != NULL) {
    float cost = solution->ObjectiveValue() / 500.0; // Back to original cost value after GetMutableDimension("time")->SetSpanCostCoefficientForAllVehicles(5)
    logger->GetFinalLog();
    TSPTWSolution sol(data, &routing, solution);
    for (int route_nbr = 0; route_nbr < routing.vehicles(); route_nbr++) {
      for (int64 index = routing.Start(route_nbr); !routing.IsEnd(index); index = solution->Value(routing.NextVar(index))) {
        RoutingModel::NodeIndex nodeIndex = routing.IndexToNode(index);
        std::cout << nodeIndex << " ";
      }
      std::cout << routing.IndexToNode(routing.End(route_nbr)) << std::endl;
    }
  } else {
    std::cout << "No solution found..." << std::endl;
  }
}

} // namespace operations_research

int main(int argc, char **argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  if(FLAGS_time_limit_in_ms > 0 || FLAGS_no_solution_improvement_limit > 0) {
    operations_research::TSPTWDataDT tsptw_data(FLAGS_instance_file);
    operations_research::TSPTWSolver(tsptw_data);
  } else {
    std::cout << "No Stop condition" << std::endl;
  }

  return 0;
}
