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
#include "limits.h"

#include "constraint_solver/routing.h"
#include "constraint_solver/routing_flags.h"

DEFINE_int64(time_limit_in_ms, 0, "Time limit in ms, no option means no limit.");
DEFINE_int64(no_solution_improvement_limit, -1,"Iterations whitout improvement");
DEFINE_int64(initial_time_out_no_solution_improvement, 30000, "Initial time whitout improvement in ms");
DEFINE_int64(time_out_multiplier, 2, "Multiplier for the nexts time out");
DEFINE_bool(nearby, false, "Short segment priority");

#define DISJUNCTION_COST std::pow(2, 32)
#define NO_LATE_MULTIPLIER (DISJUNCTION_COST+1)
#define NO_OVERLOAD_MULTIPLIER (DISJUNCTION_COST+1)

namespace operations_research {

void TWBuilder(const TSPTWDataDT &data, RoutingModel &routing, Solver *solver, int64 begin_index, int64 size) {
  for (RoutingModel::NodeIndex i(begin_index); i < begin_index + size; ++i) {

    int64 const first_ready = data.FirstTWReadyTime(i);
    int64 const first_due = data.FirstTWDueTime(i);
    int64 const second_ready = data.SecondTWReadyTime(i);
    int64 const second_due = data.SecondTWDueTime(i);
    int64 const late_multiplier = data.LateMultiplier(i);
    std::vector<int64> sticky_vehicle = data.VehicleIndices(i);
    int64 index = routing.NodeToIndex(i);

    if (first_ready > -2147483648 || first_due < 2147483647) {
      IntVar *const cumul_var = routing.CumulVar(index, "time");

      if (first_ready > -2147483648) {
        cumul_var->SetMin(first_ready);
      }

      if (late_multiplier > 0) {
        if (second_ready > -2147483648) {
          IntVar* const cost_var = solver->MakeSum(
            solver->MakeConditionalExpression(solver->MakeIsLessOrEqualCstVar(cumul_var, second_ready), solver->MakeSemiContinuousExpr(solver->MakeSum(cumul_var, -first_due), 0, late_multiplier), 0),
            solver->MakeConditionalExpression(solver->MakeIsGreaterOrEqualCstVar(cumul_var, second_due), solver->MakeSemiContinuousExpr(solver->MakeSum(cumul_var, -second_due), 0, late_multiplier), 0)
          )->Var();
          routing.AddVariableMinimizedByFinalizer(cost_var);
        } else if (first_due < 2147483647) {
          routing.SetCumulVarSoftUpperBound(i, "time", first_due, late_multiplier);
        }
      } else {
        if (second_ready > -2147483648) {
          cumul_var->SetMax(second_due);
          //Simplify at next ORtools release 09/16
          std::vector<int64> forbid_starts(1, first_due);
          std::vector<int64> forbid_ends(1, second_ready);
          solver->AddConstraint(solver->MakeNotMemberCt(cumul_var, forbid_starts, forbid_ends));
        } else if(first_due < 2147483647) {
          cumul_var->SetMax(first_due);
        }
      }
    }
    if(sticky_vehicle.size() > 0) {
      int v = 0;
      for(TSPTWDataDT::Vehicle* vehicle: data.Vehicles()) {
        IntVar *const vehicle_var = routing.VehicleVar(v);
        if(std::find(sticky_vehicle.begin(), sticky_vehicle.end(), v) != sticky_vehicle.end()) {
          vehicle_var->RemoveValue(index);
        }
        ++v;
      }
    }

    std::vector<RoutingModel::NodeIndex> *vect = new std::vector<RoutingModel::NodeIndex>(1);
    (*vect)[0] = i;
    routing.AddDisjunction(*vect, DISJUNCTION_COST);
  }
}

void TSPTWSolver(const TSPTWDataDT &data) {
  const int size_vehicles = data.Vehicles().size();
  const int size = data.Size();
  const int size_matrix = data.SizeMatrix();
  const int size_rest = data.SizeRest();

  std::vector<std::pair<RoutingModel::NodeIndex, RoutingModel::NodeIndex>> *start_ends = new std::vector<std::pair<RoutingModel::NodeIndex, RoutingModel::NodeIndex>>(size_vehicles);
  for(int v = 0; v < size_vehicles; ++v) {
    (*start_ends)[v] = std::make_pair(data.Vehicles().at(v)->start, data.Vehicles().at(v)->stop);
  }
  RoutingModel routing(size, size_vehicles, *start_ends);

  // Dimensions
  const int64 horizon = data.Horizon();
  std::vector<ResultCallback2<long long int, IntType<operations_research::_RoutingModel_NodeIndex_tag_, int>, IntType<operations_research::_RoutingModel_NodeIndex_tag_, int> >*> time_evaluators;
  std::vector<ResultCallback2<long long int, IntType<operations_research::_RoutingModel_NodeIndex_tag_, int>, IntType<operations_research::_RoutingModel_NodeIndex_tag_, int> >*> distance_evaluators;
  std::vector<ResultCallback2<long long int, IntType<operations_research::_RoutingModel_NodeIndex_tag_, int>, IntType<operations_research::_RoutingModel_NodeIndex_tag_, int> >*> order_evaluators;
  for (TSPTWDataDT::Vehicle* vehicle: data.Vehicles()) {
    time_evaluators.push_back(NewPermanentCallback(vehicle, &TSPTWDataDT::Vehicle::TimePlusServiceTime));
    distance_evaluators.push_back(NewPermanentCallback(vehicle, &TSPTWDataDT::Vehicle::Distance));
    if (FLAGS_nearby) {
      order_evaluators.push_back(NewPermanentCallback(vehicle, &TSPTWDataDT::Vehicle::TimeOrder));
    }
  }
  routing.AddDimensionWithVehicleTransits(time_evaluators, horizon, horizon, false, "time");
  routing.GetMutableDimension("time")->SetSpanCostCoefficientForAllVehicles(5);
  routing.AddDimensionWithVehicleTransits(distance_evaluators, 0, LLONG_MAX, true, "distance");
  if (FLAGS_nearby) {
    routing.AddDimensionWithVehicleTransits(order_evaluators, horizon, horizon, true, "order");
    routing.GetMutableDimension("order")->SetSpanCostCoefficientForAllVehicles(1);
  }

  for (int64 i = 0; i < data.Vehicles().at(0)->capacity.size(); ++i) {
    routing.AddDimension(NewPermanentCallback(&data, &TSPTWDataDT::Quantity, NewPermanentCallback(&routing, &RoutingModel::NodeToIndex), i), 0, LLONG_MAX, true, "quantity" + std::to_string(i));
  }

  Solver *solver = routing.solver();

  // Setting visit time windows
  TWBuilder(data, routing, solver, 0, size_matrix - 2);

  // Setting rest time windows
  TWBuilder(data, routing, solver, size_matrix, size_rest);

  // Vehicle time windows
  int64 v = 0;
  for(TSPTWDataDT::Vehicle* vehicle: data.Vehicles()) {
    if (vehicle->time_start > -2147483648) {
      int64 index = routing.Start(v);
      IntVar *const cumul_var = routing.CumulVar(index, "time");
      cumul_var->SetMin(vehicle->time_start);
    }
    if (vehicle->time_end < 2147483647) {
      int64 coef = vehicle->late_multiplier;
      if(coef > 0) {
        routing.GetMutableDimension("time")->SetEndCumulVarSoftUpperBound(v, vehicle->time_end, coef);
      } else {
        int64 index = routing.End(v);
        IntVar *const cumul_var = routing.CumulVar(index, "time");
        cumul_var->SetMax(vehicle->time_end);
      }
    }

    for (int64 i = 0; i < vehicle->capacity.size(); ++i) {
      int64 coef = vehicle->overload_multiplier[i];
      if(coef > 0) {
        routing.GetMutableDimension("quantity" + std::to_string(i))->SetEndCumulVarSoftUpperBound(v, vehicle->capacity[i], coef);
      } else {
        int64 index = routing.End(v);
        IntVar *const cumul_var = routing.CumulVar(index, "quantity" + std::to_string(i));
        cumul_var->SetMax(vehicle->capacity[i]);
      }
    }

    ++v;
  }

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
    for (int route_nbr = 0; route_nbr < routing.vehicles(); route_nbr++) {
      for (int64 index = routing.Start(route_nbr); !routing.IsEnd(index); index = solution->Value(routing.NextVar(index))) {
        RoutingModel::NodeIndex nodeIndex = routing.IndexToNode(index);
        std::cout << nodeIndex << ",";
      }
      std::cout << routing.IndexToNode(routing.End(route_nbr)) << ";";
    }
    std::cout << std::endl;
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