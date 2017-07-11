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
DEFINE_int64(vehicle_limit, 0, "Define the maximum number of vehicle");
DEFINE_bool(nearby, false, "Short segment priority");
DEFINE_bool(debug, false, "debug display");


namespace operations_research {

bool CheckOverflow(int64 a, int64 b) {
  if ( a > std::pow(2, 52) / b)
    return true;
  return false;
}

void TWBuilder(const TSPTWDataDT &data, RoutingModel &routing, Solver *solver, int64 size, int64 min_start, bool loop_route, bool unique_configuration) {
  const int size_vehicles = data.Vehicles().size();
  int64 max_time = (2 * data.MaxTime() + data.MaxServiceTime()) * data.MaxTimeCost();
  int64 max_distance = 2 * data.MaxDistance() * data.MaxDistanceCost();

  bool overflow_danger = CheckOverflow(max_time + max_distance, size_vehicles * size_vehicles);
  int64 data_verif = (max_time + max_distance)  * size_vehicles * size_vehicles;

  overflow_danger = overflow_danger || CheckOverflow(data_verif, size * size);
  data_verif = data_verif * size * size;
  RoutingModel::NodeIndex i(0);
  int32 tw_index = 0;

  int64 disjunction_cost = !overflow_danger && !CheckOverflow(data_verif, size)? data_verif : std::pow(2, 52);
  for (int activity = 0; activity < data.SizeMatrix() - 2; ++activity) {
    std::vector<RoutingModel::NodeIndex> *vect = new std::vector<RoutingModel::NodeIndex>(1);
    int64 priority = 4;

    std::vector<std::string>* linked_ids = data.LinkedServicesIds(i);
    if (linked_ids != NULL) {
      for (int link_index = 0 ; link_index < linked_ids->size(); ++link_index) {
        routing.AddPickupAndDelivery(i, RoutingModel::NodeIndex(data.IdIndex(linked_ids->at(link_index))));
        solver->AddConstraint(solver->MakeEquality(
            routing.VehicleVar(routing.NodeToIndex(i)),
            routing.VehicleVar(data.IdIndex(linked_ids->at(link_index)))));
        solver->AddConstraint(
          solver->MakeLessOrEqual(routing.GetMutableDimension("time")->CumulVar(routing.NodeToIndex(i)),
                                  routing.GetMutableDimension("time")->CumulVar(data.IdIndex(linked_ids->at(link_index)))));
      }
    }

    priority = data.Priority(i);

    int64 index = routing.NodeToIndex(i);
    std::vector<int64> ready = data.ReadyTime(i);
    std::vector<int64> due = data.DueTime(i);

    IntVar* cumul_var = routing.CumulVar(index, "time");
    int64 const late_multiplier = data.LateMultiplier(i);
    std::vector<int64> sticky_vehicle = data.VehicleIndices(i);
    std::string service_id = data.ServiceId(i);
    if (ready.size() > 0 && (ready.at(0) > -CUSTOM_MAX_INT || due.at(due.size()- 1) < CUSTOM_MAX_INT)) {
      if (FLAGS_debug) {
        std::cout << "Node " << i << " index " << index << " [" << (ready.at(0) - min_start) << " : " << (due.at(due.size()- 1) - min_start) << "]:" << data.ServiceTime(i) << std::endl;
      }
      if (ready.at(0) > -CUSTOM_MAX_INT) {
        cumul_var->SetMin(ready.at(0));
      }
      if (due.at(due.size()- 1) < CUSTOM_MAX_INT) {
        if (late_multiplier > 0) {
          routing.SetCumulVarSoftUpperBound(i, "time", due.at(due.size()- 1), late_multiplier);
        } else {
          cumul_var->SetMax(due.at(due.size()- 1));
        }
      }
    }
    if (sticky_vehicle.size() > 0) {
      for (int v = 0; v < size_vehicles; ++v) {
        bool sticked = false;
        for (int64 sticky : sticky_vehicle) {
          if (v == sticky)
            sticked = true;
        }
        if (!sticked) {
          routing.VehicleVar(index)->RemoveValue(v);
        }
      }
    }
    for (int64 q = 0 ; q < data.Quantities(i).size(); ++q) {
      IntVar *const slack_var = routing.SlackVar(index, "quantity" + std::to_string(q));
      slack_var->SetValue(0);
    }
    (*vect)[0] = i;
    if (late_multiplier > 0) {
      ++i;
      ready = data.ReadyTime(i);
      due = data.DueTime(i);
      while (data.ServiceId(i) == service_id) {
        index = routing.NodeToIndex(i);
        cumul_var = routing.CumulVar(index, "time");
        if (ready.at(0) > -CUSTOM_MAX_INT) {
          cumul_var->SetMin(ready.at(0));
        }
        if (due.at(due.size()- 1) < CUSTOM_MAX_INT) {
          routing.SetCumulVarSoftUpperBound(i, "time", due.at(due.size()- 1), late_multiplier);
        }
        if (sticky_vehicle.size() > 0) {
          for (int v = 0; v < size_vehicles; ++v) {
            bool sticked = false;
            for (int64 sticky : sticky_vehicle) {
              if (v == sticky)
                sticked = true;
            }
            if (!sticked) {
              routing.VehicleVar(index)->RemoveValue(v);
            }
          }
        }
        for (int64 q = 0 ; q < data.Quantities(i).size(); ++q) {
          IntVar *const slack_var = routing.SlackVar(index, "quantity" + std::to_string(q));
          slack_var->SetValue(0);
        }
        vect->push_back(i);
        ++i;
        ready = data.ReadyTime(i);
        due = data.DueTime(i);
      }
    } else if (due.size() > 1) {
      for (tw_index = due.size() - 1; tw_index--; ) {
        cumul_var->RemoveInterval(due.at(tw_index), ready.at(tw_index + 1));
      }
      ++i;
    } else {
      ++i;
    }

    // Otherwise this single service is never assigned
    if (size == 1)
      routing.AddDisjunction(*vect);
    else
      routing.AddDisjunction(*vect, disjunction_cost * std::pow(2, 4 - priority));
  }
}

vector<IntVar*> RestBuilder(const TSPTWDataDT &data, RoutingModel &routing, Solver *solver, int64 size) {
  std::vector<IntVar*> breaks;
  for (TSPTWDataDT::Rest* rest: data.Rests()) {
    int vehicle_index = rest->vehicle;
    IntVar* break_position = solver->MakeIntVar(-1, CUSTOM_MAX_INT, "break position");
    breaks.push_back(break_position);
    std::vector<int64> values;
    if (data.Vehicles()[vehicle_index]->break_size > 0) {
      for (RoutingModel::NodeIndex i(0); i < size ; ++i) {
        int64 index;
        if ( i == size - 2) {
          index = routing.Start(vehicle_index);
        } else if ( i == size - 1){
          index = routing.End(vehicle_index);
        } else {
          index = routing.NodeToIndex(i);
        }
        values.push_back(index);

        IntVar *cumul_var = routing.CumulVar(index, "time");
        IntVar *slack_var = routing.SlackVar(index, "time");
        IntVar *const vehicle_var = routing.VehicleVar(index);
        if (i == size - 2) {
          solver->AddConstraint(solver->MakeGreaterOrEqual(cumul_var, data.Vehicles()[vehicle_index]->time_start));
        } else if (i == size -1) {
          slack_var = solver->MakeIntVar(0, CUSTOM_MAX_INT, "end slack");
        }
        // Verify if node is affected to the right vehicle
        IntVar *const remove_index = solver->MakeConditionalExpression(solver->MakeIsDifferentCstVar(vehicle_var, vehicle_index),
          solver->MakeIntConst(index), -1)->Var();
        solver->AddConstraint(solver->MakeNonEquality(break_position, remove_index));
        // Define break_duration if the break position is equal to the current node
        IntVar *const break_duration = solver->MakeConditionalExpression(solver->MakeIsEqualCstVar(break_position, index)->Var(),
          solver->MakeIntConst(rest->rest_duration), 0)->Var();
        // Add a waiting_time before the break if its timeWindow in not already open
        IntVar *const break_wait_duration = solver->MakeConditionalExpression(solver->MakeIsEqualCstVar(break_position, index)->Var(),
        solver->MakeMax(solver->MakeDifference(rest->rest_start, solver->MakeSum(cumul_var, solver->MakeIntConst(data.ServiceTime(i)))), 0), 0)->Var();
        routing.AddVariableMinimizedByFinalizer(break_wait_duration);
        // Associate the break position accordingly to its TW
        IntVar *const upper_rest_bound = solver->MakeConditionalExpression(solver->MakeIsEqualCstVar(break_position, index)->Var(),
          solver->MakeIntConst(rest->rest_end), CUSTOM_MAX_INT)->Var();
        solver->AddConstraint(solver->MakeGreaterOrEqual(slack_var, solver->MakeSum(break_wait_duration, break_duration)));
        solver->AddConstraint(solver->MakeLessOrEqual(solver->MakeSum(cumul_var, solver->MakeIntConst(data.ServiceTime(i))), upper_rest_bound));
      }
      rest++;
    } else {
      values.push_back(-1);
    }
    break_position->SetValues(values);
    routing.AddVariableMinimizedByFinalizer(break_position);
    routing.AddToAssignment(break_position);
  }
  return breaks;
}

void RelationBuilder(const TSPTWDataDT &data, RoutingModel &routing, Solver *solver, int64 size) {
  for (TSPTWDataDT::Relation* relation: data.Relations()) {
    std::string previous_id = "";
    switch (relation->type) {
      case Sequence:
        break;
      case Order:
        break;
      case SameRoute:
        for (int link_index = 0 ; link_index < relation->linked_ids->size(); ++link_index) {
          if (previous_id != "") {
            solver->AddConstraint(solver->MakeEquality(
                routing.VehicleVar(data.IdIndex(previous_id)),
                routing.VehicleVar(data.IdIndex(relation->linked_ids->at(link_index)))
                ));
          }
          previous_id = relation->linked_ids->at(link_index);
        }
        break;
      case MinimumDayLapse:
        for (int link_index = 0 ; link_index < relation->linked_ids->size(); ++link_index) {
          previous_id = relation->linked_ids->at(link_index);
          for (int link_index_bis = link_index + 1; link_index_bis < relation->linked_ids->size(); ++link_index_bis) {
            IntVar *const previous_active_var = routing.ActiveVar(data.IdIndex(previous_id));
            IntVar *const active_var = routing.ActiveVar(data.IdIndex(relation->linked_ids->at(link_index_bis)));

            IntVar *const previous_vehicle_day_var = solver->MakeConditionalExpression(solver->MakeIsDifferentCstVar(previous_active_var, 0),
              solver->MakeElement(data.VehiclesDay(), routing.VehicleVar(data.IdIndex(previous_id))), 0)->Var();

            IntVar *const vehicle_day_var = solver->MakeConditionalExpression(solver->MakeIsDifferentCstVar(active_var, 0),
              solver->MakeElement(data.VehiclesDay(), routing.VehicleVar(data.IdIndex(relation->linked_ids->at(link_index_bis)))), 0)->Var();

            IntVar *const day_lapse = solver->MakeConditionalExpression(solver->MakeIsDifferentCstVar(solver->MakeMin(previous_active_var, active_var), 0),
            solver->MakeDifference(vehicle_day_var, previous_vehicle_day_var), CUSTOM_MAX_INT)->Var();
            solver->AddConstraint(solver->MakeGreaterOrEqual(
              day_lapse,
              (link_index_bis - link_index) * relation->lapse));
          }
        }
        break;
      case MaximumDayLapse:
        for (int link_index = 0 ; link_index < relation->linked_ids->size(); ++link_index) {
          previous_id = relation->linked_ids->at(link_index);
          for (int link_index_bis = link_index + 1; link_index_bis < relation->linked_ids->size(); ++link_index_bis) {
            IntVar *const previous_active_var = routing.ActiveVar(data.IdIndex(previous_id));
            IntVar *const active_var = routing.ActiveVar(data.IdIndex(relation->linked_ids->at(link_index_bis)));

            IntVar *const previous_vehicle_day_var = solver->MakeConditionalExpression(solver->MakeIsDifferentCstVar(previous_active_var, 0),
              solver->MakeElement(data.VehiclesDay(), routing.VehicleVar(data.IdIndex(previous_id))), 0)->Var();

            IntVar *const vehicle_day_var = solver->MakeConditionalExpression(solver->MakeIsDifferentCstVar(active_var, 0),
              solver->MakeElement(data.VehiclesDay(), routing.VehicleVar(data.IdIndex(relation->linked_ids->at(link_index_bis)))), 0)->Var();

            IntVar *const day_lapse = solver->MakeConditionalExpression(solver->MakeIsDifferentCstVar(solver->MakeMin(previous_active_var, active_var), 0),
            solver->MakeDifference(vehicle_day_var, previous_vehicle_day_var), 0)->Var();
            solver->AddConstraint(solver->MakeLessOrEqual(
              day_lapse,
              (link_index_bis - link_index) * relation->lapse));
          }
        }
        break;
      default:
        break;
    }
  }
}

void TSPTWSolver(const TSPTWDataDT &data) {
  const int size_vehicles = data.Vehicles().size();
  const int size = data.Size();
  const int size_matrix = data.SizeMatrix();
  const int size_rest = data.SizeRest();
  const int size_mtws = data.TwiceTWsCounter();
  bool has_lateness = false;
  bool has_route_duration = false;

  std::vector<std::pair<RoutingModel::NodeIndex, RoutingModel::NodeIndex>> *start_ends = new std::vector<std::pair<RoutingModel::NodeIndex, RoutingModel::NodeIndex>>(size_vehicles);
  for(int v = 0; v < size_vehicles; ++v) {
    (*start_ends)[v] = std::make_pair(data.Vehicles().at(v)->start, data.Vehicles().at(v)->stop);
    has_lateness |= data.Vehicles().at(v)->late_multiplier > 0;
  }
  RoutingModel routing(size, size_vehicles, *start_ends);

  // Dimensions
  const int64 horizon = data.Horizon() * (has_lateness && !CheckOverflow(data.Horizon(), 2) ? 2 : 1);
  std::vector<ResultCallback2<long long int, IntType<operations_research::_RoutingModel_NodeIndex_tag_, int>, IntType<operations_research::_RoutingModel_NodeIndex_tag_, int> >*> time_evaluators;
  std::vector<ResultCallback2<long long int, IntType<operations_research::_RoutingModel_NodeIndex_tag_, int>, IntType<operations_research::_RoutingModel_NodeIndex_tag_, int> >*> distance_evaluators;
  std::vector<ResultCallback2<long long int, IntType<operations_research::_RoutingModel_NodeIndex_tag_, int>, IntType<operations_research::_RoutingModel_NodeIndex_tag_, int> >*> time_order_evaluators;
  std::vector<ResultCallback2<long long int, IntType<operations_research::_RoutingModel_NodeIndex_tag_, int>, IntType<operations_research::_RoutingModel_NodeIndex_tag_, int> >*> distance_order_evaluators;
  for (TSPTWDataDT::Vehicle* vehicle: data.Vehicles()) {
    time_evaluators.push_back(NewPermanentCallback(vehicle, &TSPTWDataDT::Vehicle::TimePlusServiceTime));
    distance_evaluators.push_back(NewPermanentCallback(vehicle, &TSPTWDataDT::Vehicle::Distance));
    if (FLAGS_nearby) {
      time_order_evaluators.push_back(NewPermanentCallback(vehicle, &TSPTWDataDT::Vehicle::TimeOrder));
      distance_order_evaluators.push_back(NewPermanentCallback(vehicle, &TSPTWDataDT::Vehicle::DistanceOrder));
    }
  }
  routing.AddDimensionWithVehicleTransits(time_evaluators, horizon, horizon, false, "time");
  routing.AddDimensionWithVehicleTransits(time_evaluators, horizon, horizon, false, "time_without_wait");
  routing.AddDimensionWithVehicleTransits(distance_evaluators, 0, LLONG_MAX, true, "distance");
  if (FLAGS_nearby) {
    routing.AddDimensionWithVehicleTransits(time_order_evaluators, 0, LLONG_MAX, true, "time_order");
    routing.AddDimensionWithVehicleTransits(distance_order_evaluators, 0, LLONG_MAX, true, "distance_order");
  }

  for (int64 i = 0; i < data.Vehicles().at(0)->capacity.size(); ++i) {
    std::vector<int64> capacities;
    for(TSPTWDataDT::Vehicle* vehicle: data.Vehicles()) {
      int64 coef = vehicle->overload_multiplier[i];
      if(coef == 0 && vehicle->capacity.at(i) >= 0) {
        capacities.push_back(vehicle->capacity.at(i));
      } else {
        capacities.push_back(LLONG_MAX);
      }
    }
    routing.AddDimensionWithVehicleCapacity(NewPermanentCallback(&data, &TSPTWDataDT::Quantity, NewPermanentCallback(&routing, &RoutingModel::NodeToIndex), i), LLONG_MAX, capacities, false, "quantity" + std::to_string(i));
  }

  Solver *solver = routing.solver();

  int64 v = 0;
  int64 min_start = CUSTOM_MAX_INT;
  std::vector<IntVar*> used_vehicles;
  for(TSPTWDataDT::Vehicle* vehicle: data.Vehicles()) {
    // Vehicle costs
    int64 without_wait_cost = vehicle->cost_time_multiplier - vehicle->cost_waiting_time_multiplier;
    routing.GetMutableDimension("time")->SetSpanCostCoefficientForVehicle((int64)std::max(vehicle->cost_time_multiplier - without_wait_cost, (int64)0), v);
    routing.GetMutableDimension("time_without_wait")->SetSpanCostCoefficientForVehicle((int64)std::max(without_wait_cost, (int64)0), v);
    routing.GetMutableDimension("distance")->SetSpanCostCoefficientForVehicle(vehicle->cost_distance_multiplier, v);
    routing.SetFixedCostOfVehicle(vehicle->cost_fixed, v);
    if (FLAGS_nearby) {
      routing.GetMutableDimension("time_order")->SetSpanCostCoefficientForVehicle(vehicle->cost_time_multiplier / 5, v);
      routing.GetMutableDimension("distance_order")->SetSpanCostCoefficientForVehicle(vehicle->cost_distance_multiplier / 5, v);
    }

    int64 start_index = routing.Start(v);
    int64 end_index = routing.End(v);

    IntVar *const is_vehicle_used = solver->MakeConditionalExpression(solver->MakeIsDifferentCstVar(routing.NextVar(start_index), end_index),
      solver->MakeIntConst(1), 0)->Var();
    used_vehicles.push_back(is_vehicle_used);
    // Vehicle time windows
    if (vehicle->time_start > -CUSTOM_MAX_INT) {
      IntVar *const cumul_var = routing.CumulVar(start_index, "time");
      min_start = std::min(min_start, vehicle->time_start);
      cumul_var->SetMin(vehicle->time_start);
      if (vehicle->force_start) {
        cumul_var->SetMax(vehicle->time_start);
      }
    }
    if (vehicle->time_end < CUSTOM_MAX_INT) {
      int64 coef = vehicle->late_multiplier;
      if(coef > 0) {
        routing.GetMutableDimension("time")->SetEndCumulVarSoftUpperBound(v, vehicle->time_end, coef);
      } else {
        IntVar *const cumul_var = routing.CumulVar(end_index, "time");
        cumul_var->SetMax(vehicle->time_end);
      }
    }
    if (vehicle->duration >= 0 && vehicle->time_end - vehicle-> time_start > vehicle->duration) {
      has_route_duration = true;
      solver->AddConstraint(solver->MakeGreaterOrEqual(solver->MakeSum(routing.CumulVar(routing.Start(v), "time"), vehicle->duration), routing.CumulVar(routing.End(v), "time")));
    }

    for (int64 i = 0; i < vehicle->capacity.size(); ++i) {
      int64 coef = vehicle->overload_multiplier[i];
      if(vehicle->capacity[i] >= 0) {
        if(coef > 0) {
          routing.GetMutableDimension("quantity" + std::to_string(i))->SetEndCumulVarSoftUpperBound(v, vehicle->capacity[i], coef);
        } else {
          IntVar *const cumul_var = routing.CumulVar(end_index, "quantity" + std::to_string(i));
          cumul_var->SetMax(vehicle->capacity[i]);
        }
      }
    }
    ++v;
  }
  if (FLAGS_vehicle_limit > 0) {
    solver->AddConstraint(solver->MakeLessOrEqual(solver->MakeSum(used_vehicles), (int64)FLAGS_vehicle_limit));
  }

  // Setting solve parameters indicators
  int route_nbr = 0;
  int64 previous_distance_depot_start;
  int64 previous_distance_depot_end;
  bool force_start = false;
  bool loop_route = true;
  bool unique_configuration = true;
  RoutingModel::NodeIndex compareNodeIndex = routing.IndexToNode(rand() % (data.SizeMatrix() - 2));
  TSPTWDataDT::Vehicle* previous_vehicle = NULL;
  for (int route_nbr = 0; route_nbr < routing.vehicles(); route_nbr++) {
    TSPTWDataDT::Vehicle* vehicle = data.Vehicles().at(route_nbr);
    RoutingModel::NodeIndex nodeIndexStart = routing.IndexToNode(routing.Start(route_nbr));
    RoutingModel::NodeIndex nodeIndexEnd = routing.IndexToNode(routing.End(route_nbr));

    int64 distance_depot_start = std::max(vehicle->Time(nodeIndexStart, compareNodeIndex), vehicle->Distance(nodeIndexStart, compareNodeIndex));
    int64 distance_depot_end = std::max(vehicle->Time(compareNodeIndex, nodeIndexEnd), vehicle->Distance(compareNodeIndex, nodeIndexEnd));
    int64 distance_start_end = std::max(vehicle->Time(nodeIndexStart, nodeIndexEnd), vehicle->Distance(nodeIndexStart, nodeIndexEnd));

    if (previous_vehicle != NULL) {
      if (previous_distance_depot_start != distance_depot_start || previous_distance_depot_end != distance_depot_end) {
        unique_configuration = false;
      }
      if (distance_start_end != 0 || distance_depot_start == 0 && distance_depot_end == 0) {
        loop_route = false;
      }
    }
    force_start |= vehicle->force_start;
    previous_distance_depot_start = distance_depot_start;
    previous_distance_depot_end = distance_depot_end;
    previous_vehicle = vehicle;
  }

  // Setting visit time windows
  TWBuilder(data, routing, solver, size - 2, min_start, loop_route, unique_configuration);
  std::vector<IntVar*> breaks;
  // Setting rest time windows
  if (size_rest > 0) {
    breaks = RestBuilder(data, routing, solver, size);
  }
  RelationBuilder(data, routing, solver, size);
  RoutingSearchParameters parameters = BuildSearchParametersFromFlags();

  // Search strategy
  // parameters.set_first_solution_strategy(FirstSolutionStrategy::FIRST_UNBOUND_MIN_VALUE); // Default
  // parameters.set_first_solution_strategy(FirstSolutionStrategy::PATH_CHEAPEST_ARC);
  // parameters.set_first_solution_strategy(FirstSolutionStrategy::PATH_MOST_CONSTRAINED_ARC);
  // parameters.set_first_solution_strategy(FirstSolutionStrategy::CHRISTOFIDES);
  if (FLAGS_debug) std::cout << "First solution strategy : ";
  if (force_start) {
    if (FLAGS_debug) std::cout << "Path Cheapest Arc" << std::endl;
    parameters.set_first_solution_strategy(FirstSolutionStrategy::PATH_CHEAPEST_ARC);
  } else if (has_route_duration && size_vehicles == 1) {
    if (FLAGS_debug) std::cout << "Global Cheapest Arc" << std::endl;
    parameters.set_first_solution_strategy(FirstSolutionStrategy::GLOBAL_CHEAPEST_ARC);
  } else if (data.DeliveriesCounter() > 0 || (float)size_mtws/size > 0.2) {
    if (FLAGS_debug) std::cout << "Local Cheapest Insertion" << std::endl;
    parameters.set_first_solution_strategy(FirstSolutionStrategy::LOCAL_CHEAPEST_INSERTION);
  } else if (size_rest == 0 && loop_route && unique_configuration && size_vehicles < 10 && !has_route_duration) {
    if (FLAGS_debug) std::cout << "Savings" << std::endl;
    parameters.set_first_solution_strategy(FirstSolutionStrategy::SAVINGS);
  } else if (unique_configuration || loop_route) {
    if (FLAGS_debug) std::cout << "Paralell Cheapest Insertion" << std::endl;
    parameters.set_first_solution_strategy(FirstSolutionStrategy::PARALLEL_CHEAPEST_INSERTION);
  } else {
    if (FLAGS_debug) std::cout << "Default" << std::endl;
  }
  // parameters.set_first_solution_strategy(FirstSolutionStrategy::ALL_UNPERFORMED);
  // parameters.set_first_solution_strategy(FirstSolutionStrategy::BEST_INSERTION);
  // parameters.set_first_solution_strategy(FirstSolutionStrategy::ROUTING_BEST_INSERTION);
  // parameters.set_first_solution_strategy(FirstSolutionStrategy::PARALLEL_CHEAPEST_INSERTION);
  // parameters.set_first_solution_strategy(FirstSolutionStrategy::LOCAL_CHEAPEST_INSERTION);
  // parameters.set_first_solution_strategy(FirstSolutionStrategy::GLOBAL_CHEAPEST_ARC);
  // parameters.set_first_solution_strategy(FirstSolutionStrategy::LOCAL_CHEAPEST_ARC);
  // parameters.set_first_solution_strategy(FirstSolutionStrategy::ROUTING_BEST_INSERTION);

  // parameters.set_local_search_metaheuristic(LocalSearchMetaheuristic::GREEDY_DESCENT);
  parameters.set_local_search_metaheuristic(LocalSearchMetaheuristic::GUIDED_LOCAL_SEARCH);
  // parameters.set_guided_local_search_lambda_coefficient(0.5);
  // parameters.set_local_search_metaheuristic(LocalSearchMetaheuristic::SIMULATED_ANNEALING);
  // parameters.set_local_search_metaheuristic(LocalSearchMetaheuristic::TABU_SEARCH);

  if (FLAGS_time_limit_in_ms > 0) {
    parameters.set_time_limit_ms(FLAGS_time_limit_in_ms);
  }
  routing.CloseModelWithParameters(parameters);

  LoggerMonitor * const logger = MakeLoggerMonitor(data, &routing, min_start, size_matrix, breaks, FLAGS_debug, true);
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
    int64 cost = (int64)(solution->ObjectiveValue() / 1000.0); // Back to original cost
    logger->GetFinalLog();
    int current_break = 0;
    for (int route_nbr = 0; route_nbr < routing.vehicles(); route_nbr++) {
      int previous_index = -1;
      for (int64 index = routing.Start(route_nbr); !routing.IsEnd(index); index = solution->Value(routing.NextVar(index))) {
        RoutingModel::NodeIndex nodeIndex = routing.IndexToNode(index);
        std::cout << data.MatrixIndex(nodeIndex);
        if (previous_index != -1)
          std::cout << "[" << solution->Min(routing.GetMutableDimension("time")->CumulVar(index)) << "]";
        std::cout << ",";
        if (current_break < data.Rests().size() && data.Vehicles().at(route_nbr)->break_size > 0 && solution->Value(breaks[current_break]) == index) {
          std::cout << size_matrix + current_break << ",";
          current_break++;
        }
        previous_index = index;
      }
      if (current_break < data.Rests().size() && data.Vehicles().at(route_nbr)->break_size > 0 && solution->Value(breaks[current_break]) == routing.End(route_nbr)) {
          std::cout << size_matrix + current_break << ",";
          current_break++;
      }
      std::cout << data.MatrixIndex(routing.IndexToNode(routing.End(route_nbr))) << ";";
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
