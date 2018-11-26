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

#include "ortools/base/commandlineflags.h"
#include "ortools/constraint_solver/routing.h"
#include "ortools/base/join.h"
#include "ortools/base/timer.h"
#include <ortools/base/callback.h>

#include "tsptw_data_dt.h"
#include "limits.h"

#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_flags.h"

DEFINE_int64(time_limit_in_ms, 0, "Time limit in ms, no option means no limit.");
DEFINE_int64(no_solution_improvement_limit, -1,"Iterations whitout improvement");
DEFINE_int64(initial_time_out_no_solution_improvement, -1, "Initial time whitout improvement in ms");
DEFINE_int64(time_out_multiplier, 2, "Multiplier for the nexts time out");
DEFINE_int64(vehicle_limit, 0, "Define the maximum number of vehicle");
DEFINE_int64(solver_parameter, -1, "Force a particular behavior");
DEFINE_bool(balance, false, "Route balancing");
DEFINE_bool(nearby, false, "Short segment priority");
DEFINE_bool(debug, false, "debug display");
DEFINE_bool(intermediate_solutions, false, "display intermediate solutions");


namespace operations_research {

bool CheckOverflow(int64 a, int64 b) {
  if ( a > std::pow(2, 52) / b)
    return true;
  return false;
}

void MissionsBuilder(const TSPTWDataDT &data, RoutingModel &routing, Solver *solver, int64 size, int64 min_start, bool loop_route, bool unique_configuration) {
  const int size_vehicles = data.Vehicles().size();
  const int size_matrix = data.SizeMatrix();
  const int size_rests = data.SizeRest();
  const int size_problem = data.SizeProblem();

  int64 max_time = (2 * data.MaxTime() + data.MaxServiceTime()) * data.MaxTimeCost();
  int64 max_distance = 2 * data.MaxDistance() * data.MaxDistanceCost();
  int64 max_value = 2 * data.MaxValue() * data.MaxValueCost();

  bool overflow_danger = CheckOverflow(max_time + max_distance + max_value, size_vehicles);
  int64 data_verif = (max_time + max_distance + max_value) * size_vehicles;

  overflow_danger = overflow_danger || CheckOverflow(data_verif, std::pow(2,4) * size);
  data_verif = data_verif * std::pow(2,4) * size;
  RoutingModel::NodeIndex i(0);
  int32 tw_index = 0;
  int64 disjunction_cost = !overflow_danger && !CheckOverflow(data_verif, size)? data_verif : std::pow(2, 52);

  for (int activity = 0; activity <= size_problem + size_rests; ++activity) {
    std::vector<RoutingModel::NodeIndex> *vect = new std::vector<RoutingModel::NodeIndex>();
    int32 alternative_size = data.AlternativeSize(activity);

    int64 priority = data.Priority(i);
    int64 exclusion_cost = data.ExclusionCost(i);

    for (int alternative = 0; alternative < alternative_size; ++alternative) {
      vect->push_back(i);
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
            if (due.size() > 1) {
              for (tw_index = due.size() - 1; tw_index--; ) {
                cumul_var->RemoveInterval(due.at(tw_index), ready.at(tw_index + 1));
              }
            }
          }
        }
      }

      if (sticky_vehicle.size() > 0) {
        std::vector<int64> vehicle_indices;
        std::vector<int64> vehicle_intersection;
        std::vector<int64> vehicle_difference;
        for (int v = 0; v < size_vehicles; ++v) vehicle_indices.push_back(v);
        std::set_intersection(vehicle_indices.begin(), vehicle_indices.end(), sticky_vehicle.begin(), sticky_vehicle.end(), std::back_inserter(vehicle_intersection));
        std::set_difference(vehicle_indices.begin(), vehicle_indices.end(), vehicle_intersection.begin(), vehicle_intersection.end(), std::back_inserter(vehicle_difference));
        if (vehicle_difference.size() > 0) {
          for (int64 remove : vehicle_difference) routing.VehicleVar(index)->RemoveValue(remove);
        }
      }

      std::vector<bool> refill_quantities = data.RefillQuantities(i);
      for (int64 q = 0 ; q < data.Quantities(i).size(); ++q) {
        RoutingDimension* quantity_dimension = routing.GetMutableDimension("quantity" + std::to_string(q));
        if (!refill_quantities.at(q)) quantity_dimension->SlackVar(index)->SetValue(0);
        routing.AddVariableMinimizedByFinalizer(quantity_dimension->CumulVar(index));
      }

      ++i;
    }

    // Otherwise this single service is never assigned
    if (size == 1)
      routing.AddDisjunction(*vect);
    else
      routing.AddDisjunction(*vect, exclusion_cost == -1 ? disjunction_cost * std::pow(2, 4 - priority) : exclusion_cost);
  }
}

bool RouteBuilder(const TSPTWDataDT &data, RoutingModel &routing, Solver *solver, Assignment *assignment) {
  const int size_vehicles = data.Vehicles().size();
  std::vector<std::vector<RoutingModel::NodeIndex>> routes(size_vehicles);
  for (TSPTWDataDT::Route* route: data.Routes()) {
    int64 current_index;
    IntVar* previous_var = NULL;
    std::vector<RoutingModel::NodeIndex> route_nodes;

    if (route->vehicle_index >= 0) {
      previous_var = routing.NextVar(routing.Start(route->vehicle_index));
      assignment->Add(previous_var);
    }
    for (std::string service_id: route->service_ids) {
      current_index = data.IdIndex(service_id);
      if (current_index != -1) {
        route_nodes.push_back(RoutingModel::NodeIndex(current_index));
        IntVar* next_var = routing.NextVar(current_index);
        assignment->Add(next_var);
        if (previous_var != NULL) {
          assignment->SetValue(previous_var, current_index);
        }
        previous_var = next_var;
      }
    }
    if (route->vehicle_index >= 0) {
      if (previous_var != NULL) {
        assignment->SetValue(previous_var, routing.End(route->vehicle_index));
      }
      std::vector<RoutingModel::NodeIndex> actual_route = routes.at(route->vehicle_index);
      actual_route.insert(actual_route.end(), route_nodes.begin(), route_nodes.end());
      routes.at(route->vehicle_index) = actual_route;
    }
  }
  return routing.RoutesToAssignment(routes, true, false, assignment);
}

void RelationBuilder(const TSPTWDataDT &data, RoutingModel &routing, Solver *solver, int64 size, Assignment *assignment, bool &has_overall_duration) {

  const int size_vehicles = data.Vehicles().size();

  Solver::IndexEvaluator1 vehicle_evaluator = [&data](int64 index) {
    return data.VehicleDay(index);
  };

  Solver::IndexEvaluator1 day_to_vehicle_evaluator = [&data](int64 index) {
    return data.DayIndexToVehicleIndex(index);
  };

  Solver::IndexEvaluator1 alternative_vehicle_evaluator = [&data](int64 index) {
    return data.VehicleDayAlt(index);
  };

  for (TSPTWDataDT::Relation* relation: data.Relations()) {
    int64 previous_index;
    int64 current_index;
    std::vector<int64> previous_indices;
    switch (relation->type) {
      case Sequence:
        int64 new_current_index;
        previous_index = data.IdIndex(relation->linked_ids->at(0));
        for (int link_index = 1 ; link_index < relation->linked_ids->size(); ++link_index) {
          current_index = data.IdIndex(relation->linked_ids->at(link_index));
          IntVar *const previous_active_var = routing.ActiveVar(previous_index);
          IntVar *const active_var = routing.ActiveVar(current_index);
          solver->AddConstraint(solver->MakeLessOrEqual(active_var, previous_active_var));
          routing.AddPickupAndDelivery(RoutingModel::NodeIndex(previous_index), RoutingModel::NodeIndex(current_index));

          IntVar *const previous_vehicle_var = routing.VehicleVar(previous_index);
          IntVar *const vehicle_var = routing.VehicleVar(current_index);
          IntExpr *const isConstraintActive = solver->MakeProd(previous_active_var, active_var)->Var();
          routing.NextVar(current_index)->RemoveValues(previous_indices);

          solver->AddConstraint(solver->MakeEquality(solver->MakeProd(isConstraintActive, previous_vehicle_var),
                                                     solver->MakeProd(isConstraintActive, vehicle_var)));
          solver->AddConstraint(solver->MakeEquality(solver->MakeProd(isConstraintActive, routing.NextVar(previous_index)),
            solver->MakeProd(isConstraintActive, current_index)));
          previous_indices.push_back(previous_index);
          previous_index = data.IdIndex(relation->linked_ids->at(link_index));
        }
        break;
      case Order:
        previous_index = data.IdIndex(relation->linked_ids->at(0));
        for (int link_index = 1 ; link_index < relation->linked_ids->size(); ++link_index) {
          current_index = data.IdIndex(relation->linked_ids->at(link_index));
          routing.AddPickupAndDelivery(RoutingModel::NodeIndex(previous_index), RoutingModel::NodeIndex(current_index));
          IntVar *const previous_active_var = routing.ActiveVar(previous_index);
          IntVar *const active_var = routing.ActiveVar(current_index);

          IntVar *const previous_vehicle_var = routing.VehicleVar(previous_index);
          IntVar *const vehicle_var = routing.VehicleVar(current_index);
          routing.NextVar(current_index)->RemoveValues(previous_indices);

          solver->AddConstraint(solver->MakeLessOrEqual(active_var, previous_active_var));
          IntExpr *const isConstraintActive = solver->MakeProd(previous_active_var, active_var);

          solver->AddConstraint(solver->MakeEquality(solver->MakeProd(isConstraintActive, previous_vehicle_var),
                                                     solver->MakeProd(isConstraintActive, vehicle_var)));

          solver->AddConstraint(
            solver->MakeLessOrEqual(routing.GetMutableDimension("time")->CumulVar(previous_index),
                                    routing.GetMutableDimension("time")->CumulVar(current_index)));
          previous_indices.push_back(previous_index);
          previous_index = current_index;

        }
        break;
      case SameRoute:
        previous_index = data.IdIndex(relation->linked_ids->at(0));
        for (int link_index = 1 ; link_index < relation->linked_ids->size(); ++link_index) {
          current_index = data.IdIndex(relation->linked_ids->at(link_index));
          IntVar *const previous_active_var = routing.ActiveVar(previous_index);
          IntVar *const active_var = routing.ActiveVar(current_index);

          solver->AddConstraint(solver->MakeLessOrEqual(active_var, previous_active_var));
          IntVar *const previous_vehicle_var = routing.VehicleVar(previous_index);
          IntVar *const vehicle_var = routing.VehicleVar(current_index);

          solver->AddConstraint(solver->MakeLessOrEqual(active_var, previous_active_var));
          IntVar *const isConstraintActive = solver->MakeProd(previous_active_var, active_var)->Var();
          solver->AddConstraint(solver->MakeEquality(solver->MakeProd(previous_vehicle_var, isConstraintActive),
                                                     solver->MakeProd(vehicle_var, isConstraintActive)));
          previous_index = current_index;
        }
        break;
      case MinimumDayLapse:
        previous_index = data.IdIndex(relation->linked_ids->at(0));
        for (int link_index = 1 ; link_index < relation->linked_ids->size(); ++link_index) {
          current_index = data.IdIndex(relation->linked_ids->at(link_index));
          IntVar *const previous_active_var = routing.ActiveVar(previous_index);
          IntVar *const active_var = routing.ActiveVar(current_index);
          IntVar *const vehicle_var = routing.VehicleVar(current_index);

          IntVar *const previous_part = solver->MakeElement(vehicle_evaluator, routing.VehicleVar(previous_index))->Var();

          IntVar *const vehicle_index_var = solver->MakeElement(day_to_vehicle_evaluator, solver->MakeSum(previous_part, relation->lapse)->Var())->Var();
          solver->AddConstraint(solver->MakeLessOrEqual(active_var, previous_active_var));
          IntExpr *const isConstraintActive = solver->MakeProd(previous_active_var, active_var);

          solver->AddConstraint(solver->MakeGreaterOrEqual(solver->MakeProd(vehicle_var, isConstraintActive), solver->MakeProd(vehicle_index_var, isConstraintActive)));
          previous_index = current_index;
        }
        break;
      case MaximumDayLapse:
        previous_index = data.IdIndex(relation->linked_ids->at(0));
        for (int link_index = 1 ; link_index < relation->linked_ids->size(); ++link_index) {
          current_index = data.IdIndex(relation->linked_ids->at(link_index));
          IntVar *const previous_active_var = routing.ActiveVar(previous_index);
          IntVar *const active_var = routing.ActiveVar(current_index);
          IntVar *const vehicle_var = routing.VehicleVar(current_index);
          IntVar *const previous_vehicle_var = routing.VehicleVar(previous_index);

          IntVar *const previous_part = solver->MakeElement(vehicle_evaluator, routing.VehicleVar(previous_index))->Var();

          IntVar *const vehicle_index_var = solver->MakeElement(day_to_vehicle_evaluator, solver->MakeSum(previous_part, relation->lapse)->Var())->Var();
          solver->AddConstraint(solver->MakeLessOrEqual(active_var, previous_active_var));

          IntExpr *const isConstraintActive = solver->MakeProd(previous_active_var, active_var);
          solver->AddConstraint(solver->MakeGreaterOrEqual(solver->MakeProd(vehicle_var, isConstraintActive), solver->MakeProd(previous_vehicle_var, isConstraintActive)));
          solver->AddConstraint(solver->MakeLessOrEqual(solver->MakeProd(vehicle_var, isConstraintActive), solver->MakeProd(vehicle_index_var, isConstraintActive)));
          previous_index = current_index;
        }
        break;
      case Shipment:
        previous_index = data.IdIndex(relation->linked_ids->at(0));
        for (int link_index = 1 ; link_index < relation->linked_ids->size(); ++link_index) {
          current_index = data.IdIndex(relation->linked_ids->at(link_index));
          routing.AddPickupAndDelivery(RoutingModel::NodeIndex(previous_index), RoutingModel::NodeIndex(current_index));
          solver->AddConstraint(solver->MakeEquality(
            routing.VehicleVar(previous_index),
            routing.VehicleVar(current_index)));
          solver->AddConstraint(
            solver->MakeLessOrEqual(routing.GetMutableDimension("time")->CumulVar(previous_index),
                                    routing.GetMutableDimension("time")->CumulVar(current_index)));
          previous_index = current_index;
        }
        break;
      case MeetUp:
        previous_index = data.IdIndex(relation->linked_ids->at(0));
        for (int link_index = 1 ; link_index < relation->linked_ids->size(); ++link_index) {
          current_index = data.IdIndex(relation->linked_ids->at(link_index));
          IntVar *const previous_active_var = routing.ActiveVar(previous_index);
          IntVar *const active_var = routing.ActiveVar(current_index);
          IntExpr *const isConstraintActive = solver->MakeProd(previous_active_var, active_var)->Var();

          IntExpr *const previous_part = solver->MakeProd(isConstraintActive, routing.VehicleVar(previous_index));
          IntExpr *const next_part = solver->MakeProd(isConstraintActive, routing.VehicleVar(current_index));

          IntExpr *const lapse = solver->MakeProd(isConstraintActive, 1);
          solver->AddConstraint(solver->MakeGreaterOrEqual(solver->MakeAbs(solver->MakeDifference(next_part, previous_part)), lapse));

          IntExpr *const previous_part_global_index = solver->MakeElement(vehicle_evaluator, routing.VehicleVar(previous_index));
          IntExpr *const next_part_global_index = solver->MakeElement(vehicle_evaluator, routing.VehicleVar(current_index));

          solver->AddConstraint(solver->MakeEquality(previous_part_global_index, next_part_global_index));

          solver->AddConstraint(
            solver->MakeEquality(solver->MakeProd(isConstraintActive,routing.GetMutableDimension("time")->CumulVar(previous_index)),
                                    solver->MakeProd(isConstraintActive,routing.GetMutableDimension("time")->CumulVar(current_index))));
          previous_index = current_index;
        }
        break;
      case MaximumDurationLapse:
        previous_index = data.IdIndex(relation->linked_ids->at(0));
        for (int link_index = 1 ; link_index < relation->linked_ids->size(); ++link_index) {
          current_index = data.IdIndex(relation->linked_ids->at(link_index));
          IntVar *const previous_active_var = routing.ActiveVar(previous_index);
          IntVar *const active_var = routing.ActiveVar(current_index);

          solver->AddConstraint(solver->MakeLessOrEqual(active_var, previous_active_var));
          IntExpr *const isConstraintActive = solver->MakeProd(previous_active_var, active_var)->Var();
          IntExpr *const difference = solver->MakeDifference(routing.GetMutableDimension("time")->CumulVar(current_index), routing.GetMutableDimension("time")->CumulVar(previous_index));

          IntExpr *const lapse = solver->MakeProd(isConstraintActive, relation->lapse);
          solver->AddConstraint(solver->MakeLessOrEqual(solver->MakeProd(isConstraintActive, difference), lapse));
          previous_index = current_index;
        }
        break;
      case NeverFirst:
        for (int link_index = 0 ; link_index < relation->linked_ids->size(); ++link_index) {
          current_index = data.IdIndex(relation->linked_ids->at(link_index));
          for (int v = 0; v < data.Vehicles().size(); ++v) {
            int64 start_index = routing.Start(v);
            int64 end_index = routing.End(v);
            IntVar *const next_var = routing.NextVar(start_index);
            next_var->RemoveValue(current_index);
          }
        }
        break;
      case ForceFirst:
        {
          std::vector<int64> values;
          for (int link_index = 0 ; link_index < relation->linked_ids->size(); ++link_index) {
            current_index = data.IdIndex(relation->linked_ids->at(link_index));
            values.push_back(current_index);
          }
          for (int v = 0; v < data.Vehicles().size(); ++v) {
            int64 start_index = routing.Start(v);
            int64 end_index = routing.End(v);
            IntVar *const next_var = routing.NextVar(start_index);
            values.push_back(end_index);
            next_var->SetValues(values);
            values.pop_back();
          }
        }
        break;
      case ForceLast:
        {
          std::vector<int64> values;
          for (int link_index = 0 ; link_index < relation->linked_ids->size(); ++link_index) {
            current_index = data.IdIndex(relation->linked_ids->at(link_index));
            values.push_back(current_index);
          }
          std::vector<int64> intermediate_values(values);
          for (int v = 0; v < data.Vehicles().size(); ++v) {
            int64 end_index = routing.End(v);
            intermediate_values.push_back(end_index);
          }
          for (int index : values) {
            IntVar *const next_var = routing.NextVar(index);
            next_var->SetValues(intermediate_values);
          }
        }
        break;
      case VehicleGroupDuration:
        {
          if (relation->lapse > -1){
            has_overall_duration = true;
            std::vector<IntVar*> same_vehicle_vars;
            for (int link_index = 0 ; link_index < relation->linked_vehicle_ids->size(); ++link_index) {
              current_index = data.VehicleIdIndex(relation->linked_vehicle_ids->at(link_index));
              int64 start_index = routing.Start(current_index);
              int64 end_index = routing.End(current_index);
              IntVar *const cumul_var = routing.CumulVar(start_index, "time");
              IntVar *const end_cumul_var = routing.CumulVar(end_index, "time");
              IntVar *const vehicle_time = solver->MakeDifference(end_cumul_var, cumul_var)->Var();
              same_vehicle_vars.push_back(vehicle_time);
            }
            solver->AddConstraint(solver->MakeLessOrEqual(solver->MakeSum(same_vehicle_vars), relation->lapse));
          }
        }
        break;
      default:
        break;
    }
  }
}

int TSPTWSolver(const TSPTWDataDT &data, std::string filename) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ortools_result::Result result;

  const int size_vehicles = data.Vehicles().size();
  const int size = data.Size();
  const int size_missions = data.SizeMissions();
  const int size_matrix = data.SizeMatrix();
  const int size_rest = data.SizeRest();
  const int size_mtws = data.TwiceTWsCounter();
  bool has_lateness = false;
  bool has_route_duration = false;
  bool has_overall_duration = false;
  bool free_approach_return = false;

  for (TSPTWDataDT::Vehicle* vehicle: data.Vehicles()) {
    if (vehicle->free_approach == true || vehicle->free_return == true) {
      free_approach_return = true;
    }
  }

  std::vector<std::pair<RoutingModel::NodeIndex, RoutingModel::NodeIndex>> *start_ends = new std::vector<std::pair<RoutingModel::NodeIndex, RoutingModel::NodeIndex>>(size_vehicles);
  for(int v = 0; v < size_vehicles; ++v) {
    (*start_ends)[v] = std::make_pair(data.Vehicles().at(v)->start, data.Vehicles().at(v)->stop);
    has_lateness |= data.Vehicles().at(v)->late_multiplier > 0;
  }
  RoutingModel routing(size, size_vehicles, *start_ends);

  int64 maximum_route_distance = 0;
  int64 v = 0;
  while ((maximum_route_distance != INT_MAX) && (v<size_vehicles)) {
    if (data.Vehicles().at(v)->distance == -1)
      maximum_route_distance = INT_MAX;
    else
      maximum_route_distance = std::max(maximum_route_distance, data.Vehicles().at(v)->distance);
    v++;
  }

  // Dimensions
  const int64 horizon = data.Horizon() * (has_lateness && !CheckOverflow(data.Horizon(), 2) ? 2 : 1);

  std::vector<ResultCallback2<long long int, IntType<operations_research::RoutingNodeIndex_tag_, int>, IntType<operations_research::RoutingNodeIndex_tag_, int> >*> zero_evaluators;
  std::vector<ResultCallback2<long long int, IntType<operations_research::RoutingNodeIndex_tag_, int>, IntType<operations_research::RoutingNodeIndex_tag_, int> >*> time_evaluators;
  std::vector<ResultCallback2<long long int, IntType<operations_research::RoutingNodeIndex_tag_, int>, IntType<operations_research::RoutingNodeIndex_tag_, int> >*> fake_time_evaluators;
  std::vector<ResultCallback2<long long int, IntType<operations_research::RoutingNodeIndex_tag_, int>, IntType<operations_research::RoutingNodeIndex_tag_, int> >*> distance_evaluators;
  std::vector<ResultCallback2<long long int, IntType<operations_research::RoutingNodeIndex_tag_, int>, IntType<operations_research::RoutingNodeIndex_tag_, int> >*> value_evaluators;
  std::vector<ResultCallback2<long long int, IntType<operations_research::RoutingNodeIndex_tag_, int>, IntType<operations_research::RoutingNodeIndex_tag_, int> >*> time_order_evaluators;
  std::vector<ResultCallback2<long long int, IntType<operations_research::RoutingNodeIndex_tag_, int>, IntType<operations_research::RoutingNodeIndex_tag_, int> >*> distance_order_evaluators;
  for (TSPTWDataDT::Vehicle* vehicle: data.Vehicles()) {
    zero_evaluators.push_back(NewPermanentCallback(vehicle, &TSPTWDataDT::Vehicle::ReturnZero));
    time_evaluators.push_back(NewPermanentCallback(vehicle, &TSPTWDataDT::Vehicle::TimePlusServiceTime));
    if (vehicle->free_approach == true || vehicle->free_return == true) {
      fake_time_evaluators.push_back(NewPermanentCallback(vehicle, &TSPTWDataDT::Vehicle::FakeTimePlusServiceTime));
    }
    distance_evaluators.push_back(NewPermanentCallback(vehicle, &TSPTWDataDT::Vehicle::Distance));
    value_evaluators.push_back(NewPermanentCallback(vehicle, &TSPTWDataDT::Vehicle::ValuePlusServiceValue));
    if (FLAGS_nearby) {
      time_order_evaluators.push_back(NewPermanentCallback(vehicle, &TSPTWDataDT::Vehicle::TimeOrder));
      distance_order_evaluators.push_back(NewPermanentCallback(vehicle, &TSPTWDataDT::Vehicle::DistanceOrder));
    }
  }

  if (FLAGS_balance) {
    routing.AddDimensionWithVehicleTransits(zero_evaluators, horizon, horizon, true, "time_balance");
    routing.AddDimensionWithVehicleTransits(zero_evaluators, LLONG_MAX, LLONG_MAX, true, "distance_balance");
  }
  routing.AddDimensionWithVehicleTransits(time_evaluators, horizon, horizon, false, "time");

  if (free_approach_return == true) {
    routing.AddDimensionWithVehicleTransits(fake_time_evaluators, horizon, horizon, false, "fake_time");
  }
  routing.AddDimensionWithVehicleTransits(time_evaluators, 0, horizon, false, "time_without_wait");
  routing.AddDimensionWithVehicleTransits(distance_evaluators, 0,  maximum_route_distance, true, "distance");
  routing.AddDimensionWithVehicleTransits(value_evaluators, 0, LLONG_MAX, true, "value");
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
  Assignment *assignment = routing.solver()->MakeAssignment();

  v = 0;
  int64 min_start = CUSTOM_MAX_INT;
  std::vector<IntVar*> used_vehicles;
  std::vector<IntVar*> shift_vars;
  std::vector<IntVar*> ends_distance_vars;

  for(TSPTWDataDT::Vehicle* vehicle: data.Vehicles()) {
    // Vehicle costs
    int64 without_wait_cost = vehicle->cost_time_multiplier - vehicle->cost_waiting_time_multiplier;
    if (FLAGS_balance) {
      routing.GetMutableDimension("time_balance")->SetSpanCostCoefficientForVehicle((int64)vehicle->cost_time_multiplier, v);
      routing.GetMutableDimension("distance_balance")->SetSpanCostCoefficientForVehicle(vehicle->cost_distance_multiplier, v);
    }
    if (vehicle->free_approach == true || vehicle->free_return == true) {
      routing.GetMutableDimension("fake_time")->SetSpanCostCoefficientForVehicle((int64)vehicle->cost_waiting_time_multiplier, v);
    } else {
      routing.GetMutableDimension("time")->SetSpanCostCoefficientForVehicle((int64)vehicle->cost_waiting_time_multiplier, v);
    }
    vehicle->routing = &routing;
    routing.GetMutableDimension("time_without_wait")->SetSpanCostCoefficientForVehicle((int64)std::max(without_wait_cost, (int64)0), v);
    routing.GetMutableDimension("distance")->SetSpanCostCoefficientForVehicle(vehicle->cost_distance_multiplier, v);
    routing.GetMutableDimension("value")->SetSpanCostCoefficientForVehicle(vehicle->cost_value_multiplier, v);
    routing.SetFixedCostOfVehicle(vehicle->cost_fixed, v);
    if (FLAGS_nearby) {
      routing.GetMutableDimension("time_order")->SetSpanCostCoefficientForVehicle(vehicle->cost_time_multiplier / 5, v);
      routing.GetMutableDimension("distance_order")->SetSpanCostCoefficientForVehicle(vehicle->cost_distance_multiplier / 5, v);
    }

    int64 start_index = routing.Start(v);
    int64 end_index = routing.End(v);
    IntVar *const cumul_var = routing.CumulVar(start_index, "time");
    IntVar *const end_cumul_var = routing.CumulVar(end_index, "time");

    IntVar *const is_vehicle_used = solver->MakeConditionalExpression(solver->MakeIsDifferentCstVar(routing.NextVar(start_index), end_index),
      solver->MakeIntConst(1), 0)->Var();
    used_vehicles.push_back(is_vehicle_used);
    IntVar *const time_difference = solver->MakeDifference(end_cumul_var, cumul_var)->Var();
    shift_vars.push_back(time_difference);
    IntVar *const dist_end_cumul_var = routing.CumulVar(end_index, "distance");
    ends_distance_vars.push_back(dist_end_cumul_var);
    // Vehicle maximum distance
    if (vehicle->distance > 0){
      // routing.AddVariableMinimizedByFinalizer(dist_end_cumul_var);
      solver->AddConstraint(solver->MakeLessOrEqual(dist_end_cumul_var, vehicle->distance));
    }
    // Vehicle time windows
    if (vehicle->time_start > -CUSTOM_MAX_INT) {
      min_start = std::min(min_start, vehicle->time_start);
      cumul_var->SetMin(vehicle->time_start);
      if (vehicle->shift_preference == ForceStart) {
        routing.GetMutableDimension("time")->SetStartCumulVarSoftUpperBound(v, vehicle->time_start, (int64)vehicle->cost_time_multiplier * 100);
        IntVar *const slack_var = routing.SlackVar(start_index, "time");
        routing.AddVariableMinimizedByFinalizer(slack_var);
      }
    }
    if (vehicle->time_end < CUSTOM_MAX_INT) {
      int64 coef = vehicle->late_multiplier;
      if(coef > 0) {
        routing.GetMutableDimension("time")->SetEndCumulVarSoftUpperBound(v, vehicle->time_end, coef);
        if (vehicle->shift_preference == ForceEnd) {
          routing.AddVariableMaximizedByFinalizer(end_cumul_var);
        }
      } else {
        end_cumul_var->SetMax(vehicle->time_end);
        if (vehicle->shift_preference == ForceEnd) {
          end_cumul_var->SetMin(vehicle->time_end);
          IntVar *const slack_var = routing.SlackVar(end_index, "time");
          routing.AddVariableMinimizedByFinalizer(slack_var);
        }
      }
    }
    if (vehicle->duration >= 0 && vehicle->time_end - vehicle-> time_start > vehicle->duration) {
      has_route_duration = true;
      routing.AddVariableMinimizedByFinalizer(end_cumul_var);
      if (vehicle->shift_preference == ForceStart)
        routing.AddVariableMinimizedByFinalizer(cumul_var);
      else
        routing.AddVariableMaximizedByFinalizer(cumul_var);
      solver->AddConstraint(solver->MakeGreaterOrEqual(solver->MakeSum(cumul_var, vehicle->duration), end_cumul_var));
    } else if (FLAGS_balance) {
      routing.AddVariableMinimizedByFinalizer(end_cumul_var);
      routing.AddVariableMaximizedByFinalizer(cumul_var);
    } else {
      if (vehicle->free_approach == true)
        routing.AddVariableMaximizedByFinalizer(cumul_var);
      if (vehicle->free_return == true)
        routing.AddVariableMaximizedByFinalizer(end_cumul_var);
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
    if (vehicle->free_approach == true) {
        int64 start_index = routing.Start(v);
        IntVar *const start_cumul_var = routing.CumulVar(start_index, "fake_time");
        solver->AddConstraint(solver->MakeGreaterOrEqual(start_cumul_var, solver->MakeDifference(solver->MakeMax(shift_vars), solver->MakeMin(shift_vars))));
    }

    if (vehicle->free_return == true) {
        int64 end_index = routing.End(v);
        IntVar *const end_cumul_var = routing.CumulVar(end_index, "fake_time");
        solver->AddConstraint(solver->MakeGreaterOrEqual(end_cumul_var, solver->MakeDifference(solver->MakeMax(shift_vars), solver->MakeMin(shift_vars))));
    }
    ++v;
  }

  if (FLAGS_balance) {
    for(int v = 0; v < size_vehicles; ++v) {
      int64 start_index = routing.Start(v);
      int64 end_index = routing.End(v);

      IntVar *const start_time_cumul_var = routing.CumulVar(start_index, "time_balance");
      IntVar *const end_time_cumul_var = routing.CumulVar(end_index, "time_balance");
      start_time_cumul_var->SetMax(0);
      solver->AddConstraint(solver->MakeGreaterOrEqual(end_time_cumul_var, solver->MakeDifference(solver->MakeMax(shift_vars), solver->MakeMin(shift_vars))));

      IntVar *const start_distance_cumul_var = routing.CumulVar(start_index, "distance_balance");
      IntVar *const end_distance_cumul_var = routing.CumulVar(end_index, "distance_balance");
      start_distance_cumul_var->SetMax(0);
      solver->AddConstraint(solver->MakeGreaterOrEqual(end_distance_cumul_var, solver->MakeDifference(solver->MakeMax(ends_distance_vars), solver->MakeMin(ends_distance_vars))));
    }
  }

  if (FLAGS_vehicle_limit > 0) {
    solver->AddConstraint(solver->MakeLessOrEqual(solver->MakeSum(used_vehicles), (int64)FLAGS_vehicle_limit));
  }

  // Setting solve parameters indicators
  int route_nbr = 0;
  int64 previous_distance_depot_start;
  int64 previous_distance_depot_end;
  ShiftPref shift_preference = MinimizeSpan;
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
    shift_preference = vehicle->shift_preference;
    previous_distance_depot_start = distance_depot_start;
    previous_distance_depot_end = distance_depot_end;
    previous_vehicle = vehicle;
  }

  // Setting visit time windows
  MissionsBuilder(data, routing, solver, size - 2, min_start, loop_route, unique_configuration);
  RelationBuilder(data, routing, solver, size, assignment, has_overall_duration);
  RoutingSearchParameters parameters = BuildSearchParametersFromFlags();

  // Search strategy
  // parameters.set_first_solution_strategy(FirstSolutionStrategy::FIRST_UNBOUND_MIN_VALUE); // Default
  // parameters.set_first_solution_strategy(FirstSolutionStrategy::PATH_CHEAPEST_ARC);
  // parameters.set_first_solution_strategy(FirstSolutionStrategy::PATH_MOST_CONSTRAINED_ARC);
  // parameters.set_first_solution_strategy(FirstSolutionStrategy::CHRISTOFIDES);
  std::cout << "First solution strategy : ";
  switch (FLAGS_solver_parameter) {
  case 0:
    std::cout << "Path Cheapest Arc" << std::endl;
    parameters.set_first_solution_strategy(FirstSolutionStrategy::PATH_CHEAPEST_ARC);
    break;
  case 1:
    std::cout << "Global Cheapest Arc" << std::endl;
    parameters.set_first_solution_strategy(FirstSolutionStrategy::GLOBAL_CHEAPEST_ARC);
    break;
  case 2:
    std::cout << "Local Cheapest Insertion" << std::endl;
    parameters.set_first_solution_strategy(FirstSolutionStrategy::LOCAL_CHEAPEST_INSERTION);
    break;
  case 3:
    std::cout << "Savings" << std::endl;
    parameters.set_first_solution_strategy(FirstSolutionStrategy::SAVINGS);
    break;
  case 4:
    std::cout << "Paralell Cheapest Insertion" << std::endl;
    parameters.set_first_solution_strategy(FirstSolutionStrategy::PARALLEL_CHEAPEST_INSERTION);
    break;
  case 5:
    std::cout << "Default" << std::endl;
    break;
  case 6:
    std::cout << "Christofides" << std::endl;
    parameters.set_first_solution_strategy(FirstSolutionStrategy::CHRISTOFIDES);
    break;
  default:
    if (has_overall_duration) {
      std::cout << "Christofides" << std::endl;
      parameters.set_first_solution_strategy(FirstSolutionStrategy::CHRISTOFIDES);
    } else if (shift_preference == ForceStart) {
      std::cout << "Path Cheapest Arc" << std::endl;
      parameters.set_first_solution_strategy(FirstSolutionStrategy::PATH_CHEAPEST_ARC);
    } else if (loop_route && unique_configuration && (has_route_duration && size_vehicles == 1 || ((float)size_mtws)/size_missions > 0.2 && size_rest == 0)) {
      std::cout << "Global Cheapest Arc" << std::endl;
      parameters.set_first_solution_strategy(FirstSolutionStrategy::GLOBAL_CHEAPEST_ARC);
    } else if (size_rest > 0 && size_vehicles == 1 || data.DeliveriesCounter() > 0 || size_mtws > 0) {
      std::cout << "Local Cheapest Insertion" << std::endl;
      parameters.set_first_solution_strategy(FirstSolutionStrategy::LOCAL_CHEAPEST_INSERTION);
    } else if (size_rest == 0 && loop_route && unique_configuration && size_vehicles < 10 && !has_route_duration) {
      std::cout << "Savings" << std::endl;
      parameters.set_first_solution_strategy(FirstSolutionStrategy::SAVINGS);
    } else if (size_rest > 0 || unique_configuration || loop_route) {
      std::cout << "Paralell Cheapest Insertion" << std::endl;
      parameters.set_first_solution_strategy(FirstSolutionStrategy::PARALLEL_CHEAPEST_INSERTION);
    } else {
      std::cout << "Default" << std::endl;
    }
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

  bool build_route = RouteBuilder(data, routing, solver, assignment);

  LoggerMonitor * const logger = MakeLoggerMonitor(data, &routing, min_start, size_matrix, FLAGS_debug, FLAGS_intermediate_solutions, &result, filename, true);
  routing.AddSearchMonitor(logger);

  if (data.Size() > 3) {
    if (FLAGS_no_solution_improvement_limit > 0 || FLAGS_initial_time_out_no_solution_improvement > 0) {
      NoImprovementLimit * const no_improvement_limit = MakeNoImprovementLimit(routing.solver(), routing.CostVar(), FLAGS_no_solution_improvement_limit, FLAGS_initial_time_out_no_solution_improvement, FLAGS_time_out_multiplier, true);
      routing.AddSearchMonitor(no_improvement_limit);
    }
  } else {
    SearchLimit * const limit = solver->MakeLimit(kint64max,kint64max,kint64max,1);
    routing.AddSearchMonitor(limit);
  }

  const Assignment *solution;
  if ((data.Routes().size() > 0 && build_route || data.OrderCounter() == 1) && routing.solver()->CheckAssignment(assignment)) {
    solution = routing.SolveFromAssignmentWithParameters(assignment, parameters);
  } else {
    solution = routing.SolveWithParameters(parameters);
  }


  if (solution != NULL) {
    if (result.routes_size() > 0) result.clear_routes();
    for (int route_nbr = 0; route_nbr < routing.vehicles(); route_nbr++) {
      ortools_result::Route* route = result.add_routes();
      int previous_index = -1;
      for (int64 index = routing.Start(route_nbr); !routing.IsEnd(index); index = solution->Value(routing.NextVar(index))) {
        ortools_result::Activity* activity = route->add_activities();
        RoutingModel::NodeIndex nodeIndex = routing.IndexToNode(index);
        activity->set_start_time(solution->Min(routing.GetMutableDimension("time")->CumulVar(index)));
        if (previous_index == -1) activity->set_type("start");
        else {
           if (index >= size_missions) {
            activity->set_type("break");
            activity->set_index(int64 (nodeIndex.value() - size_missions));
          } else {
            activity->set_type("service");
            activity->set_index(data.ProblemIndex(nodeIndex));
            activity->set_alternative(data.AlternativeIndex(nodeIndex));
          }
        }
        for (int64 q = 0 ; q < data.Quantities(RoutingModel::NodeIndex(0)).size(); ++q) {
          double exchange = solution->Min(routing.CumulVar(solution->Value(routing.NextVar(index)), "quantity" + std::to_string(q)));
          activity->add_quantities(exchange/1000.);
        }
        previous_index = index;
      }
      ortools_result::Activity* end_activity = route->add_activities();
      RoutingModel::NodeIndex nodeIndex = routing.IndexToNode(routing.End(route_nbr));
      end_activity->set_index(data.ProblemIndex(nodeIndex));
      end_activity->set_start_time(solution->Min(routing.GetMutableDimension("time")->CumulVar(routing.End(route_nbr))));
      end_activity->set_type("end");
    }

    std::vector<double> scores = logger->GetFinalScore();
    result.set_cost((int64)scores[0]);
    result.set_duration(scores[1]);
    result.set_iterations(scores[2]);

    std::fstream output(filename, std::ios::out | std::ios::trunc | std::ios::binary);
    if (!result.SerializeToOstream(&output)) {
      std::cout << "Failed to write result." << std::endl;
      return -1;
    }
    output.close();

    logger->GetFinalLog();
  } else {
    std::cout << "No solution found..." << std::endl;
  }

  google::protobuf::ShutdownProtobufLibrary();
  return 0;
}

} // namespace operations_research

int main(int argc, char **argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  if(FLAGS_time_limit_in_ms > 0 || FLAGS_no_solution_improvement_limit > 0) {
    operations_research::TSPTWDataDT tsptw_data(FLAGS_instance_file);
    return operations_research::TSPTWSolver(tsptw_data, FLAGS_solution_file);
  } else {
    std::cout << "No Stop condition" << std::endl;
  }

  return 0;
}
