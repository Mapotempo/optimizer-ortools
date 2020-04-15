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

#include "./limits.h"

#include "google/protobuf/text_format.h"

#include "ortools/base/commandlineflags.h"

#include "ortools/base/protoutil.h"
#include "ortools/base/timer.h"

#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"
#include "ortools/constraint_solver/routing_parameters.pb.h"

namespace operations_research {

bool CheckOverflow(int64 a, int64 b) {
  if (a > std::pow(2, 52) / b)
    return true;
  return false;
}

double GetSpanCostForVehicleForDimension(RoutingModel& routing,
                                         const Assignment* solution, int vehicle,
                                         const std::string& dimension_name) {
  if (routing.GetMutableDimension(dimension_name) == nullptr)
    return 0;

  return (solution->Min(routing.GetMutableDimension(dimension_name)
                            ->CumulVar(routing.End(vehicle))) -
          solution->Max(routing.GetMutableDimension(dimension_name)
                            ->CumulVar(routing.Start(vehicle)))) *
         routing.GetMutableDimension(dimension_name)
             ->GetSpanCostCoefficientForVehicle(vehicle) /
         CUSTOM_BIGNUM;
}

void MissionsBuilder(const TSPTWDataDT& data, RoutingModel& routing,
                     RoutingIndexManager& manager, int64 size, int64 min_start) {
  const int size_vehicles = data.Vehicles().size();
  // const int size_matrix = data.SizeMatrix();
  const int size_problem = data.SizeProblem();

  int64 max_time     = (2 * data.MaxTime() + data.MaxServiceTime()) * data.MaxTimeCost();
  int64 max_distance = 2 * data.MaxDistance() * data.MaxDistanceCost();
  int64 max_value    = 2 * data.MaxValue() * data.MaxValueCost();

  bool overflow_danger =
      CheckOverflow(max_time + max_distance + max_value, size_vehicles);
  int64 data_verif = (max_time + max_distance + max_value) * size_vehicles;

  overflow_danger = overflow_danger || CheckOverflow(data_verif, std::pow(2, 4) * size);
  data_verif      = data_verif * std::pow(2, 4) * size;
  RoutingIndexManager::NodeIndex i(0);
  int32 tw_index = 0;
  int64 disjunction_cost =
      !overflow_danger && !CheckOverflow(data_verif, size) ? data_verif : std::pow(2, 52);

  for (int activity = 0; activity <= size_problem; ++activity) {
    std::vector<int64>* vect = new std::vector<int64>();
    int32 alternative_size   = data.AlternativeSize(activity);

    int64 priority       = data.Priority(i);
    int64 exclusion_cost = data.ExclusionCost(i);

    for (int alternative = 0; alternative < alternative_size; ++alternative) {
      vect->push_back(manager.NodeToIndex(i));
      int64 index              = manager.NodeToIndex(i);
      std::vector<int64> ready = data.ReadyTime(i);
      std::vector<int64> due   = data.DueTime(i);

      IntVar* cumul_var           = routing.GetMutableDimension(kTime)->CumulVar(index);
      int64 const late_multiplier = data.LateMultiplier(i);
      std::vector<int64> sticky_vehicle = data.VehicleIndices(i);
      std::string service_id            = data.ServiceId(i);
      if (ready.size() > 0 &&
          (ready.at(0) > -CUSTOM_MAX_INT || due.at(due.size() - 1) < CUSTOM_MAX_INT)) {
        if (FLAGS_debug) {
          std::cout << "Node " << i << " index " << index << " ["
                    << (ready.at(0) - min_start) << " : "
                    << (due.at(due.size() - 1) - min_start) << "]:" << data.ServiceTime(i)
                    << std::endl;
        }
        if (ready.at(0) > -CUSTOM_MAX_INT) {
          cumul_var->SetMin(ready.at(0));
        }
        if (due.at(due.size() - 1) < CUSTOM_MAX_INT) {
          if (late_multiplier > 0) {
            routing.GetMutableDimension(kTime)->SetCumulVarSoftUpperBound(
                index, due.at(due.size() - 1), late_multiplier);
          } else {
            cumul_var->SetMax(due.at(due.size() - 1));
            if (due.size() > 1) {
              for (tw_index = due.size() - 1; tw_index--;) {
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
        for (int v = 0; v < size_vehicles; ++v)
          vehicle_indices.push_back(v);
        std::set_intersection(vehicle_indices.begin(), vehicle_indices.end(),
                              sticky_vehicle.begin(), sticky_vehicle.end(),
                              std::back_inserter(vehicle_intersection));
        std::set_difference(vehicle_indices.begin(), vehicle_indices.end(),
                            vehicle_intersection.begin(), vehicle_intersection.end(),
                            std::back_inserter(vehicle_difference));
        if (vehicle_difference.size() > 0) {
          for (int64 remove : vehicle_difference)
            routing.VehicleVar(index)->RemoveValue(remove);
        }
      }

      std::vector<bool> refill_quantities = data.RefillQuantities(i);
      for (std::size_t q = 0; q < data.Quantities(i).size(); ++q) {
        RoutingDimension* quantity_dimension =
            routing.GetMutableDimension("quantity" + std::to_string(q));
        if (!refill_quantities.at(q))
          quantity_dimension->SlackVar(index)->SetValue(0);
        routing.AddVariableMinimizedByFinalizer(quantity_dimension->CumulVar(index));
      }

      ++i;
    }

    if (FLAGS_debug) {
      std::cout << "Activity " << activity << "\t exclusion cost: " << exclusion_cost
                << "\t disjunction cost: " << disjunction_cost * std::pow(2, 4 - priority)
                << std::endl;
    }
    // Otherwise this single service is never assigned
    if (size == 1)
      routing.AddDisjunction(*vect);
    else
      routing.AddDisjunction(*vect, exclusion_cost == -1
                                        ? disjunction_cost * std::pow(2, 4 - priority)
                                        : exclusion_cost);

    delete vect;
  }
}

bool RouteBuilder(const TSPTWDataDT& data, RoutingModel& routing,
                  RoutingIndexManager& manager, Assignment* assignment) {
  const int size_vehicles = data.Vehicles().size();
  std::vector<std::vector<int64>> routes(size_vehicles);
  for (TSPTWDataDT::Route* route : data.Routes()) {
    int64 current_index;
    IntVar* previous_var = NULL;
    std::vector<int64> route_variable_indicies;

    if (route->vehicle_index >= 0) {
      previous_var = routing.NextVar(routing.Start(route->vehicle_index));
      assignment->Add(previous_var);
    }
    for (std::string service_id : route->service_ids) {
      current_index = data.IdIndex(service_id);
      if (current_index != -1) {
        route_variable_indicies.push_back(
            manager.NodeToIndex(RoutingIndexManager::NodeIndex(current_index)));
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
      std::vector<int64> actual_route = routes.at(route->vehicle_index);
      actual_route.insert(actual_route.end(), route_variable_indicies.begin(),
                          route_variable_indicies.end());
      routes.at(route->vehicle_index) = actual_route;
    }
  }
  return routing.RoutesToAssignment(routes, true, false, assignment);
}

std::vector<std::vector<IntervalVar*>> RestBuilder(const TSPTWDataDT& data,
                                                   RoutingModel& routing) {
  Solver* solver          = routing.solver();
  const int size_vehicles = data.Vehicles().size();
  std::vector<std::vector<IntervalVar*>> stored_rests;
  for (int vehicle_index = 0; vehicle_index < size_vehicles; ++vehicle_index) {
    const RoutingDimension& time_dimension = routing.GetDimensionOrDie(kTime);
    IntVar* const cumul_var     = time_dimension.CumulVar(routing.Start(vehicle_index));
    IntVar* const cumul_var_end = time_dimension.CumulVar(routing.End(vehicle_index));
    std::vector<IntervalVar*> rest_array;
    for (TSPTWDataDT::Rest rest : data.Vehicles().at(vehicle_index)->rests) {
      IntervalVar* const rest_interval = solver->MakeFixedDurationIntervalVar(
          std::max(rest.ready_time[0], data.Vehicles().at(vehicle_index)->time_start),
          std::min(rest.due_time[0],
                   data.Vehicles().at(vehicle_index)->time_end - rest.service_time),
          rest.service_time, // Currently only one timewindow
          false, absl::StrCat("Rest/", rest.rest_id, "/", vehicle_index));
      rest_array.push_back(rest_interval);
      solver->AddConstraint(
          solver->MakeGreaterOrEqual(rest_interval->SafeStartExpr(0), cumul_var));
      solver->AddConstraint(
          solver->MakeLessOrEqual(rest_interval->SafeEndExpr(0), cumul_var_end));
      routing.AddIntervalToAssignment(rest_interval);
    }
    routing.GetMutableDimension(kTime)->SetBreakIntervalsOfVehicle(
        rest_array, vehicle_index, data.ServiceTimes());
    stored_rests.push_back(rest_array);
  }
  return stored_rests;
}

void RelationBuilder(const TSPTWDataDT& data, RoutingModel& routing,
                     bool& has_overall_duration) {
  Solver* solver = routing.solver();
  // const int size_vehicles = data.Vehicles().size();

  Solver::IndexEvaluator1 vehicle_evaluator = [&data](int64 index) {
    return data.VehicleDay(index);
  };

  Solver::IndexEvaluator1 day_to_vehicle_evaluator = [&data](int64 index) {
    return data.DayIndexToVehicleIndex(index);
  };

  // Solver::IndexEvaluator1 alternative_vehicle_evaluator = [&data](int64 index) {
  //   return data.VehicleDayAlt(index);
  // };

  std::vector<IntVar*> next_vars;
  for (int i = 0; i < data.SizeMissions(); ++i) {
    next_vars.push_back(routing.NextVar(i));
  }

  for (TSPTWDataDT::Relation* relation : data.Relations()) {
    int64 previous_index;
    int64 current_index;
    std::vector<int64> previous_indices;
    std::vector<std::pair<int, int>> pairs;
    switch (relation->type) {
    case Sequence:
      // int64 new_current_index;
      previous_index = data.IdIndex(relation->linked_ids->at(0));
      for (std::size_t link_index = 1; link_index < relation->linked_ids->size();
           ++link_index) {
        current_index = data.IdIndex(relation->linked_ids->at(link_index));
        pairs.push_back(std::make_pair(previous_index, current_index));

        IntVar* const previous_active_var = routing.ActiveVar(previous_index);
        IntVar* const active_var          = routing.ActiveVar(current_index);
        solver->AddConstraint(solver->MakeLessOrEqual(active_var, previous_active_var));
        // routing.AddPickupAndDelivery(previous_index, current_index);

        IntVar* const previous_vehicle_var = routing.VehicleVar(previous_index);
        IntVar* const vehicle_var          = routing.VehicleVar(current_index);
        IntExpr* const isConstraintActive =
            solver->MakeProd(previous_active_var, active_var)->Var();
        routing.NextVar(current_index)->RemoveValues(previous_indices);

        solver->AddConstraint(solver->MakeEquality(
            solver->MakeProd(isConstraintActive, previous_vehicle_var),
            solver->MakeProd(isConstraintActive, vehicle_var)));
        solver->AddConstraint(solver->MakeEquality(
            solver->MakeProd(isConstraintActive, routing.NextVar(previous_index)),
            solver->MakeProd(isConstraintActive, current_index)));
        previous_indices.push_back(previous_index);
        previous_index = current_index;
      }
      if (relation->linked_ids->size() > 1)
        solver->AddConstraint(solver->MakePathPrecedenceConstraint(next_vars, pairs));
      break;
    case Order:
      previous_index = data.IdIndex(relation->linked_ids->at(0));
      previous_indices.push_back(previous_index);
      for (std::size_t link_index = 1; link_index < relation->linked_ids->size();
           ++link_index) {
        current_index = data.IdIndex(relation->linked_ids->at(link_index));
        pairs.push_back(std::make_pair(previous_index, current_index));
        // routing.AddPickupAndDelivery(previous_index, current_index);
        IntVar* const previous_active_var = routing.ActiveVar(previous_index);
        IntVar* const active_var          = routing.ActiveVar(current_index);

        IntVar* const previous_vehicle_var = routing.VehicleVar(previous_index);
        IntVar* const vehicle_var          = routing.VehicleVar(current_index);
        routing.NextVar(current_index)->RemoveValues(previous_indices);

        solver->AddConstraint(solver->MakeLessOrEqual(active_var, previous_active_var));
        IntExpr* const isConstraintActive =
            solver->MakeProd(previous_active_var, active_var);
        solver->AddConstraint(solver->MakeEquality(
            solver->MakeProd(isConstraintActive, previous_vehicle_var),
            solver->MakeProd(isConstraintActive, vehicle_var)));
        previous_indices.push_back(current_index);
        previous_index = current_index;
      }
      if (relation->linked_ids->size() > 1)
        solver->AddConstraint(solver->MakePathPrecedenceConstraint(next_vars, pairs));
      break;
    case SameRoute:
      previous_index = data.IdIndex(relation->linked_ids->at(0));
      for (std::size_t link_index = 1; link_index < relation->linked_ids->size();
           ++link_index) {
        current_index = data.IdIndex(relation->linked_ids->at(link_index));
        IntVar* const previous_active_var = routing.ActiveVar(previous_index);
        IntVar* const active_var          = routing.ActiveVar(current_index);

        solver->AddConstraint(solver->MakeLessOrEqual(active_var, previous_active_var));
        IntVar* const previous_vehicle_var = routing.VehicleVar(previous_index);
        IntVar* const vehicle_var          = routing.VehicleVar(current_index);

        solver->AddConstraint(solver->MakeLessOrEqual(active_var, previous_active_var));
        IntVar* const isConstraintActive =
            solver->MakeProd(previous_active_var, active_var)->Var();
        solver->AddConstraint(solver->MakeEquality(
            solver->MakeProd(previous_vehicle_var, isConstraintActive),
            solver->MakeProd(vehicle_var, isConstraintActive)));
        previous_index = current_index;
      }
      break;
    case MinimumDayLapse:
      previous_index = data.IdIndex(relation->linked_ids->at(0));
      for (std::size_t link_index = 1; link_index < relation->linked_ids->size();
           ++link_index) {
        current_index = data.IdIndex(relation->linked_ids->at(link_index));
        IntVar* const previous_active_var = routing.ActiveVar(previous_index);
        IntVar* const active_var          = routing.ActiveVar(current_index);
        IntVar* const vehicle_var         = routing.VehicleVar(current_index);

        IntVar* const previous_part =
            solver->MakeElement(vehicle_evaluator, routing.VehicleVar(previous_index))
                ->Var();

        IntVar* const vehicle_index_var =
            solver
                ->MakeElement(day_to_vehicle_evaluator,
                              solver->MakeSum(previous_part, relation->lapse)->Var())
                ->Var();
        solver->AddConstraint(solver->MakeLessOrEqual(active_var, previous_active_var));
        IntExpr* const isConstraintActive =
            solver->MakeProd(previous_active_var, active_var);

        solver->AddConstraint(solver->MakeGreaterOrEqual(
            solver->MakeProd(vehicle_var, isConstraintActive),
            solver->MakeProd(vehicle_index_var, isConstraintActive)));
        previous_index = current_index;
      }
      break;
    case MaximumDayLapse:
      previous_index = data.IdIndex(relation->linked_ids->at(0));
      for (std::size_t link_index = 1; link_index < relation->linked_ids->size();
           ++link_index) {
        current_index = data.IdIndex(relation->linked_ids->at(link_index));
        IntVar* const previous_active_var  = routing.ActiveVar(previous_index);
        IntVar* const active_var           = routing.ActiveVar(current_index);
        IntVar* const vehicle_var          = routing.VehicleVar(current_index);
        IntVar* const previous_vehicle_var = routing.VehicleVar(previous_index);

        IntVar* const previous_part =
            solver->MakeElement(vehicle_evaluator, previous_vehicle_var)->Var();

        IntVar* const vehicle_index_var =
            solver
                ->MakeElement(day_to_vehicle_evaluator,
                              solver->MakeSum(previous_part, relation->lapse)->Var())
                ->Var();
        solver->AddConstraint(solver->MakeLessOrEqual(active_var, previous_active_var));

        IntExpr* const isConstraintActive =
            solver->MakeProd(previous_active_var, active_var);
        solver->AddConstraint(solver->MakeGreaterOrEqual(
            solver->MakeProd(vehicle_var, isConstraintActive),
            solver->MakeProd(previous_vehicle_var, isConstraintActive)));
        solver->AddConstraint(solver->MakeLessOrEqual(
            solver->MakeProd(vehicle_var, isConstraintActive),
            solver->MakeProd(vehicle_index_var, isConstraintActive)));
        previous_index = current_index;
      }
      break;
    case Shipment:
      previous_index = data.IdIndex(relation->linked_ids->at(0));
      for (std::size_t link_index = 1; link_index < relation->linked_ids->size();
           ++link_index) {
        current_index = data.IdIndex(relation->linked_ids->at(link_index));
        routing.AddPickupAndDelivery(previous_index, current_index);
        solver->AddConstraint(solver->MakeEquality(routing.VehicleVar(previous_index),
                                                   routing.VehicleVar(current_index)));
        solver->AddConstraint(solver->MakeLessOrEqual(
            routing.GetMutableDimension(kTime)->CumulVar(previous_index),
            routing.GetMutableDimension(kTime)->CumulVar(current_index)));
        previous_index = current_index;
      }
      break;
    case MeetUp:
      previous_index = data.IdIndex(relation->linked_ids->at(0));
      for (std::size_t link_index = 1; link_index < relation->linked_ids->size();
           ++link_index) {
        current_index = data.IdIndex(relation->linked_ids->at(link_index));
        IntVar* const previous_active_var = routing.ActiveVar(previous_index);
        IntVar* const active_var          = routing.ActiveVar(current_index);
        IntExpr* const isConstraintActive =
            solver->MakeProd(previous_active_var, active_var)->Var();

        IntExpr* const previous_part =
            solver->MakeProd(isConstraintActive, routing.VehicleVar(previous_index));
        IntExpr* const next_part =
            solver->MakeProd(isConstraintActive, routing.VehicleVar(current_index));

        IntExpr* const lapse = solver->MakeProd(isConstraintActive, 1);
        solver->AddConstraint(solver->MakeGreaterOrEqual(
            solver->MakeAbs(solver->MakeDifference(next_part, previous_part)), lapse));

        IntExpr* const previous_part_global_index =
            solver->MakeElement(vehicle_evaluator, routing.VehicleVar(previous_index));
        IntExpr* const next_part_global_index =
            solver->MakeElement(vehicle_evaluator, routing.VehicleVar(current_index));

        solver->AddConstraint(
            solver->MakeEquality(previous_part_global_index, next_part_global_index));

        solver->AddConstraint(solver->MakeEquality(
            solver->MakeProd(
                isConstraintActive,
                routing.GetMutableDimension(kTime)->CumulVar(previous_index)),
            solver->MakeProd(
                isConstraintActive,
                routing.GetMutableDimension(kTime)->CumulVar(current_index))));
        previous_index = current_index;
      }
      break;
    case MaximumDurationLapse:
      previous_index = data.IdIndex(relation->linked_ids->at(0));
      for (std::size_t link_index = 1; link_index < relation->linked_ids->size();
           ++link_index) {
        current_index = data.IdIndex(relation->linked_ids->at(link_index));
        IntVar* const previous_active_var = routing.ActiveVar(previous_index);
        IntVar* const active_var          = routing.ActiveVar(current_index);

        solver->AddConstraint(solver->MakeLessOrEqual(active_var, previous_active_var));
        IntExpr* const isConstraintActive =
            solver->MakeProd(previous_active_var, active_var)->Var();
        IntExpr* const difference = solver->MakeDifference(
            routing.GetMutableDimension(kTime)->CumulVar(current_index),
            routing.GetMutableDimension(kTime)->CumulVar(previous_index));

        IntExpr* const lapse = solver->MakeProd(isConstraintActive, relation->lapse);
        solver->AddConstraint(solver->MakeLessOrEqual(
            solver->MakeProd(isConstraintActive, difference), lapse));
        previous_index = current_index;
      }
      break;
    case MinimumDurationLapse:
      previous_index = data.IdIndex(relation->linked_ids->at(0));
      for (std::size_t link_index = 1; link_index < relation->linked_ids->size();
           ++link_index) {
        current_index = data.IdIndex(relation->linked_ids->at(link_index));
        IntVar* const previous_active_var = routing.ActiveVar(previous_index);
        IntVar* const active_var          = routing.ActiveVar(current_index);

        solver->AddConstraint(solver->MakeGreaterOrEqual(previous_active_var, active_var));
        IntExpr* const isConstraintActive =
            solver->MakeProd(previous_active_var, active_var)->Var();
        IntExpr* const difference = solver->MakeDifference(
            routing.GetMutableDimension(kTime)->CumulVar(current_index),
            routing.GetMutableDimension(kTime)->CumulVar(previous_index));

        IntExpr* const lapse = solver->MakeProd(isConstraintActive, relation->lapse);
        solver->AddConstraint(solver->MakeGreaterOrEqual(
            solver->MakeProd(isConstraintActive, difference), lapse));
        previous_index = current_index;
      }
      break;
    case NeverFirst:
      for (std::size_t link_index = 0; link_index < relation->linked_ids->size();
           ++link_index) {
        current_index = data.IdIndex(relation->linked_ids->at(link_index));
        for (std::size_t v = 0; v < data.Vehicles().size(); ++v) {
          int64 start_index = routing.Start(v);
          // int64 end_index = routing.End(v);
          IntVar* const next_var = routing.NextVar(start_index);
          next_var->RemoveValue(current_index);
        }
      }
      break;
    case ForceFirst: {
      std::vector<int64> values;
      for (int activity = 0; activity < data.SizeMissions(); ++activity) {
        values.push_back(activity);
      }

      std::vector<int64> first_values;
      for (std::size_t link_index = 0; link_index < relation->linked_ids->size();
           ++link_index) {
        current_index = data.IdIndex(relation->linked_ids->at(link_index));
        first_values.push_back(current_index);

        std::vector<int64>::iterator it = std::find(values.begin(), values.end(), current_index);
        values.erase(it);
      }

      for (int index : values ) {
        IntVar* const next_var = routing.NextVar(index);
        next_var->RemoveValues(first_values);
      }
    } break;
    case NeverLast:
      for (std::size_t link_index = 0; link_index < relation->linked_ids->size();
           ++link_index) {
        current_index = data.IdIndex(relation->linked_ids->at(link_index));
        IntVar* const next_var = routing.NextVar(current_index);
        for (std::size_t v = 0; v < data.Vehicles().size(); ++v) {
          int64 end_index = routing.End(v);
          next_var->RemoveValue(end_index);
      }
    } break;
    case ForceLast: {
      std::vector<int64> values;
      for (std::size_t link_index = 0; link_index < relation->linked_ids->size();
           ++link_index) {
        current_index = data.IdIndex(relation->linked_ids->at(link_index));
        values.push_back(current_index);
      }
      std::vector<int64> intermediate_values(values);
      for (std::size_t v = 0; v < data.Vehicles().size(); ++v) {
        int64 end_index = routing.End(v);
        intermediate_values.push_back(end_index);
      }
      for (int index : values) {
        IntVar* const next_var = routing.NextVar(index);
        next_var->SetValues(intermediate_values);
      }
    } break;
    case VehicleGroupDuration: {
      if (relation->lapse > -1) {
        has_overall_duration = true;
        std::vector<IntVar*> same_vehicle_vars;
        for (std::size_t link_index = 0;
             link_index < relation->linked_vehicle_ids->size(); ++link_index) {
          current_index =
              data.VehicleIdIndex(relation->linked_vehicle_ids->at(link_index));
          int64 start_index = routing.Start(current_index);
          int64 end_index   = routing.End(current_index);
          IntVar* const cumul_var =
              routing.GetMutableDimension(kTime)->CumulVar(start_index);
          IntVar* const end_cumul_var =
              routing.GetMutableDimension(kTime)->CumulVar(end_index);
          IntVar* const vehicle_time =
              solver->MakeDifference(end_cumul_var, cumul_var)->Var();
          same_vehicle_vars.push_back(vehicle_time);
        }
        solver->AddConstraint(
            solver->MakeLessOrEqual(solver->MakeSum(same_vehicle_vars), relation->lapse));
      }
    } break;
    case VehicleTrips: {
      if (relation->linked_vehicle_ids->size() > 1) {
        int current_vehicle_index;
        int previous_vehicle_index =
            data.VehicleIdIndex(relation->linked_vehicle_ids->at(0));
        for (std::size_t link_index = 1;
             link_index < relation->linked_vehicle_ids->size(); ++link_index) {
          current_vehicle_index =
              data.VehicleIdIndex(relation->linked_vehicle_ids->at(link_index));
          int64 current_start_index = routing.Start(current_vehicle_index);
          int64 previous_end_index  = routing.End(previous_vehicle_index);
          IntVar* const current_cumul_var =
              routing.GetMutableDimension(kTime)->CumulVar(current_start_index);
          IntVar* const previous_end_cumul_var =
              routing.GetMutableDimension(kTime)->CumulVar(previous_end_index);
          solver->AddConstraint(
              solver->MakeLessOrEqual(previous_end_cumul_var, current_cumul_var));
          previous_vehicle_index = current_vehicle_index;
        }
      }
    } break;
    case VehicleGroupNumber: {
      if (relation->linked_vehicle_ids->size() > 1) {
        std::vector<IntVar*> used_vehicles;
        for (std::size_t link_index = 0;
             link_index < relation->linked_vehicle_ids->size(); ++link_index) {
          int vehicle_index =
              data.VehicleIdIndex(relation->linked_vehicle_ids->at(link_index));
          int64 start_index = routing.Start(vehicle_index);
          int64 end_index   = routing.End(vehicle_index);
          IntVar* const is_vehicle_used =
              solver
                  ->MakeConditionalExpression(
                      solver->MakeIsDifferentCstVar(routing.NextVar(start_index), end_index),
                      solver->MakeIntConst(1), 0)
                  ->Var();
          used_vehicles.push_back(is_vehicle_used);
        }

        solver->AddConstraint(solver->MakeLessOrEqual(solver->MakeSum(used_vehicles),
                                                      relation->lapse));
      }

    } break;
    default:
      break;
    }
  }
}

void AddBalanceDimensions(const TSPTWDataDT& data, RoutingModel& routing,
                          RoutingIndexManager& manager, int horizon) {
  if (FLAGS_balance) {
    std::vector<IntVar*> ends_distance_vars;
    std::vector<IntVar*> shift_vars;
    std::vector<int> zero_evaluators;
    Solver* solver = routing.solver();

    int v = 0;
    for (TSPTWDataDT::Vehicle* vehicle : data.Vehicles()) {
      zero_evaluators.push_back(
          routing.RegisterTransitCallback([vehicle, &manager](int64 i, int64 j) {
            return vehicle->ReturnZero(manager.IndexToNode(i),

                                       manager.IndexToNode(j));
          }));
      const operations_research::RoutingDimension& time_dimension =
          routing.GetDimensionOrDie(kTime);
      const operations_research::RoutingDimension& distance_dimension =
          routing.GetDimensionOrDie(kDistance);

      int64 start_index = routing.Start(v);
      int64 end_index   = routing.End(v);

      IntVar* const time_cumul_var     = time_dimension.CumulVar(start_index);
      IntVar* const time_cumul_var_end = time_dimension.CumulVar(end_index);
      IntVar* const dist_end_cumul_var = distance_dimension.CumulVar(end_index);

      ends_distance_vars.push_back(dist_end_cumul_var);
      IntVar* const time_difference =
          solver->MakeDifference(time_cumul_var_end, time_cumul_var)->Var();
      shift_vars.push_back(time_difference);
      ++v;
    }

    routing.AddDimensionWithVehicleTransits(zero_evaluators, horizon, horizon, true,
                                            kTimeBalance);
    routing.AddDimensionWithVehicleTransits(zero_evaluators, LLONG_MAX, LLONG_MAX, true,
                                            kDistanceBalance);

    v = 0;
    for (TSPTWDataDT::Vehicle* vehicle : data.Vehicles()) {
      routing.GetMutableDimension(kTimeBalance)
          ->SetSpanCostCoefficientForVehicle((int64)vehicle->cost_time_multiplier, v);
      routing.GetMutableDimension(kDistanceBalance)
          ->SetSpanCostCoefficientForVehicle((int64)vehicle->cost_distance_multiplier, v);

      int64 start_index = routing.Start(v);
      int64 end_index   = routing.End(v);

      IntVar* const start_time_cumul_var =
          routing.GetMutableDimension(kTimeBalance)->CumulVar(start_index);
      IntVar* const end_time_cumul_var =
          routing.GetMutableDimension(kTimeBalance)->CumulVar(end_index);
      start_time_cumul_var->SetMax(0);
      solver->AddConstraint(solver->MakeGreaterOrEqual(
          end_time_cumul_var, solver->MakeDifference(solver->MakeMax(shift_vars),
                                                     solver->MakeMin(shift_vars))));

      IntVar* const start_distance_cumul_var =
          routing.GetMutableDimension(kDistanceBalance)->CumulVar(start_index);
      IntVar* const end_distance_cumul_var =
          routing.GetMutableDimension(kDistanceBalance)->CumulVar(end_index);
      start_distance_cumul_var->SetMax(0);
      solver->AddConstraint(solver->MakeGreaterOrEqual(
          end_distance_cumul_var,
          solver->MakeDifference(solver->MakeMax(ends_distance_vars),
                                 solver->MakeMin(ends_distance_vars))));
      ++v;
    }
  }
}

void AddCapacityDimensions(const TSPTWDataDT& data, RoutingModel& routing,
                           RoutingIndexManager& manager) {
  for (std::size_t unit_i = 0; unit_i < data.Vehicles().at(0)->capacity.size();
       ++unit_i) {
    std::vector<int64> capacities;
    for (TSPTWDataDT::Vehicle* vehicle : data.Vehicles()) {
      int64 coef = vehicle->overload_multiplier[unit_i];
      if (coef == 0 && vehicle->capacity.at(unit_i) >= 0) {
        capacities.push_back(vehicle->capacity.at(unit_i));
      } else {
        capacities.push_back(LLONG_MAX);
      }
    }
    operations_research::RoutingTransitCallback2 quantity_evaluator = [&data, &manager,
                                                                       unit_i](int64 from,
                                                                               int64 to) {
      return data.Quantity(unit_i, manager.IndexToNode(from), manager.IndexToNode(to));
    };
    routing.AddDimensionWithVehicleCapacity(
        routing.RegisterTransitCallback(quantity_evaluator), LLONG_MAX, capacities, false,
        "quantity" + std::to_string(unit_i));
  }
}

void AddDistanceDimensions(const TSPTWDataDT& data, RoutingModel& routing,
                           RoutingIndexManager& manager, int64 maximum_route_distance,
                           bool free_approach_return) {
  std::vector<int> distance_evaluators;
  std::vector<int> fake_distance_evaluators;
  std::vector<int> distance_order_evaluators;

  for (TSPTWDataDT::Vehicle* vehicle : data.Vehicles()) {
    distance_evaluators.push_back(
        routing.RegisterTransitCallback([vehicle, &manager](int64 i, int64 j) {
          return vehicle->Distance(manager.IndexToNode(i), manager.IndexToNode(j));
        }));
    if (free_approach_return == true) {
      fake_distance_evaluators.push_back(
          routing.RegisterTransitCallback([vehicle, &manager](int64 i, int64 j) {
            return vehicle->FakeDistance(manager.IndexToNode(i), manager.IndexToNode(j));
          }));
    }
    if (FLAGS_nearby) {
      distance_order_evaluators.push_back(
          routing.RegisterTransitCallback([vehicle, &manager](int64 i, int64 j) {
            return vehicle->DistanceOrder(manager.IndexToNode(i), manager.IndexToNode(j));
          }));
    }
  }

  if (FLAGS_nearby) {
    routing.AddDimensionWithVehicleTransits(distance_order_evaluators, 0, LLONG_MAX, true,
                                            kDistanceOrder);
  }
  if (free_approach_return == true) {
    routing.AddDimensionWithVehicleTransits(fake_distance_evaluators, 0,
                                            maximum_route_distance, true, kFakeDistance);
  }

  routing.AddDimensionWithVehicleTransits(distance_evaluators, 0, maximum_route_distance,
                                          true, kDistance);

  int v = 0;
  for (TSPTWDataDT::Vehicle* vehicle : data.Vehicles()) {
    if (FLAGS_nearby) {
      routing.GetMutableDimension(kDistanceOrder)
          ->SetSpanCostCoefficientForVehicle(vehicle->cost_distance_multiplier / 5, v);
    }
    if (vehicle->free_approach == true || vehicle->free_return == true) {
      routing.GetMutableDimension(kFakeDistance)
          ->SetSpanCostCoefficientForVehicle(vehicle->cost_distance_multiplier, v);
    } else {
      routing.GetMutableDimension(kDistance)->SetSpanCostCoefficientForVehicle(
          vehicle->cost_distance_multiplier, v);
    }
    ++v;
  }
}

void AddTimeDimensions(const TSPTWDataDT& data, RoutingModel& routing,
                       RoutingIndexManager& manager, int64 horizon,
                       bool free_approach_return) {
  std::vector<int> time_evaluators;
  std::vector<int> fake_time_evaluators;
  std::vector<int> time_order_evaluators;

  for (TSPTWDataDT::Vehicle* vehicle : data.Vehicles()) {
    time_evaluators.push_back(routing.RegisterTransitCallback([vehicle, &manager](
                                                                  int64 i, int64 j) {
      return vehicle->TimePlusServiceTime(manager.IndexToNode(i), manager.IndexToNode(j));
    }));

    if (free_approach_return == true) {
      fake_time_evaluators.push_back(
          routing.RegisterTransitCallback([vehicle, &manager](int64 i, int64 j) {
            return vehicle->FakeTimePlusServiceTime(manager.IndexToNode(i),
                                                    manager.IndexToNode(j));
          }));
    }
    if (FLAGS_nearby) {
      time_order_evaluators.push_back(
          routing.RegisterTransitCallback([vehicle, &manager](int64 i, int64 j) {
            return vehicle->TimeOrder(manager.IndexToNode(i), manager.IndexToNode(j));
          }));
    }
  }

  routing.AddDimensionWithVehicleTransits(time_evaluators, horizon, horizon, false,
                                          kTime);
  routing.AddDimensionWithVehicleTransits(time_evaluators, 0, horizon, false,
                                          kTimeNoWait);

  if (FLAGS_nearby)
    routing.AddDimensionWithVehicleTransits(time_order_evaluators, 0, LLONG_MAX, true,
                                            kTimeOrder);
  if (free_approach_return == true) {
    routing.AddDimensionWithVehicleTransits(fake_time_evaluators, horizon, horizon, false,
                                            kFakeTime);
    routing.AddDimensionWithVehicleTransits(fake_time_evaluators, 0, horizon, false,
                                            kFakeTimeNoWait);
  }

  int v = 0;
  for (TSPTWDataDT::Vehicle* vehicle : data.Vehicles()) {
    int64 without_wait_cost =
        vehicle->cost_time_multiplier - vehicle->cost_waiting_time_multiplier;
    // Vehicle costs
    if (vehicle->free_approach == true || vehicle->free_return == true) {
      routing.GetMutableDimension(kFakeTime)->SetSpanCostCoefficientForVehicle(
          (int64)vehicle->cost_waiting_time_multiplier, v);
      routing.GetMutableDimension(kFakeTimeNoWait)
          ->SetSpanCostCoefficientForVehicle((int64)std::max(without_wait_cost, (int64)0),
                                             v);
    } else {
      routing.GetMutableDimension(kTime)->SetSpanCostCoefficientForVehicle(
          (int64)vehicle->cost_waiting_time_multiplier, v);
      routing.GetMutableDimension(kTimeNoWait)
          ->SetSpanCostCoefficientForVehicle((int64)std::max(without_wait_cost, (int64)0),
                                             v);
    }
    if (FLAGS_nearby) {
      routing.GetMutableDimension(kTimeOrder)
          ->SetSpanCostCoefficientForVehicle(vehicle->cost_time_multiplier / 5, v);
    }
    ++v;
  }
}

void AddValueDimensions(const TSPTWDataDT& data, RoutingModel& routing,
                        RoutingIndexManager& manager) {
  std::vector<int> value_evaluators;
  for (TSPTWDataDT::Vehicle* vehicle : data.Vehicles()) {
    value_evaluators.push_back(
        routing.RegisterTransitCallback([vehicle, &manager](int64 i, int64 j) {
          return vehicle->ValuePlusServiceValue(manager.IndexToNode(i),
                                                manager.IndexToNode(j));
        }));
  }
  routing.AddDimensionWithVehicleTransits(value_evaluators, 0, LLONG_MAX, true, kValue);
  int v = 0;
  for (TSPTWDataDT::Vehicle* vehicle : data.Vehicles()) {
    routing.GetMutableDimension(kValue)->SetSpanCostCoefficientForVehicle(
        vehicle->cost_value_multiplier, v);
    ++v;
  }
}

void AddVehicleTimeConstraints(const TSPTWDataDT& data, RoutingModel& routing,
                               bool& has_route_duration) {
  Solver* solver = routing.solver();
  int v          = 0;
  for (TSPTWDataDT::Vehicle* vehicle : data.Vehicles()) {
    const operations_research::RoutingDimension& time_dimension =
        routing.GetDimensionOrDie(kTime);

    int64 start_index                = routing.Start(v);
    int64 end_index                  = routing.End(v);
    IntVar* const time_cumul_var     = time_dimension.CumulVar(start_index);
    IntVar* const time_cumul_var_end = time_dimension.CumulVar(end_index);

    // Vehicle time windows
    if (vehicle->time_start >= 0) {
      // Timewindow Start is always hard
      time_cumul_var->SetMin(vehicle->time_start);
      // In case of Force Start a penalty is applied for every time unit
      // away from the opening
      if (vehicle->shift_preference == ForceStart) {
        routing.GetMutableDimension(kTime)->SetCumulVarSoftUpperBound(
            start_index, vehicle->time_start,
            std::max((int64)vehicle->cost_time_multiplier,
                     (int64)vehicle->cost_distance_multiplier) *
                100);
        IntVar* const slack_var =
            routing.GetMutableDimension(kTime)->SlackVar(start_index);
        routing.AddVariableMinimizedByFinalizer(slack_var);
      }
    }
    if (vehicle->time_end < CUSTOM_MAX_INT) {
      int64 coef = vehicle->late_multiplier;
      if (coef > 0) {
        // Timewindow end may be soft
        routing.GetMutableDimension(kTime)->SetCumulVarSoftUpperBound(
            end_index, vehicle->time_end, coef);
        if (vehicle->shift_preference == ForceEnd) {
          routing.AddVariableMaximizedByFinalizer(time_cumul_var_end);
        }
      } else {
        // Or hard
        time_cumul_var_end->SetMax(vehicle->time_end);
        if (vehicle->shift_preference == ForceEnd) {
          time_cumul_var_end->SetMin(vehicle->time_end);
          IntVar* const slack_var =
              routing.GetMutableDimension(kTime)->SlackVar(end_index);
          routing.AddVariableMinimizedByFinalizer(slack_var);
        }
      }
    }

    // Route duration may be limited
    if (vehicle->duration >= 0 &&
        vehicle->time_end - vehicle->time_start > vehicle->duration) {
      has_route_duration = true;
      routing.AddVariableMinimizedByFinalizer(time_cumul_var_end);
      if (vehicle->shift_preference == ForceStart)
        routing.AddVariableMinimizedByFinalizer(time_cumul_var);
      else
        routing.AddVariableMaximizedByFinalizer(time_cumul_var);

      routing.GetMutableDimension(kTime)->SetSpanUpperBoundForVehicle(vehicle->duration, v);
    } else if (FLAGS_balance) {
      routing.AddVariableMinimizedByFinalizer(time_cumul_var_end);
      routing.AddVariableMaximizedByFinalizer(time_cumul_var);
    } else {
      if (vehicle->free_approach == true)
        routing.AddVariableMaximizedByFinalizer(time_cumul_var);
      if (vehicle->free_return == true)
        routing.AddVariableMaximizedByFinalizer(time_cumul_var_end);
    }
    ++v;
  }
}

void AddVehicleDistanceConstraints(const TSPTWDataDT& data, RoutingModel& routing) {
  Solver* solver = routing.solver();
  int v          = 0;
  const operations_research::RoutingDimension& distance_dimension =
      routing.GetDimensionOrDie(kDistance);
  for (TSPTWDataDT::Vehicle* vehicle : data.Vehicles()) {
    if (vehicle->distance > 0) {
      int64 end_index = routing.End(v);
      // Vehicle maximum distance
      IntVar* const dist_end_cumul_var = distance_dimension.CumulVar(end_index);
      solver->AddConstraint(
          solver->MakeLessOrEqual(dist_end_cumul_var, vehicle->distance));
    }
    ++v;
  }
}

void AddVehicleCapacityConstraints(const TSPTWDataDT& data, RoutingModel& routing) {
  int v = 0;
  for (TSPTWDataDT::Vehicle* vehicle : data.Vehicles()) {
    int64 end_index = routing.End(v);
    for (std::size_t i = 0; i < vehicle->capacity.size(); ++i) {
      int64 coef = vehicle->overload_multiplier[i];
      // Capacity is already limited by the dimension horizon
      if (vehicle->capacity[i] >= 0 && coef > 0) {
        std::string kQuantity = ("quantity" + std::to_string(i)).c_str();
        routing.GetMutableDimension(kQuantity)->SetCumulVarSoftUpperBound(
            end_index, vehicle->capacity[i], coef);
      }
    }
    ++v;
  }
}

void SetFirstSolutionStrategy(const TSPTWDataDT& data,
                              RoutingSearchParameters& parameters,
                              ShiftPref shift_preference, bool has_overall_duration,
                              bool unique_configuration, bool has_route_duration,
                              bool loop_route) {
  // Search strategy
  const int size_missions = data.SizeMissions();
  const int size_mtws     = data.TwiceTWsCounter();
  const int size_vehicles = data.Vehicles().size();
  const int size_rest     = data.SizeRest();
  switch (FLAGS_solver_parameter) {
  case 0:
    parameters.set_first_solution_strategy(FirstSolutionStrategy::PATH_CHEAPEST_ARC);
    break;
  case 1:
    parameters.set_first_solution_strategy(FirstSolutionStrategy::GLOBAL_CHEAPEST_ARC);
    break;
  case 2:
    parameters.set_first_solution_strategy(
        FirstSolutionStrategy::LOCAL_CHEAPEST_INSERTION);
    break;
  case 3:
    parameters.set_first_solution_strategy(FirstSolutionStrategy::SAVINGS);
    break;
  case 4:
    parameters.set_first_solution_strategy(
        FirstSolutionStrategy::PARALLEL_CHEAPEST_INSERTION);
    break;
  case 5:
    parameters.set_first_solution_strategy(FirstSolutionStrategy::FIRST_UNBOUND_MIN_VALUE);
    break;
  case 6:
    parameters.set_first_solution_strategy(FirstSolutionStrategy::CHRISTOFIDES);
    break;
  default:
    if (has_overall_duration) {
      parameters.set_first_solution_strategy(FirstSolutionStrategy::CHRISTOFIDES);
    } else if (shift_preference == ForceStart) {
      parameters.set_first_solution_strategy(FirstSolutionStrategy::PATH_CHEAPEST_ARC);
    } else if (loop_route && unique_configuration &&
               ((has_route_duration && size_vehicles == 1) ||
                (((float)size_mtws) / size_missions > 0.2 && size_rest == 0))) {
      parameters.set_first_solution_strategy(FirstSolutionStrategy::GLOBAL_CHEAPEST_ARC);
    } else if ((size_rest > 0 && size_vehicles == 1) || data.DeliveriesCounter() > 0 ||
               size_mtws > 0) {
      parameters.set_first_solution_strategy(
          FirstSolutionStrategy::LOCAL_CHEAPEST_INSERTION);
    } else if (size_rest == 0 && loop_route && unique_configuration &&
               size_vehicles < 10 && !has_route_duration) {
      parameters.set_first_solution_strategy(FirstSolutionStrategy::SAVINGS);
    } else if (size_rest > 0 || unique_configuration || loop_route) {
      parameters.set_first_solution_strategy(
          FirstSolutionStrategy::PARALLEL_CHEAPEST_INSERTION);
    } else {
      // std::cout << "Default" << std::endl;
    }
  }
  // Unused options
  // parameters.set_first_solution_strategy(FirstSolutionStrategy::ALL_UNPERFORMED);
  // parameters.set_first_solution_strategy(FirstSolutionStrategy::BEST_INSERTION);
  // parameters.set_first_solution_strategy(FirstSolutionStrategy::ROUTING_BEST_INSERTION);
  // parameters.set_first_solution_strategy(FirstSolutionStrategy::LOCAL_CHEAPEST_ARC);
  // parameters.set_first_solution_strategy(FirstSolutionStrategy::PATH_MOST_CONSTRAINED_ARC);
}

int TSPTWSolver(const TSPTWDataDT& data, std::string filename) {
  ortools_result::Result result;

  const int size_vehicles   = data.Vehicles().size();
  const int size            = data.Size();
  const int size_matrix     = data.SizeMatrix();
  bool has_lateness         = false;
  bool has_route_duration   = false;
  bool has_overall_duration = false;
  bool free_approach_return = false;

  for (TSPTWDataDT::Vehicle* vehicle : data.Vehicles()) {
    if (vehicle->free_approach == true || vehicle->free_return == true) {
      free_approach_return = true;
    }
  }

  std::vector<std::pair<RoutingIndexManager::NodeIndex, RoutingIndexManager::NodeIndex>>*
      start_ends = new std::vector<
          std::pair<RoutingIndexManager::NodeIndex, RoutingIndexManager::NodeIndex>>(
          size_vehicles);
  for (int v = 0; v < size_vehicles; ++v) {
    (*start_ends)[v] =
        std::make_pair(data.Vehicles().at(v)->start, data.Vehicles().at(v)->stop);
    has_lateness |= data.Vehicles().at(v)->late_multiplier > 0;
  }

  RoutingIndexManager manager(size, size_vehicles, *start_ends);
  RoutingModel routing(manager);

  int64 maximum_route_distance = 0;
  int64 v                      = 0;
  while ((maximum_route_distance != INT_MAX) && (v < size_vehicles)) {
    if (data.Vehicles().at(v)->distance == -1)
      maximum_route_distance = INT_MAX;
    else
      maximum_route_distance =
          std::max(maximum_route_distance, data.Vehicles().at(v)->distance);
    v++;
  }

  // Dimensions
  const int64 horizon =
      data.Horizon() * (has_lateness && !CheckOverflow(data.Horizon(), 2) ? 2 : 1);

  AddTimeDimensions(data, routing, manager, horizon, free_approach_return);
  AddDistanceDimensions(data, routing, manager, maximum_route_distance,
                        free_approach_return);
  AddBalanceDimensions(data, routing, manager, horizon);
  AddCapacityDimensions(data, routing, manager);
  AddValueDimensions(data, routing, manager);

  Solver* solver         = routing.solver();
  Assignment* assignment = routing.solver()->MakeAssignment();

  AddVehicleTimeConstraints(data, routing, has_route_duration);
  AddVehicleDistanceConstraints(data, routing);
  AddVehicleCapacityConstraints(data, routing);

  v               = 0;
  int64 min_start = CUSTOM_MAX_INT;

  for (TSPTWDataDT::Vehicle* vehicle : data.Vehicles()) {
    routing.SetFixedCostOfVehicle(vehicle->cost_fixed, v);
    min_start = std::min(min_start, vehicle->time_start);
    ++v;
  }

  std::vector<IntVar*> used_vehicles;
  if (FLAGS_vehicle_limit > 0) {
    v = 0;
    for (TSPTWDataDT::Vehicle* vehicle : data.Vehicles()) {
      int64 start_index = routing.Start(v);
      int64 end_index   = routing.End(v);
      IntVar* const is_vehicle_used =
          solver
              ->MakeConditionalExpression(
                  solver->MakeIsDifferentCstVar(routing.NextVar(start_index), end_index),
                  solver->MakeIntConst(1), 0)
              ->Var();
      used_vehicles.push_back(is_vehicle_used);
      ++v;
    }

    solver->AddConstraint(solver->MakeLessOrEqual(solver->MakeSum(used_vehicles),
                                                  (int64)FLAGS_vehicle_limit));
  }

  // Setting solve parameters indicators
  int64 previous_distance_depot_start = -1;
  int64 previous_distance_depot_end   = -1;
  ShiftPref shift_preference          = MinimizeSpan;
  bool loop_route                     = true;
  bool unique_configuration           = true;
  RoutingIndexManager::NodeIndex compareNodeIndex =
      manager.IndexToNode(rand() % (data.SizeMatrix() - 2));
  TSPTWDataDT::Vehicle* previous_vehicle = NULL;
  for (int route_nbr = 0; route_nbr < routing.vehicles(); route_nbr++) {
    TSPTWDataDT::Vehicle* vehicle = data.Vehicles().at(route_nbr);
    RoutingIndexManager::NodeIndex nodeIndexStart =
        manager.IndexToNode(routing.Start(route_nbr));
    RoutingIndexManager::NodeIndex nodeIndexEnd =
        manager.IndexToNode(routing.End(route_nbr));

    int64 distance_depot_start =
        std::max(vehicle->Time(nodeIndexStart, compareNodeIndex),
                 vehicle->Distance(nodeIndexStart, compareNodeIndex));
    int64 distance_depot_end =
        std::max(vehicle->Time(compareNodeIndex, nodeIndexEnd),
                 vehicle->Distance(compareNodeIndex, nodeIndexEnd));
    int64 distance_start_end = std::max(vehicle->Time(nodeIndexStart, nodeIndexEnd),
                                        vehicle->Distance(nodeIndexStart, nodeIndexEnd));

    if (previous_vehicle != NULL) {
      if (previous_distance_depot_start != distance_depot_start ||
          previous_distance_depot_end != distance_depot_end) {
        unique_configuration = false;
      }
      if (distance_start_end != 0 ||
          (distance_depot_start == 0 && distance_depot_end == 0)) {
        loop_route = false;
      }
    }
    shift_preference              = vehicle->shift_preference;
    previous_distance_depot_start = distance_depot_start;
    previous_distance_depot_end   = distance_depot_end;
    previous_vehicle              = vehicle;
  }

  // Setting visit time windows
  MissionsBuilder(data, routing, manager, size - 2, min_start);
  std::vector<std::vector<IntervalVar*>> stored_rests = RestBuilder(data, routing);
  RelationBuilder(data, routing, has_overall_duration);
  RoutingSearchParameters parameters = DefaultRoutingSearchParameters();

  CHECK(google::protobuf::TextFormat::MergeFromString(FLAGS_routing_search_parameters,
                                                      &parameters));
  SetFirstSolutionStrategy(data, parameters, shift_preference, has_overall_duration,
                           unique_configuration, has_route_duration, loop_route);

  // parameters.set_local_search_metaheuristic(LocalSearchMetaheuristic::GREEDY_DESCENT);
  // parameters.set_guided_local_search_lambda_coefficient(0.5);
  // parameters.set_local_search_metaheuristic(LocalSearchMetaheuristic::SIMULATED_ANNEALING);
  // parameters.set_local_search_metaheuristic(LocalSearchMetaheuristic::TABU_SEARCH);
  // parameters.set_local_search_metaheuristic(LocalSearchMetaheuristic::GENERIC_TABU_SEARCH);

  const Assignment* solution;
  if (FLAGS_time_limit_in_ms > 0) {
    CHECK_OK(util_time::EncodeGoogleApiProto(absl::Milliseconds(FLAGS_time_limit_in_ms),
                                             parameters.mutable_time_limit()));
  }

  if (FLAGS_only_first_solution) {
    parameters.set_solution_limit(1);
  } else {
    parameters.set_local_search_metaheuristic(
        LocalSearchMetaheuristic::GUIDED_LOCAL_SEARCH);
  }

  routing.CloseModelWithParameters(parameters);

  bool build_route = RouteBuilder(data, routing, manager, assignment);

  LoggerMonitor* const logger = MakeLoggerMonitor(
      data, &routing, &manager, min_start, size_matrix, FLAGS_debug,
      FLAGS_intermediate_solutions, &result, stored_rests, filename, true);
  routing.AddSearchMonitor(logger);

  if (data.Size() > 3) {
    if (FLAGS_no_solution_improvement_limit > 0 || FLAGS_minimum_duration > 0 ||
        FLAGS_init_duration > 0) {
      NoImprovementLimit* const no_improvement_limit = MakeNoImprovementLimit(
          routing.solver(), routing.CostVar(), FLAGS_no_solution_improvement_limit,
          FLAGS_minimum_duration, FLAGS_time_out_multiplier, FLAGS_init_duration, true);
      routing.AddSearchMonitor(no_improvement_limit);
    }
  } else {
    SearchLimit* const limit = solver->MakeLimit(kint64max, kint64max, kint64max, 1);
    routing.AddSearchMonitor(limit);
  }

  if (((data.Routes().size() > 0 && build_route) || data.OrderCounter() == 1) &&
      routing.solver()->CheckAssignment(assignment)) {
    std::cout << "Using initial solution provided." << std::endl;
    solution = routing.SolveFromAssignmentWithParameters(assignment, parameters);
  } else {
    std::cout << "First solution strategy : "
              << FirstSolutionStrategy::Value_Name(parameters.first_solution_strategy())
              << std::endl;
    solution = routing.SolveWithParameters(parameters);
  }

  if (FLAGS_debug) {
    std::cout << std::endl;
    std::cout << "Solutions: " << solver->solutions() << std::endl;
    std::cout << "Failures: " << solver->failures() << std::endl;
    std::cout << "Branches: " << solver->branches() << std::endl;
    std::cout << "Wall time: " << solver->wall_time() << "ms" << std::endl;
    std::cout << std::endl;
  }

  if (solution != NULL) {
    if (result.routes_size() > 0)
      result.clear_routes();

    double total_time_order_cost(0), total_distance_order_cost(0);

    for (int route_nbr = 0; route_nbr < routing.vehicles(); route_nbr++) {
      std::vector<IntervalVar*> rests = stored_rests.at(route_nbr);
      ortools_result::Route* route    = result.add_routes();
      int previous_index              = -1;
      int previous_start_time         = 0;
      bool vehicle_used               = false;
      for (int64 index = routing.Start(route_nbr); !routing.IsEnd(index);
           index       = solution->Value(routing.NextVar(index))) {
        for (std::vector<IntervalVar*>::iterator it = rests.begin(); it != rests.end();) {
          int64 rest_start_time = solution->StartValue(*it);
          if (solution->PerformedValue(*it) && previous_index != -1 &&
              rest_start_time >= previous_start_time &&
              rest_start_time <=
                  solution->Min(routing.GetMutableDimension(kTime)->CumulVar(index))) {
            std::stringstream ss((*it)->name());
            std::string item;
            std::vector<std::string> parsed_name;
            while (std::getline(ss, item, '/')) {
              parsed_name.push_back(item);
            }

            ortools_result::Activity* rest = route->add_activities();
            rest->set_type("break");
            rest->set_id(parsed_name[1]);
            rest->set_start_time(rest_start_time);
            it = rests.erase(it);
          } else {
            ++it;
          }
        }

        ortools_result::Activity* activity       = route->add_activities();
        RoutingIndexManager::NodeIndex nodeIndex = manager.IndexToNode(index);
        activity->set_start_time(
            solution->Min(routing.GetMutableDimension(kTime)->CumulVar(index)));
        activity->set_current_distance(
            solution->Min(routing.GetMutableDimension(kDistance)->CumulVar(index)));
        if (previous_index == -1)
          activity->set_type("start");
        else {
          vehicle_used = true;
          activity->set_type("service");
          activity->set_id(data.ServiceId(nodeIndex));
          activity->set_index(data.ProblemIndex(nodeIndex));
          activity->set_alternative(data.AlternativeIndex(nodeIndex));
        }
        for (std::size_t q = 0;
             q < data.Quantities(RoutingIndexManager::NodeIndex(0)).size(); ++q) {
          double exchange =
              solution->Min(routing.GetMutableDimension("quantity" + std::to_string(q))
                                ->CumulVar(solution->Value(routing.NextVar(index))));
          activity->add_quantities(exchange);
        }
        previous_index = index;
        previous_start_time =
            solution->Min(routing.GetMutableDimension(kTime)->CumulVar(index));
      }

      for (std::vector<IntervalVar*>::iterator it = rests.begin(); it != rests.end();
           ++it) {
        int64 rest_start_time = solution->StartValue(*it);
        if (solution->PerformedValue(*it)) {
          ortools_result::Activity* rest = route->add_activities();
          std::stringstream ss((*it)->name());
          std::string item;
          std::vector<std::string> parsed_name;
          while (std::getline(ss, item, '/')) {
            parsed_name.push_back(item);
          }
          rest->set_type("break");
          rest->set_id(parsed_name[1]);
          rest->set_start_time(rest_start_time);
        }
      }
      ortools_result::Activity* end_activity = route->add_activities();
      RoutingIndexManager::NodeIndex nodeIndex =
          manager.IndexToNode(routing.End(route_nbr));
      end_activity->set_index(data.ProblemIndex(nodeIndex));
      end_activity->set_start_time(solution->Min(
          routing.GetMutableDimension(kTime)->CumulVar(routing.End(route_nbr))));
      end_activity->set_current_distance(solution->Min(
          routing.GetMutableDimension(kDistance)->CumulVar(routing.End(route_nbr))));
      end_activity->set_type("end");

      ortools_result::Costs* route_costs = ortools_result::Costs().New();

      if (vehicle_used) {
        double fixed_cost = routing.GetFixedCostOfVehicle(route_nbr) / CUSTOM_BIGNUM;
        route_costs->set_fixed(fixed_cost);
      }

      double time_cost =
          GetSpanCostForVehicleForDimension(routing, solution, route_nbr, kTime);
      route_costs->set_time(time_cost);

      double distance_cost =
          GetSpanCostForVehicleForDimension(routing, solution, route_nbr, kDistance);
      route_costs->set_distance(distance_cost);

      if (FLAGS_nearby) {
        double time_order_cost =
            GetSpanCostForVehicleForDimension(routing, solution, route_nbr, kTimeOrder);
        total_time_order_cost += time_order_cost;
        route_costs->set_time_order(time_order_cost);

        double distance_order_cost = GetSpanCostForVehicleForDimension(
            routing, solution, route_nbr, kDistanceOrder);
        total_distance_order_cost += distance_order_cost;
        route_costs->set_distance_order(distance_order_cost);
      }

      if (FLAGS_balance) {
        double time_balance_cost =
            GetSpanCostForVehicleForDimension(routing, solution, route_nbr, kTimeBalance);
        route_costs->set_time_balance(time_balance_cost);

        double distance_balance_cost = GetSpanCostForVehicleForDimension(
            routing, solution, route_nbr, kDistanceBalance);
        route_costs->set_distance_balance(distance_balance_cost);
      }

      if (data.Vehicles(route_nbr)->free_approach == true ||
          data.Vehicles(route_nbr)->free_return == true) {
        double fake_time_cost =
            GetSpanCostForVehicleForDimension(routing, solution, route_nbr, kFakeTime);
        route_costs->set_time_fake(fake_time_cost);

        double fake_distance_cost = GetSpanCostForVehicleForDimension(
            routing, solution, route_nbr, kFakeDistance);
        route_costs->set_distance_fake(fake_distance_cost);
      }
      route->set_allocated_costs(route_costs);
    }

    std::vector<double> scores = logger->GetFinalScore();
    result.set_cost(
        (solution->ObjectiveValue() - total_time_order_cost - total_distance_order_cost) /
        CUSTOM_BIGNUM);
    result.set_duration(scores[1]);
    result.set_iterations(scores[2]);

    std::fstream output(filename, std::ios::out | std::ios::trunc | std::ios::binary);
    if (!result.SerializeToOstream(&output)) {
      std::cout << "Failed to write result." << std::endl;
      return -1;
    }
    output.close();

    std::cout << "Final Iteration : " << result.iterations()
              << " Cost : " << result.cost() << " Time : " << result.duration()
              << std::endl;
  } else {
    std::cout << "No solution found..." << std::endl;
  }

  google::protobuf::ShutdownProtobufLibrary();
  delete start_ends;
  return 0;
}

} // namespace operations_research

int main(int argc, char** argv) {
  std::cout << "OR-Tools v" << OR_TOOLS_VERSION << std::endl;

  GOOGLE_PROTOBUF_VERIFY_VERSION;
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_time_limit_in_ms > 0 || FLAGS_no_solution_improvement_limit > 0) {
    operations_research::TSPTWDataDT tsptw_data(FLAGS_instance_file);
    return operations_research::TSPTWSolver(tsptw_data, FLAGS_solution_file);
  } else {
    std::cout << "No stopping condition" << std::endl;
    return -1;
  }

  return 0;
}
