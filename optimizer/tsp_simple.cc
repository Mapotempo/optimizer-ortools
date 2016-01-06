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
#include "routing_common/routing_common_flags.h"

DEFINE_int64(time_limit_in_ms, 2000, "Time limit in ms, 0 means no limit.");
DEFINE_int64(soft_upper_bound, 3, "Soft upper bound multiplicator, 0 means hard limit.");

namespace operations_research {

void TSPTWSolver(const TSPTWDataDT & data) {

  const int size = data.Size();
  const int size_matrix = data.SizeMatrix();
  const int size_rest = data.SizeRest();

  std::vector<std::pair<RoutingModel::NodeIndex, RoutingModel::NodeIndex>> *start_ends = new std::vector<std::pair<RoutingModel::NodeIndex, RoutingModel::NodeIndex>>(1);
  (*start_ends)[0] = std::make_pair(data.Start(), data.Stop());
  RoutingModel routing(size, 1, *start_ends);
  routing.SetCost(NewPermanentCallback(&data, &TSPTWDataDT::Distance));

  const int64 horizon = data.Horizon();
  routing.AddDimension(NewPermanentCallback(&data, &TSPTWDataDT::TimePlusServiceTime),
    horizon, horizon, true, "time");

  routing.GetMutableDimension("time")->SetSpanCostCoefficientForAllVehicles(5);

  //  Setting time windows
  for (RoutingModel::NodeIndex i(1); i < size_matrix - 1; ++i) {
    int64 index = routing.NodeToIndex(i);
    IntVar* const cumul_var = routing.CumulVar(index, "time");
    int64 const ready = data.ReadyTime(i);
    int64 const due = data.DueTime(i);

    if (ready <= 0 && due <= 0) {
      std::vector<RoutingModel::NodeIndex> *vect = new std::vector<RoutingModel::NodeIndex>(1);
      (*vect)[0] = i;
      routing.AddDisjunction(*vect, 0); // skip node for free
        cumul_var->SetMin(0);
        cumul_var->SetMax(0);
    } else if (ready > 0 || due > 0) {
      if (ready > 0) {
        cumul_var->SetMin(ready);
      }
      if (due > 0 && due < 2147483647) {
        if (FLAGS_soft_upper_bound > 0) {
          routing.SetCumulVarSoftUpperBound(i, "time", due, FLAGS_soft_upper_bound);
        } else {
          routing.SetCumulVarSoftUpperBound(i, "time", due, 10000000);
        }
      }

      std::vector<RoutingModel::NodeIndex> *vect = new std::vector<RoutingModel::NodeIndex>(1);
      (*vect)[0] = i;
      routing.AddDisjunction(*vect, 100000);
    } else {
      std::vector<RoutingModel::NodeIndex> *vect = new std::vector<RoutingModel::NodeIndex>(1);
      (*vect)[0] = i;
      routing.AddDisjunction(*vect);
    }
  }

  for (int n = 0; n < size_rest; ++n) {
    std::vector<RoutingModel::NodeIndex> *vect = new std::vector<RoutingModel::NodeIndex>(1);
    RoutingModel::NodeIndex rest(size_matrix + n);
    (*vect)[0] = rest;

    int64 index = routing.NodeToIndex(rest);
    IntVar* const cumul_var = routing.CumulVar(index, "time");
    int64 const ready = data.ReadyTime(rest);
    int64 const due = data.DueTime(rest);

    if (ready > 0) {
      cumul_var->SetMin(ready);
    }
    if (due > 0 && due < 2147483647) {
      if (FLAGS_soft_upper_bound > 0) {
        routing.SetCumulVarSoftUpperBound(rest, "time", due, FLAGS_soft_upper_bound);
      } else {
        routing.SetCumulVarSoftUpperBound(rest, "time", due, 10000000);
      }
    }
    routing.AddDisjunction(*vect);
  }

  //  Search strategy
  // routing.set_first_solution_strategy(RoutingModel::ROUTING_DEFAULT_STRATEGY);
  // routing.set_first_solution_strategy(RoutingModel::RoutingStrategy::ROUTING_GLOBAL_CHEAPEST_ARC);
  // routing.set_first_solution_strategy(RoutingModel::RoutingStrategy::ROUTING_LOCAL_CHEAPEST_ARC);
  // routing.set_first_solution_strategy(RoutingModel::RoutingStrategy::ROUTING_PATH_CHEAPEST_ARC);
  // routing.set_first_solution_strategy(RoutingModel::RoutingStrategy::ROUTING_EVALUATOR_STRATEGY);
  // routing.set_first_solution_strategy(RoutingModel::RoutingStrategy::ROUTING_ALL_UNPERFORMED);
  // routing.set_first_solution_strategy(RoutingModel::RoutingStrategy::ROUTING_BEST_INSERTION);

  // routing.set_metaheuristic(RoutingModel::RoutingMetaheuristic::ROUTING_GREEDY_DESCENT);
  routing.set_metaheuristic(RoutingModel::RoutingMetaheuristic::ROUTING_GUIDED_LOCAL_SEARCH);
  // routing.set_metaheuristic(RoutingModel::RoutingMetaheuristic::ROUTING_SIMULATED_ANNEALING);
  // routing.set_metaheuristic(RoutingModel::RoutingMetaheuristic::ROUTING_TABU_SEARCH);

  // routing.SetCommandLineOption("routing_no_lns", "true");

  if (FLAGS_time_limit_in_ms > 0) {
    routing.UpdateTimeLimit(FLAGS_time_limit_in_ms);
  }

  Solver *solver = routing.solver();

  const Assignment* solution = routing.Solve(NULL);

  if (solution != NULL) {
    std::cout << "Cost: " << solution->ObjectiveValue() << std::endl;
    TSPTWSolution sol(data, &routing, solution);
    for(int route_nbr = 0; route_nbr < routing.vehicles(); route_nbr++) {
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
  operations_research::TSPTWDataDT tsptw_data(FLAGS_instance_file);
  operations_research::TSPTWSolver(tsptw_data);

  return 0;
}
