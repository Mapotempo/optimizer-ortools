// Copyright 2011-2014 Google
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
//
//  Common routing flags.

#ifndef OR_TOOLS_TUTORIALS_CPLUSPLUS_ROUTING_COMMON_FLAGS_H
#define OR_TOOLS_TUTORIALS_CPLUSPLUS_ROUTING_COMMON_FLAGS_H

#include "ortools/base/commandlineflags.h"

#include "common/constants.h"

ABSL_FLAG(int, instance_size, 10, "Number of nodes, including the depot.");
ABSL_FLAG(std::string, instance_name, "Dummy instance", "Instance name.");

ABSL_FLAG(int, instance_depot, 0, "Depot for instance.");

ABSL_FLAG(std::string, instance_file, "", "TSPLIB instance file.");
ABSL_FLAG(std::string, solution_file, "", "TSPLIB solution file.");

ABSL_FLAG(std::string, distance_file, "", "TSP matrix distance file.");


ABSL_FLAG(int, edge_min, 1, "Minimum edge value.");
ABSL_FLAG(int, edge_max, 100, "Maximum edge value.");


ABSL_FLAG(int, instance_edges_percent, 20, "Percent of edges in the graph.");

ABSL_FLAG(int, x_max, 100, "Maximum x coordinate.");
ABSL_FLAG(int, y_max, 100, "Maximum y coordinate.");


ABSL_FLAG(int, width_size, 6, "Width size of fields in output.");

ABSL_FLAG(int, epix_width, 10, "Width of the pictures in cm.");
ABSL_FLAG(int, epix_height, 10, "Height  of the pictures in cm.");

ABSL_FLAG(double, epix_radius, 0.3, "Radius of circles.");

ABSL_FLAG(bool, epix_node_labels, false, "Print node labels?");

ABSL_FLAG(int, percentage_forbidden_arcs_max, 94, "Maximum percentage of arcs to forbid.");
ABSL_FLAG(int64_t, M, operations_research::kPostiveInfinityInt64, "Big m value to represent infinity.");


#endif //  OR_TOOLS_TUTORIALS_CPLUSPLUS_ROUTING_COMMON_FLAGS_H