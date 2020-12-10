#include <vector>

#include "ortools/constraint_solver/routing_index_manager.h"

namespace operations_research {
class RoutingValues {
public:
  explicit RoutingValues(const TSPTWDataDT& tsptw_data) { BuildValues(tsptw_data); }

  struct NodeValue {
    NodeValue() : initial_time_value(-1) {}
    int64_t initial_time_value;
  };

  std::vector<NodeValue>& NodeValues() { return node_values; }
  std::vector<NodeValue>& RouteStartValues() { return route_start_values; }
  std::vector<NodeValue>& RouteEndValues() { return route_end_values; }

  NodeValue& NodeValues(RoutingIndexManager::NodeIndex i) {
    return node_values[i.value()];
  }
  NodeValue& RouteStartValues(int index) { return route_start_values[index]; }
  NodeValue& RouteEndValues(int index) { return route_end_values[index]; }

private:
  void BuildValues(const TSPTWDataDT tsptw_data) {
    for (int i = 0; i < tsptw_data.Size(); ++i) {
      node_values.emplace_back();
    }

    for (std::size_t i = 0; i < tsptw_data.Vehicles().size(); ++i) {
      route_start_values.emplace_back();
      route_end_values.emplace_back();
    }
  }
  std::vector<NodeValue> node_values;
  std::vector<NodeValue> route_start_values;
  std::vector<NodeValue> route_end_values;
};
} // namespace operations_research
