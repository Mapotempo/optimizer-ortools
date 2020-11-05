#include <vector>

namespace operations_research {
class RoutingValues {
public:
  explicit RoutingValues(const TSPTWDataDT& tsptw_data) { BuildValues(tsptw_data); }

  struct NodeValue {
    NodeValue() : initial_time_value(-1) {}
    int64 initial_time_value;
  };

  std::vector<NodeValue>& NodeValues() { return node_values; }
  std::vector<NodeValue>& RouteEndValues() { return node_values; }

  NodeValue& NodeValues(RoutingIndexManager::NodeIndex i) {
    return node_values[i.value()];
  }
  NodeValue& RouteEndValues(int index) { return route_end_values[index]; }

private:
  void BuildValues(const TSPTWDataDT tsptw_data) {
    int size = tsptw_data.Size();
    for (int i = 0; i < size; ++i) {
      node_values.push_back(NodeValue());
    }

    for (int i = 0; i < tsptw_data.Vehicles().size(); ++i) {
      route_end_values.push_back(NodeValue());
    }
  }
  std::vector<NodeValue> node_values;
  std::vector<NodeValue> route_end_values;
};
} // namespace operations_research
