#ifndef OR_TOOLS_TUTORIALS_CPLUSPLUS_TSPTW_DATA_DT_H
#define OR_TOOLS_TUTORIALS_CPLUSPLUS_TSPTW_DATA_DT_H

#include <cmath>
#include <iomanip>
#include <ostream>
#include <string>
#include <vector>

#include "./ortools_vrp.pb.h"

#include "routing_common/routing_common.h"

#include "ortools/constraint_solver/routing.h"

#include "ortools/constraint_solver/routing_index_manager.h"

#define CUSTOM_MAX_INT (int64_t) std::pow(2, 30)

#define CUSTOM_BIGNUM_COST 1e6

#define CUSTOM_BIGNUM_QUANTITY 1e3 // Needs to stay smaller than CUSTOM_BIGNUM_COST

enum RelationType {
  MinimumDurationLapse = 15,
  VehicleGroupNumber   = 14,
  NeverLast            = 13,
  VehicleTrips         = 12,
  VehicleGroupDuration = 11,
  ForceLast            = 10,
  ForceFirst           = 9,
  NeverFirst           = 8,
  MaximumDurationLapse = 7,
  MeetUp               = 6,
  Shipment             = 5,
  MaximumDayLapse      = 4,
  MinimumDayLapse      = 3,
  SameRoute            = 2,
  Order                = 1,
  Sequence             = 0
};
enum ShiftPref { ForceStart = 2, ForceEnd = 1, MinimizeSpan = 0 };

namespace operations_research {

class TSPTWDataDT {
public:
  explicit TSPTWDataDT(const std::string& filename)
      : size_problem_(0)
      , size_(0)
      , size_matrix_(0)
      , size_missions_(0)
      , size_rest_(0)
      , size_alternative_relations_(0)
      , deliveries_counter_(0)
      , horizon_(0)
      , earliest_start_(CUSTOM_MAX_INT)
      , max_distance_(0)
      , max_distance_cost_(0)
      , max_rest_(0)
      , max_service_(0)
      , max_time_(0)
      , max_time_cost_(0)
      , max_value_(0)
      , max_value_cost_(0)
      , multiple_tws_counter_(0)
      , sum_max_time_(0)
      , tws_counter_(0)
      , max_coef_service_(0)
      , max_coef_setup_(0) {
    LoadInstance(filename);
  }

  void LoadInstance(const std::string& filename);

  //  Helper function
  int64_t& SetDistMatrix(const int i, const int j) {
    return distances_matrices_.back().Cost(RoutingIndexManager::NodeIndex(i),
                                           RoutingIndexManager::NodeIndex(j));
  }

  int64_t& SetTimeMatrix(const int i, const int j) {
    return times_matrices_.back().Cost(RoutingIndexManager::NodeIndex(i),
                                       RoutingIndexManager::NodeIndex(j));
  }

  int64_t& SetValueMatrix(const int i, const int j) {
    return values_matrices_.back().Cost(RoutingIndexManager::NodeIndex(i),
                                        RoutingIndexManager::NodeIndex(j));
  }

  int64_t BuildTimeMatrix(const ortools_vrp::Matrix& matrix) {
    int64_t max_time          = 0;
    const int32_t size_matrix = sqrt(matrix.time_size());
    for (int64_t i = 0; i < size_matrix; ++i) {
      for (int64_t j = 0; j < size_matrix; ++j) {
        const int64_t time = matrix.time(i * size_matrix + j) + 0.5;
        if (time < CUSTOM_MAX_INT)
          max_time = std::max(max_time, time);
        SetTimeMatrix(i, j) = time;
      }
      // std::cout << std::endl;
    }

    return max_time;
  }

  int64_t BuildDistanceMatrix(const ortools_vrp::Matrix& matrix) {
    int64_t max_distance      = 0;
    const int32_t size_matrix = sqrt(matrix.distance_size());
    for (int64_t i = 0; i < size_matrix; ++i) {
      for (int64_t j = 0; j < size_matrix; ++j) {
        const int64_t distance = matrix.distance(i * size_matrix + j);
        if (distance < CUSTOM_MAX_INT)
          max_distance = std::max(max_distance, distance);
        SetDistMatrix(i, j) = distance;
      }
    }
    return max_distance;
  }

  int64_t BuildValueMatrix(const ortools_vrp::Matrix& matrix) {
    int64_t max_value         = 0;
    const int32_t size_matrix = sqrt(matrix.value_size());
    for (int64_t i = 0; i < size_matrix; ++i) {
      for (int64_t j = 0; j < size_matrix; ++j) {
        const int64_t value = matrix.value(i * size_matrix + j);
        if (value < CUSTOM_MAX_INT)
          max_value = std::max(max_value, value);
        SetValueMatrix(i, j) = value;
      }
    }
    return max_value;
  }

  int64_t Horizon() const { return horizon_; }

  int64_t MatrixIndex(const RoutingIndexManager::NodeIndex i) const {
    return tsptw_clients_[i.value()].matrix_index;
  }

  int64_t EarliestStart() const { return earliest_start_; }

  int64_t MaxTime() const { return max_time_; }

  int64_t MaxDistance() const { return max_distance_; }

  int64_t MaxValue() const { return max_value_; }

  int64_t MaxServiceTime() const { return max_service_; }

  int64_t MaxTimeCost() const { return max_time_cost_; }

  int64_t MaxDistanceCost() const { return max_distance_cost_; }

  int64_t MaxValueCost() const { return max_value_cost_; }

  int64_t TWsCounter() const { return tws_counter_; }

  int64_t TwiceTWsCounter() const { return multiple_tws_counter_; }

  int64_t DeliveriesCounter() const { return deliveries_counter_; }

  int64_t IdIndex(const std::string& id) const {
    std::map<std::string, int64_t>::const_iterator it = ids_map_.find(id);
    if (it != ids_map_.end())
      return it->second;
    else
      return -1;
  }

  int64_t VehicleIdIndex(const std::string& id) const {
    std::map<std::string, int64_t>::const_iterator it = vehicle_ids_map_.find(id);
    if (it != vehicle_ids_map_.end())
      return it->second;
    else
      return -1;
  }

  int64_t DayIndexToVehicleIndex(const int64_t day_index) const {
    if (day_index_to_vehicle_index_.count(day_index)) {
      return day_index_to_vehicle_index_.at(day_index);
    }
    return CUSTOM_MAX_INT;
  }

  int32_t AlternativeSize(const int32_t problem_index) const {
    if (alternative_size_map_.count(problem_index))
      return alternative_size_map_.at(problem_index);
    return -1;
  }

  std::string ServiceId(const RoutingIndexManager::NodeIndex i) const {
    return tsptw_clients_[i.value()].customer_id;
  }

  std::string PointId(const RoutingIndexManager::NodeIndex i) const {
    return tsptw_clients_[i.value()].point_id;
  }

  int32_t ProblemIndex(const RoutingIndexManager::NodeIndex i) const {
    return tsptw_clients_[i.value()].problem_index;
  }

  int32_t AlternativeIndex(const RoutingIndexManager::NodeIndex i) const {
    return tsptw_clients_[i.value()].alternative_index;
  }

  const std::vector<int64_t>& ReadyTime(const RoutingIndexManager::NodeIndex i) const {
    return tsptw_clients_[i.value()].ready_time;
  }

  const std::vector<int64_t>& DueTime(const RoutingIndexManager::NodeIndex i) const {
    return tsptw_clients_[i.value()].due_time;
  }

  bool AllServicesHaveEnd() const {
    for (std::size_t i = 0; i < tsptw_clients_.size(); i++) {
      if (tsptw_clients_[i].due_time.size() == 0)
        return false;
    }
    return true;
  }

  const std::vector<int64_t>&
  MaximumLateness(const RoutingIndexManager::NodeIndex i) const {
    return tsptw_clients_[i.value()].maximum_lateness;
  }

  int64_t LateMultiplier(const RoutingIndexManager::NodeIndex i) const {
    return tsptw_clients_[i.value()].late_multiplier;
  }

  int64_t ServiceTime(const RoutingIndexManager::NodeIndex i) const {
    return tsptw_clients_[i.value()].service_time;
  }

  const std::vector<int64_t>& ServiceTimes() const { return service_times_; }

  int64_t ServiceValue(const RoutingIndexManager::NodeIndex i) const {
    return tsptw_clients_[i.value()].service_value;
  }

  int64_t SetupTime(const RoutingIndexManager::NodeIndex i) const {
    return tsptw_clients_[i.value()].setup_time;
  }

  int64_t Priority(const RoutingIndexManager::NodeIndex i) const {
    return tsptw_clients_[i.value()].priority;
  }

  int64_t ExclusionCost(const RoutingIndexManager::NodeIndex i) const {
    return tsptw_clients_[i.value()].exclusion_cost;
  }

  const std::vector<int64_t>&
  VehicleIndices(const RoutingIndexManager::NodeIndex i) const {
    return tsptw_clients_[i.value()].vehicle_indices;
  }

  int32_t TimeWindowsSize(const int i) const { return tws_size_[i]; }

  int32_t Size() const { return size_; }

  int32_t SizeMissions() const { return size_missions_; }

  int32_t SizeMatrix() const { return size_matrix_; }

  int32_t SizeProblem() const { return size_problem_; }

  int32_t SizeRest() const { return size_rest_; }

  int32_t SizeAlternativeRelations() const { return size_alternative_relations_; }

  const std::vector<bool>&
  RefillQuantities(const RoutingIndexManager::NodeIndex i) const {
    return tsptw_clients_[i.value()].refill_quantities;
  }

  int64_t Quantity(const std::size_t unit_i, const RoutingIndexManager::NodeIndex from,
                   const RoutingIndexManager::NodeIndex to) const {
    //    CheckNodeIsValid(from);
    //    CheckNodeIsValid(to);
    const int64_t index = from.value();
    if (unit_i < tsptw_clients_[index].quantities.size()) {
      if (tsptw_vehicles_[0].counting[unit_i]) {
        if (tsptw_vehicles_[0].stop == to || tsptw_vehicles_[0].Distance(from, to) > 0 ||
            tsptw_vehicles_[0].Time(from, to) > 0)
          return tsptw_clients_[index].quantities[unit_i] -
                 tsptw_clients_[index].setup_quantities[unit_i];
        else
          return tsptw_clients_[index].quantities[unit_i];
      }
      return tsptw_clients_[index].quantities[unit_i];
    } else {
      return 0;
    }
  }

  const std::vector<int64_t>& Quantities(const RoutingIndexManager::NodeIndex i) const {
    return tsptw_clients_[i.value()].quantities;
  }

  std::vector<int64_t> MaxTimes(const ortools_vrp::Matrix& matrix) const {
    int64_t max_row;
    int32_t size_matrix = sqrt(matrix.time_size());
    std::vector<int64_t> max_times;
    for (int32_t i = 0; i < size_matrix; i++) {
      max_row = 0;
      for (int32_t j = 0; j < size_matrix; j++) {
        int64_t cell = matrix.time(i * size_matrix + j);
        if (cell + 0.5 < CUSTOM_MAX_INT)
          max_row = std::max(max_row, (int64_t)(cell + 0.5));
      }
      max_times.push_back(max_row);
    }
    return max_times;
  }

  struct Rest {
    Rest(std::string id, int64_t ready_t, int64_t due_t, int64_t dur)
        : rest_id(id)
        , ready_time(std::max(0L, ready_t))
        , due_time(std::min(CUSTOM_MAX_INT, due_t))
        , duration(dur) {}
    std::string rest_id;
    int64_t ready_time;
    int64_t due_time;
    int64_t duration;
  };

  struct Vehicle {
    Vehicle(TSPTWDataDT* data_, int32_t size_)
        : data(data_)
        , size(size_)
        , problem_matrix_index(0)
        , value_matrix_index(0)
        , start_point_id("")
        , vehicle_indices(0)
        , initial_capacity(0)
        , initial_load(0)
        , capacity(0)
        , overload_multiplier(0)
        , break_size(0)
        , time_start(0)
        , time_end(0)
        , time_maximum_lateness(CUSTOM_MAX_INT)
        , late_multiplier(0) {}

    int32_t SizeMatrix() const { return size_matrix; }

    int32_t SizeRest() const { return size_rest; }

    void SetStart(const RoutingIndexManager::NodeIndex s) {
      CHECK_LT(s, size);
      start = s;
    }

    void SetStop(const RoutingIndexManager::NodeIndex s) {
      CHECK_LT(s, size);
      stop = s;
    }

    int64_t ReturnZero(const RoutingIndexManager::NodeIndex,
                       const RoutingIndexManager::NodeIndex) const {
      return 0;
    }

    int64_t Distance(const RoutingIndexManager::NodeIndex i,
                     const RoutingIndexManager::NodeIndex j) const {
      CheckNodeIsValid(i);
      CheckNodeIsValid(j);
      if (vehicle_indices[i.value()] == -1 || vehicle_indices[j.value()] == -1)
        return 0;
      if (i != Start() && j != Stop() && max_ride_distance_ > 0 &&
          data->distances_matrices_[problem_matrix_index].Cost(
              RoutingIndexManager::NodeIndex(vehicle_indices[i.value()]),
              RoutingIndexManager::NodeIndex(vehicle_indices[j.value()])) >
              max_ride_distance_)
        return CUSTOM_MAX_INT;
      return data->distances_matrices_[problem_matrix_index].Cost(
          RoutingIndexManager::NodeIndex(vehicle_indices[i.value()]),
          RoutingIndexManager::NodeIndex(vehicle_indices[j.value()]));
    }

    int64_t FakeDistance(const RoutingIndexManager::NodeIndex i,
                         const RoutingIndexManager::NodeIndex j) const {
      CheckNodeIsValid(i);
      CheckNodeIsValid(j);
      if (vehicle_indices[i.value()] == -1 || vehicle_indices[j.value()] == -1 ||
          (i == Start() && free_approach) || (j == Stop() && free_return))
        return 0;
      if (i != Start() && j != Stop() && max_ride_distance_ > 0 &&
          data->distances_matrices_[problem_matrix_index].Cost(
              RoutingIndexManager::NodeIndex(vehicle_indices[i.value()]),
              RoutingIndexManager::NodeIndex(vehicle_indices[j.value()])) >
              max_ride_distance_)
        return CUSTOM_MAX_INT;
      return data->distances_matrices_[problem_matrix_index].Cost(
          RoutingIndexManager::NodeIndex(vehicle_indices[i.value()]),
          RoutingIndexManager::NodeIndex(vehicle_indices[j.value()]));
    }

    int64_t Time(const RoutingIndexManager::NodeIndex i,
                 const RoutingIndexManager::NodeIndex j) const {
      CheckNodeIsValid(i);
      CheckNodeIsValid(j);
      if (vehicle_indices[i.value()] == -1 || vehicle_indices[j.value()] == -1)
        return 0;
      if (i != Start() && j != Stop() && max_ride_time_ > 0 &&
          data->times_matrices_[problem_matrix_index].Cost(
              RoutingIndexManager::NodeIndex(vehicle_indices[i.value()]),
              RoutingIndexManager::NodeIndex(vehicle_indices[j.value()])) >
              max_ride_time_)
        return CUSTOM_MAX_INT;
      return data->times_matrices_[problem_matrix_index].Cost(
          RoutingIndexManager::NodeIndex(vehicle_indices[i.value()]),
          RoutingIndexManager::NodeIndex(vehicle_indices[j.value()]));
    }

    int64_t FakeTime(const RoutingIndexManager::NodeIndex i,
                     const RoutingIndexManager::NodeIndex j) const {
      CheckNodeIsValid(i);
      CheckNodeIsValid(j);
      if (vehicle_indices[i.value()] == -1 || vehicle_indices[j.value()] == -1 ||
          (i == Start() && free_approach) || (j == Stop() && free_return))
        return 0;
      if (i != Start() && j != Stop() && max_ride_time_ > 0 &&
          data->times_matrices_[problem_matrix_index].Cost(
              RoutingIndexManager::NodeIndex(vehicle_indices[i.value()]),
              RoutingIndexManager::NodeIndex(vehicle_indices[j.value()])) >
              max_ride_time_)
        return CUSTOM_MAX_INT;
      return data->times_matrices_[problem_matrix_index].Cost(
          RoutingIndexManager::NodeIndex(vehicle_indices[i.value()]),
          RoutingIndexManager::NodeIndex(vehicle_indices[j.value()]));
    }

    int64_t Value(const RoutingIndexManager::NodeIndex i,
                  const RoutingIndexManager::NodeIndex j) const {
      CheckNodeIsValid(i);
      CheckNodeIsValid(j);
      if (vehicle_indices[i.value()] == -1 || vehicle_indices[j.value()] == -1)
        return 0;
      return data->values_matrices_[value_matrix_index].Cost(
          RoutingIndexManager::NodeIndex(vehicle_indices[i.value()]),
          RoutingIndexManager::NodeIndex(vehicle_indices[j.value()]));
    }

    int64_t TimeOrder(const RoutingIndexManager::NodeIndex i,
                      const RoutingIndexManager::NodeIndex j) const {
      CheckNodeIsValid(i);
      CheckNodeIsValid(j);
      if (vehicle_indices[i.value()] == -1 || vehicle_indices[j.value()] == -1)
        return 0;
      return 10 * std::sqrt(data->times_matrices_[problem_matrix_index].Cost(
                      RoutingIndexManager::NodeIndex(vehicle_indices[i.value()]),
                      RoutingIndexManager::NodeIndex(vehicle_indices[j.value()])));
    }

    int64_t DistanceOrder(const RoutingIndexManager::NodeIndex i,
                          const RoutingIndexManager::NodeIndex j) const {
      CheckNodeIsValid(i);
      CheckNodeIsValid(j);
      if (vehicle_indices[i.value()] == -1 || vehicle_indices[j.value()] == -1)
        return 0;
      return 100 * std::sqrt(data->distances_matrices_[problem_matrix_index].Cost(
                       RoutingIndexManager::NodeIndex(vehicle_indices[i.value()]),
                       RoutingIndexManager::NodeIndex(vehicle_indices[j.value()])));
    }

    //  Transit quantity at a node "from"
    //  This is the quantity added after visiting node "from"
    int64_t TimePlusServiceTime(const RoutingIndexManager::NodeIndex from,
                              const RoutingIndexManager::NodeIndex to) const {
      std::string from_id =
          from.value() > data->SizeMissions() ? start_point_id : data->PointId(from);
      int64_t current_time = Time(from, to) + coef_service * data->ServiceTime(from) +
                           additional_service +
                           (from_id != data->PointId(to)
                                ? coef_setup * data->SetupTime(to) +
                                      (data->SetupTime(to) > 0 ? additional_setup : 0)
                                : 0);

      // In case of order or sequence relations having no duration
      // will violate relations as the cumul_var will be the same.
      // Moreover with sequence+shipment lead or-tools to try only
      // invalid order of nodes
      if (current_time == 0 && data->SizeAlternativeRelations() > 0 && to.value() < data->SizeMissions()) {
        ++current_time;
      }

      return current_time;
      // FIXME:
      // (Time(from, to) == 0 ? 0
      // and
      // data->SetupTime(from) > 0 ? additional_setup
      // logics can be incorporated in data->SetupTime(from), with a wrapper function
      // called data->SetupTime(from, to)
    }

    int64_t FakeTimePlusServiceTime(const RoutingIndexManager::NodeIndex from,
                                  const RoutingIndexManager::NodeIndex to) const {
      std::string from_id =
          from.value() > data->SizeMissions() ? start_point_id : data->PointId(from);
      return FakeTime(from, to) + coef_service * data->ServiceTime(from) +
             additional_service +
             (from_id != data->PointId(to)
                  ? coef_setup * data->SetupTime(to) +
                        (data->SetupTime(to) > 0 ? additional_setup : 0)
                  : 0);
    }

    int64_t ValuePlusServiceValue(const RoutingIndexManager::NodeIndex from,
                                  const RoutingIndexManager::NodeIndex to) const {
      return Time(from, to) + data->ServiceValue(from);
    }

    int64_t TimePlus(const RoutingIndexManager::NodeIndex from,
                     const RoutingIndexManager::NodeIndex to) const {
      return Time(from, to);
    }

    RoutingIndexManager::NodeIndex Start() const { return start; }

    RoutingIndexManager::NodeIndex Stop() const { return stop; }

    inline void CheckNodeIsValid(const RoutingIndexManager::NodeIndex i) const {
      DCHECK_GE(i.value(), 0) << "Internal node " << i.value()
                              << " should be greater than or equal to 0!";
      DCHECK_LT(i.value(), size)
          << "Internal node " << i.value() << " should be less than " << size;
    }

    const std::vector<Rest>& Rests() const { return rests; }

    const TSPTWDataDT* const data;
    std::string id;
    int64_t vehicle_index;
    int32_t size;
    int32_t size_matrix;
    int32_t size_rest;
    RoutingIndexManager::NodeIndex start;
    RoutingIndexManager::NodeIndex stop;
    int64_t problem_matrix_index;
    int64_t value_matrix_index;
    std::string start_point_id;
    std::vector<int64_t> vehicle_indices;
    std::vector<int64_t> initial_capacity;
    std::vector<int64_t> initial_load;
    std::vector<int64_t> capacity;
    std::vector<bool> counting;
    std::vector<int64_t> overload_multiplier;
    std::vector<Rest> rests;
    int32_t break_size;
    int64_t time_start;
    int64_t time_end;
    int64_t time_maximum_lateness;
    int64_t late_multiplier;
    int64_t cost_fixed;
    int64_t cost_distance_multiplier;
    int64_t cost_time_multiplier;
    int64_t cost_waiting_time_multiplier;
    int64_t cost_value_multiplier;
    float coef_service;
    int64_t additional_service;
    float coef_setup;
    int64_t additional_setup;
    int64_t duration;
    int64_t distance;
    ShiftPref shift_preference;
    int32_t day_index;
    int64_t max_ride_time_;
    int64_t max_ride_distance_;
    bool free_approach;
    bool free_return;
  };

  const std::vector<Vehicle>& Vehicles() const { return tsptw_vehicles_; }

  const Vehicle& Vehicles(const int64_t index) const { return tsptw_vehicles_[index]; }

  bool VehicleHasEnd(const int64_t index) const {
    return tsptw_vehicles_[index].time_end < CUSTOM_MAX_INT;
  }

  bool AllVehiclesHaveEnd() {
    for (std::size_t v = 0; v < tsptw_vehicles_.size(); v++) {
      if (!VehicleHasEnd(v))
        return false;
    }
    return true;
  }

  struct Route {
    Route(std::string v_id) : vehicle_id(v_id), vehicle_index(-1) {}
    Route(std::string v_id, int v_int, std::vector<std::string> s_ids)
        : vehicle_id(v_id), vehicle_index(v_int), service_ids(s_ids) {}
    std::string vehicle_id;
    int vehicle_index;
    std::vector<std::string> service_ids;
  };

  const std::vector<Route>& Routes() const { return tsptw_routes_; }

  std::vector<Route>* Routes() { return &tsptw_routes_; }

  struct Relation {
    Relation(int relation_no)
        : relation_number(relation_no)
        , type(Order)
        , linked_ids()
        , linked_vehicle_ids()
        , lapse(-1) {}
    Relation(int relation_no, RelationType t,
             const google::protobuf::RepeatedPtrField<std::string>& l_i)
        : relation_number(relation_no)
        , type(t)
        , linked_ids(l_i)
        , linked_vehicle_ids()
        , lapse(-1) {}
    Relation(int relation_no, RelationType t,
             const google::protobuf::RepeatedPtrField<std::string>& l_i,
             const google::protobuf::RepeatedPtrField<std::string>& l_v_i, int32_t l)
        : relation_number(relation_no)
        , type(t)
        , linked_ids(l_i)
        , linked_vehicle_ids(l_v_i)
        , lapse(l) {}

    int relation_number;
    RelationType type;
    const google::protobuf::RepeatedPtrField<std::string> linked_ids;
    const google::protobuf::RepeatedPtrField<std::string> linked_vehicle_ids;
    int32_t lapse;
  };

  const std::vector<Relation>& Relations() const { return tsptw_relations_; }

  const std::vector<int>& VehiclesDay() const { return vehicles_day_; }

  int VehicleDay(const int64_t index) const {
    if (index < 0) {
      return -1;
    }
    return vehicles_day_[index];
  }

  int VehicleDayAlt(const int64_t index) const {
    if (index < 0) {
      return CUSTOM_MAX_INT;
    }
    return vehicles_day_[index];
  }

private:
  void ProcessNewLine(char* const line);

  struct TSPTWClient {
    // Depot definition
    TSPTWClient(std::string cust_id, std::string p_id, int32_t m_i, int32_t p_i)
        : customer_id(cust_id)
        , point_id(p_id)
        , matrix_index(m_i)
        , problem_index(p_i)
        , alternative_index(0)
        , ready_time({-CUSTOM_MAX_INT})
        , due_time({CUSTOM_MAX_INT})
        , maximum_lateness({CUSTOM_MAX_INT})
        , service_time(0.0)
        , service_value(0.0)
        , setup_time(0.0)
        , priority(4)
        , late_multiplier(0)
        , is_break(false) {}
    // Mission definition
    TSPTWClient(std::string cust_id, std::string p_id, int32_t m_i, int32_t p_i, int32_t a_i,
                std::vector<int64_t> r_t, std::vector<int64_t> d_t,
                std::vector<int64_t>& max_lateness, double s_t, double s_v, double st_t,
                int32_t p_t, double l_m, std::vector<int64_t>& v_i, std::vector<int64_t>& q,
                std::vector<int64_t>& s_q, int64_t e_c, std::vector<bool>& r_q)
        : customer_id(cust_id)
        , point_id(p_id)
        , matrix_index(m_i)
        , problem_index(p_i)
        , alternative_index(a_i)
        , ready_time(r_t)
        , due_time(d_t)
        , maximum_lateness(max_lateness)
        , service_time(s_t)
        , service_value(s_v)
        , setup_time(st_t)
        , priority(p_t)
        , late_multiplier(l_m)
        , vehicle_indices(v_i)
        , quantities(q)
        , setup_quantities(s_q)
        , exclusion_cost(e_c)
        , refill_quantities(r_q)
        , is_break(false) {}
    std::string customer_id;
    std::string point_id;
    int32_t matrix_index;
    int32_t problem_index;
    int32_t alternative_index;
    std::vector<int64_t> ready_time;
    std::vector<int64_t> due_time;
    std::vector<int64_t> maximum_lateness;
    int64_t service_time;
    int64_t service_value;
    int64_t setup_time;
    int64_t priority;
    int64_t late_multiplier;
    std::vector<int64_t> vehicle_indices;
    std::vector<int64_t> quantities;
    std::vector<int64_t> setup_quantities;
    int64_t exclusion_cost;
    std::vector<bool> refill_quantities;
    bool is_break;
  };

  int32_t size_problem_;
  int32_t size_;
  int32_t size_matrix_;
  int32_t size_missions_;
  int32_t size_rest_;
  int32_t size_alternative_relations_;
  int64_t deliveries_counter_;
  int64_t horizon_;
  int64_t earliest_start_;
  int64_t max_distance_;
  int64_t max_distance_cost_;
  int64_t max_rest_;
  int64_t max_service_;
  int64_t max_time_;
  int64_t max_time_cost_;
  int64_t max_value_;
  int64_t max_value_cost_;
  int64_t multiple_tws_counter_;
  int64_t sum_max_time_;
  int64_t tws_counter_;
  float max_coef_service_;
  float max_coef_setup_;
  std::vector<int32_t> tws_size_;
  std::vector<Vehicle> tsptw_vehicles_;
  std::vector<Relation> tsptw_relations_;
  std::vector<TSPTWClient> tsptw_clients_;
  std::map<int32_t, int32_t> alternative_size_map_;
  std::vector<Route> tsptw_routes_;
  std::vector<CompleteGraphArcCost> distances_matrices_;
  std::vector<CompleteGraphArcCost> times_matrices_;
  std::vector<CompleteGraphArcCost> values_matrices_;
  std::vector<int> vehicles_day_;
  std::vector<int64_t> service_times_;
  std::string details_;
  std::map<std::string, int64_t> ids_map_;
  std::map<std::string, int64_t> vehicle_ids_map_;
  std::map<int64_t, int64_t> day_index_to_vehicle_index_;
};

void TSPTWDataDT::LoadInstance(const std::string& filename) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ortools_vrp::Problem problem;

  {
    std::fstream input(filename, std::ios::in | std::ios::binary);
    if (!problem.ParseFromIstream(&input)) {
      LOG(FATAL) << "Failed to parse pbf.";
    }
  }

  // compute earliest start first
  for (const ortools_vrp::Vehicle& vehicle : problem.vehicles()) {
    earliest_start_ = std::min((int64_t) vehicle.time_window().start(), earliest_start_);
  }

  int32_t node_index           = 0;
  int32_t matrix_index         = 0;
  int32_t previous_matrix_size = 0;
  std::vector<int64_t> matrix_indices;
  for (const ortools_vrp::Service& service : problem.services()) {
    if (!alternative_size_map_.count(service.problem_index()))
      alternative_size_map_[service.problem_index()] = 0;
    const int32_t tws_size = service.time_windows_size();
    tws_size_.push_back(tws_size);
    std::vector<const ortools_vrp::TimeWindow*> timewindows;
    for (int32_t tw = 0; tw < tws_size; ++tw) {
      timewindows.push_back(&service.time_windows().Get(tw));
    }

    std::vector<int64_t> q;
    for (const float& quantity : service.quantities()) {
      if (quantity < 0)
        ++deliveries_counter_;
      q.push_back(std::round(quantity * CUSTOM_BIGNUM_QUANTITY));
    }

    std::vector<int64_t> s_q;
    for (const float& setup_quantity : service.setup_quantities()) {
      s_q.push_back(std::round(setup_quantity * CUSTOM_BIGNUM_QUANTITY));
    }

    std::vector<bool> r_q;
    for (const bool refill : service.refill_quantities()) {
      r_q.push_back(refill);
    }

    std::vector<int64_t> v_i;
    for (const int64_t& index : service.vehicle_indices()) {
      v_i.push_back(index);
    }

    tws_counter_ += timewindows.size();

    if (timewindows.size() > 1)
      multiple_tws_counter_ += 1;

    int timewindow_index = 0;

    if (service.late_multiplier() > 0) {
      do {
        if (timewindows.size() == 0 ||
            (earliest_start_ < timewindows[timewindow_index]->end() +
                                   timewindows[timewindow_index]->maximum_lateness())) {
          matrix_indices.push_back(service.matrix_index());
          std::vector<int64_t> start;
          if (timewindows.size() > 0 &&
              (timewindows[timewindow_index]->start() - earliest_start_) > 0)
            start.push_back(timewindows[timewindow_index]->start() - earliest_start_);
          else
            start.push_back(0);

          std::vector<int64_t> end;
          if (timewindows.size() > 0 &&
              (timewindows[timewindow_index]->end() - earliest_start_) < CUSTOM_MAX_INT)
            end.push_back(timewindows[timewindow_index]->end() - earliest_start_);
          else
            end.push_back(CUSTOM_MAX_INT);

          std::vector<int64_t> max_lateness;
          if (timewindows.size() > 0 &&
              timewindows[timewindow_index]->maximum_lateness() < CUSTOM_MAX_INT)
            max_lateness.push_back(timewindows[timewindow_index]->maximum_lateness());
          else
            max_lateness.push_back(CUSTOM_MAX_INT);

          size_problem_ = std::max(size_problem_, (int32_t)service.problem_index());
          tsptw_clients_.push_back(TSPTWClient(
              (std::string)service.id(), (std::string)service.point_id(), matrix_index,
              service.problem_index(), alternative_size_map_[service.problem_index()],
              start, end, max_lateness, service.duration(), service.additional_value(),
              service.setup_duration(), service.priority(),
              start.size() > 0 ? (int64_t)(service.late_multiplier() * CUSTOM_BIGNUM_COST)
                               : 0,
              v_i, q, s_q,
              service.exclusion_cost() > 0 ? service.exclusion_cost() * CUSTOM_BIGNUM_COST
                                           : -1,
              r_q));

          service_times_.push_back(service.duration());
          alternative_size_map_[service.problem_index()] += 1;
          if (ids_map_.find((std::string)service.id()) == ids_map_.end())
            ids_map_[(std::string)service.id()] = node_index;
          node_index++;
        }
        ++timewindow_index;
      } while (timewindow_index < service.time_windows_size());
    } else {
      std::vector<int64_t> ready_time;
      std::vector<int64_t> due_time;
      std::vector<int64_t> max_lateness;

      for (const ortools_vrp::TimeWindow* timewindow : timewindows) {
        if (earliest_start_ < timewindow->end()) {
          (timewindow->start() - earliest_start_) > 0
              ? ready_time.push_back(timewindow->start() - earliest_start_)
              : ready_time.push_back(0);
          (timewindow->end() - earliest_start_) < CUSTOM_MAX_INT
              ? due_time.push_back(timewindow->end() - earliest_start_)
              : due_time.push_back(CUSTOM_MAX_INT);
          timewindow->maximum_lateness() < CUSTOM_MAX_INT
              ? max_lateness.push_back(timewindow->maximum_lateness())
              : max_lateness.push_back(CUSTOM_MAX_INT);
        }
      }

      matrix_indices.push_back(service.matrix_index());
      size_problem_ = std::max(size_problem_, (int32_t)service.problem_index());
      tsptw_clients_.push_back(TSPTWClient(
          (std::string)service.id(), (std::string)service.point_id(), matrix_index,
          service.problem_index(), alternative_size_map_[service.problem_index()],
          ready_time, due_time, max_lateness, service.duration(),
          service.additional_value(), service.setup_duration(), service.priority(),
          ready_time.size() > 0 ? (int64_t)(service.late_multiplier() * CUSTOM_BIGNUM_COST)
                                : 0,
          v_i, q, s_q,
          service.exclusion_cost() > 0 ? service.exclusion_cost() * CUSTOM_BIGNUM_COST
                                       : -1,
          r_q));
      service_times_.push_back(service.duration());
      alternative_size_map_[service.problem_index()] += 1;
      if (ids_map_.find((std::string)service.id()) == ids_map_.end())
        ids_map_[(std::string)service.id()] = node_index;
      node_index++;
    }
    if (previous_matrix_size == (int32)matrix_indices.size()) {
      throw std::invalid_argument(
          "A Service transmitted should always lead to at least one Node");
    }
    previous_matrix_size = matrix_indices.size();
    ++matrix_index;
  }

  for (const ortools_vrp::Vehicle& vehicle : problem.vehicles()) {
    service_times_.push_back(0);
    service_times_.push_back(0);
    size_rest_ += vehicle.rests().size();
  }
  size_matrix_ = matrix_index + 2;

  size_missions_ = node_index;
  size_          = node_index + 2;

  for (const ortools_vrp::Matrix& matrix : problem.matrices()) {
    // + 2 In case vehicles have no depots
    int32_t problem_size =
        std::max(std::max(sqrt(matrix.distance_size()), sqrt(matrix.time_size())),
                 sqrt(matrix.value_size())) +
        2 + (size_rest_ > 0 ? 1 : 0);

    distances_matrices_.emplace_back(std::max(problem_size, 3));
    times_matrices_.emplace_back(std::max(problem_size, 3));
    values_matrices_.emplace_back(std::max(problem_size, 3));

    // Matrix default values
    for (int64_t i = 0; i < std::max(problem_size, 3); ++i) {
      for (int64_t j = 0; j < std::max(problem_size, 3); ++j) {
        SetTimeMatrix(i, j)  = 0;
        SetDistMatrix(i, j)  = 0;
        SetValueMatrix(i, j) = 0;
      }
    }

    if (matrix.time_size() > 0) {
      max_time_ = std::max(max_time_, BuildTimeMatrix(matrix));
    }

    // Estimate necessary horizon due to time matrix
    std::vector<int64_t> max_times(MaxTimes(matrix));
    int64_t matrix_sum_time = 0;
    if (sqrt(matrix.time_size()) > 0) {
      for (std::size_t i = 0; i < matrix_indices.size(); i++) {
        matrix_sum_time += max_times.at(matrix_indices[i]);
      }
    }
    sum_max_time_ = std::max(sum_max_time_, matrix_sum_time);

    if (matrix.distance_size() > 0) {
      max_distance_ = std::max(max_distance_, BuildDistanceMatrix(matrix));
    }

    if (matrix.value_size() > 0) {
      max_value_ = std::max(max_value_, BuildValueMatrix(matrix));
    }
  }

  // Approximate depot time need
  sum_max_time_ += 2 * max_time_;

  int64_t current_day_index      = 0;
  int v_idx                      = 0;
  day_index_to_vehicle_index_[0] = v_idx;
  for (const ortools_vrp::Vehicle& vehicle : problem.vehicles()) {
    if (!vehicle.has_time_window()) {
      throw std::invalid_argument(
          "A vehicle should always have an initialized timewindow");
    }

    tsptw_vehicles_.emplace_back(this, size_);
    auto v = tsptw_vehicles_.rbegin();

    // Every vehicle has its own matrix definition
    std::vector<int64_t> vehicle_indices(matrix_indices);
    vehicle_indices.push_back(vehicle.start_index());
    vehicle_indices.push_back(vehicle.end_index());

    for (const ortools_vrp::Capacity& capacity : vehicle.capacities()) {
      v->capacity.push_back(std::round(capacity.limit() * CUSTOM_BIGNUM_QUANTITY));
      v->initial_capacity.push_back(
          std::round(capacity.initial_limit() * CUSTOM_BIGNUM_QUANTITY));
      v->initial_load.push_back(
          std::round(capacity.initial_load() * CUSTOM_BIGNUM_QUANTITY));
      // quantities and capacities are multiplied with CUSTOM_BIGNUM_QUANTITY so divide
      // the CUSTOM_BIGNUM_COST by CUSTOM_BIGNUM_QUANTITY so that the cost will be correct
      // when it is divided by CUSTOM_BIGNUM_COST
      v->overload_multiplier.push_back(std::round(
          capacity.overload_multiplier() * CUSTOM_BIGNUM_COST / CUSTOM_BIGNUM_QUANTITY));
      v->counting.push_back(capacity.counting());
    }

    v->id                   = vehicle.id();
    v->vehicle_index        = v_idx;
    v->break_size           = vehicle.rests().size();
    v->problem_matrix_index = vehicle.matrix_index();
    v->value_matrix_index   = vehicle.value_matrix_index();
    v->start_point_id        = vehicle.start_point_id();
    v->vehicle_indices      = vehicle_indices;
    v->time_start           = (vehicle.time_window().start() - earliest_start_) > 0
                        ? vehicle.time_window().start() - earliest_start_
                        : 0;
    v->time_end = (vehicle.time_window().end() - earliest_start_) < CUSTOM_MAX_INT
                      ? vehicle.time_window().end() - earliest_start_
                      : CUSTOM_MAX_INT;
    v->time_maximum_lateness = vehicle.time_window().maximum_lateness() < CUSTOM_MAX_INT
                                   ? vehicle.time_window().maximum_lateness()
                                   : CUSTOM_MAX_INT;
    v->late_multiplier = (int64_t)(vehicle.cost_late_multiplier() * CUSTOM_BIGNUM_COST);
    v->cost_fixed      = (int64_t)(vehicle.cost_fixed() * CUSTOM_BIGNUM_COST);
    v->cost_distance_multiplier =
        (int64_t)(vehicle.cost_distance_multiplier() * CUSTOM_BIGNUM_COST);
    v->cost_time_multiplier =
        (int64_t)(vehicle.cost_time_multiplier() * CUSTOM_BIGNUM_COST);
    v->cost_waiting_time_multiplier =
        (int64_t)(vehicle.cost_waiting_time_multiplier() * CUSTOM_BIGNUM_COST);
    v->cost_value_multiplier =
        (int64_t)(vehicle.cost_value_multiplier() * CUSTOM_BIGNUM_COST);
    v->coef_service   = vehicle.coef_service();
    max_coef_service_ = std::max(max_coef_service_, v->coef_service);

    v->additional_service = vehicle.additional_service();
    v->coef_setup         = vehicle.coef_setup();
    max_coef_setup_       = std::max(max_coef_setup_, v->coef_setup);

    v->additional_setup = vehicle.additional_setup();
    v->duration         = (int64_t)(vehicle.duration());
    v->distance         = vehicle.distance();
    v->free_approach    = vehicle.free_approach();
    v->free_return      = vehicle.free_return();
    if (vehicle.shift_preference().compare("force_start") == 0)
      v->shift_preference = ForceStart;
    else if (vehicle.shift_preference().compare("force_end") == 0)
      v->shift_preference = ForceEnd;
    else
      v->shift_preference = MinimizeSpan;
    v->day_index          = vehicle.day_index();
    v->max_ride_time_     = vehicle.max_ride_time();
    v->max_ride_distance_ = vehicle.max_ride_distance();
    vehicles_day_.push_back(vehicle.day_index());

    max_distance_cost_ = std::max(max_distance_cost_, v->cost_distance_multiplier);
    max_time_cost_     = std::max(max_time_cost_, v->cost_time_multiplier);
    max_value_cost_    = std::max(max_value_cost_, v->cost_value_multiplier);

    vehicle_ids_map_[(std::string)vehicle.id()] = v_idx;
    if (current_day_index < vehicle.day_index()) {
      do {
        ++current_day_index;
        day_index_to_vehicle_index_[current_day_index] = v_idx;
      } while (current_day_index < vehicle.day_index());
    }

    // Add vehicle rests
    for (const ortools_vrp::Rest& rest : vehicle.rests()) {
      v->rests.emplace_back((std::string)rest.id(),
                            rest.time_window().start() - earliest_start_,
                            rest.time_window().end() - earliest_start_, rest.duration());
    }

    v_idx++;
  }

  for (const ortools_vrp::Route& route : problem.routes()) {
    Route r(route.vehicle_id());
    for (std::size_t i = 0; i < tsptw_vehicles_.size(); ++i) {
      if (tsptw_vehicles_[i].id == route.vehicle_id())
        r.vehicle_index = i;
    }
    for (std::string service_id : route.service_ids()) {
      r.service_ids.push_back(service_id);
    }
    tsptw_routes_.push_back(r);
  }

  int re_index      = 0;
  int64_t sum_lapse = 0;
  // Setting start
  for (Vehicle& v : tsptw_vehicles_) {
    v.start = RoutingIndexManager::NodeIndex(node_index);
  }
  tsptw_clients_.push_back(
      TSPTWClient("vehicles_start", "vehicles_start", matrix_index, node_index));
  service_times_.push_back(0);

  node_index++;
  // Setting stop
  for (Vehicle& v : tsptw_vehicles_) {
    v.stop = RoutingIndexManager::NodeIndex(node_index);
  }
  // node_index++;
  tsptw_clients_.push_back(
      TSPTWClient("vehicles_end", "vehicles_end", ++matrix_index, node_index));
  service_times_.push_back(0);

  for (const ortools_vrp::Relation& relation : problem.relations()) {
    RelationType relType;
    if (relation.type() == "sequence") {
      ++size_alternative_relations_;
      relType = Sequence;
    } else if (relation.type() == "order") {
      ++size_alternative_relations_;
      relType = Order;
    } else if (relation.type() == "same_route") {
      ++size_alternative_relations_;
      relType = SameRoute;
    } else if (relation.type() == "minimum_day_lapse")
      relType = MinimumDayLapse;
    else if (relation.type() == "maximum_day_lapse")
      relType = MaximumDayLapse;
    else if (relation.type() == "shipment") {
      ++size_alternative_relations_;
      relType = Shipment;
    } else if (relation.type() == "meetup")
      relType = MeetUp;
    else if (relation.type() == "maximum_duration_lapse")
      relType = MaximumDurationLapse;
    else if (relation.type() == "force_first") {
      ++size_alternative_relations_;
      relType = ForceFirst;
    } else if (relation.type() == "never_first") {
      ++size_alternative_relations_;
      relType = NeverFirst;
    } else if (relation.type() == "never_last") {
      ++size_alternative_relations_;
      relType = NeverLast;
    } else if (relation.type() == "force_end") {
      ++size_alternative_relations_;
      relType = ForceLast;
    } else if (relation.type() == "vehicle_group_duration")
      relType = VehicleGroupDuration;
    else if (relation.type() == "vehicle_trips") {
      relType = VehicleTrips;
      sum_max_time_ += 2 * relation.linked_vehicle_ids().size() * max_time_;
    } else if (relation.type() == "vehicle_group_number")
      relType = VehicleGroupNumber;
    else if (relation.type() == "minimum_duration_lapse")
      relType = MinimumDurationLapse;
    else
      throw std::invalid_argument("Unknown relation type");

    sum_lapse += relation.lapse();
    tsptw_relations_.emplace_back(re_index, relType, relation.linked_ids(),
                                  relation.linked_vehicle_ids(), relation.lapse());
    ++re_index;
  }

  // Compute horizon
  int64_t rest_duration;
  for (std::size_t v = 0; v < tsptw_vehicles_.size(); v++) {
    rest_duration = 0;
    for (std::size_t r = 0; r < tsptw_vehicles_[v].Rests().size(); r++) {
      rest_duration += tsptw_vehicles_[v].Rests()[r].duration;
    }
    max_rest_ = std::max(max_rest_, rest_duration);
  }
  if (AllVehiclesHaveEnd()) {
    for (std::size_t v = 0; v < tsptw_vehicles_.size(); v++) {
      horizon_ = std::max(horizon_, tsptw_vehicles_[v].time_end +
                                        tsptw_vehicles_[v].time_maximum_lateness);
    }
  } else if (AllServicesHaveEnd()) {
    for (std::size_t i = 0; i < tsptw_clients_.size(); i++) {
      horizon_ = std::max(horizon_, tsptw_clients_[i].due_time.back() +
                                        tsptw_clients_[i].maximum_lateness.back());
    }
    for (std::size_t v = 0; v < tsptw_vehicles_.size(); v++) {
      for (std::size_t r = 0; r < tsptw_vehicles_[v].Rests().size(); r++) {
        horizon_ = std::max(horizon_, tsptw_vehicles_[v].Rests()[r].due_time);
      }
    }
  } else {
    int64_t latest_start    = 0;
    int64_t latest_rest_end = 0;
    int64_t sum_service     = 0;
    int64_t sum_setup       = 0;
    for (int32_t i = 0; i < size_missions_; ++i) {
      sum_service += tsptw_clients_[i].service_time;
      sum_setup += tsptw_clients_[i].setup_time;
      if (tsptw_clients_[i].ready_time.size() > 0) {
        latest_start = std::max(latest_start, tsptw_clients_[i].ready_time.back());
      }
    }
    for (std::size_t v = 0; v < tsptw_vehicles_.size(); v++) {
      latest_start = std::max(latest_start, tsptw_vehicles_[v].time_start);

      for (std::size_t r = 0; r < tsptw_vehicles_[v].Rests().size(); r++) {
        latest_rest_end =
            std::max(latest_rest_end, tsptw_vehicles_[v].Rests()[r].due_time);
      }
    }
    horizon_ = std::max(latest_start, latest_rest_end) + sum_service * max_coef_service_ +
               sum_setup * max_coef_setup_ + sum_max_time_ + sum_lapse + max_rest_;
  }

  if (size_alternative_relations_ > 0)
    horizon_ += size_missions_;

  for (int32_t i = 0; i < size_missions_; ++i) {
    max_service_ = std::max(max_service_, tsptw_clients_[i].service_time);
  }
}

} //  namespace operations_research

#endif //  OR_TOOLS_TUTORIALS_CPLUSPLUS_TSP_DATA_DT_H
