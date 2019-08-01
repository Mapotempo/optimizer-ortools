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

#define CUSTOM_MAX_INT (int64) std::pow(2, 30)

enum RelationType {
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
  explicit TSPTWDataDT(std::string filename) { LoadInstance(filename); }

  ~TSPTWDataDT() {
    for (auto i : tsptw_vehicles_)
      delete i;

    for (auto i : tsptw_relations_)
      delete i;

    for (auto i : distances_matrices_)
      delete i;

    for (auto i : times_matrices_)
      delete i;

    for (auto i : values_matrices_)
      delete i;

    for (auto i : tsptw_routes_)
      delete i;
  }
  void LoadInstance(const std::string& filename);

  //  Helper function
  int64& SetMatrix(int i, int j) {
    return distances_matrices_.back()->Cost(RoutingIndexManager::NodeIndex(i),
                                            RoutingIndexManager::NodeIndex(j));
  }

  int64& SetTimeMatrix(int i, int j) {
    return times_matrices_.back()->Cost(RoutingIndexManager::NodeIndex(i),
                                        RoutingIndexManager::NodeIndex(j));
  }

  int64& SetValueMatrix(int i, int j) {
    return values_matrices_.back()->Cost(RoutingIndexManager::NodeIndex(i),
                                         RoutingIndexManager::NodeIndex(j));
  }

  int64 BuildTimeMatrix(ortools_vrp::Matrix matrix) {
    int64 max_time    = 0;
    int32 size_matrix = sqrt(matrix.time_size());
    for (int64 i = 0; i < size_matrix; ++i) {
      for (int64 j = 0; j < size_matrix; ++j) {
        if (static_cast<int64>(matrix.time(i * size_matrix + j)) < CUSTOM_MAX_INT)
          max_time = std::max(max_time,
                              static_cast<int64>(matrix.time(i * size_matrix + j) + 0.5));
        // std::cout << matrix.time(i * size_matrix + j) << "->" <<
        // static_cast<int64>(matrix.time(i * size_matrix + j) + 0.5) << "\t | \t";
        SetTimeMatrix(i, j) = static_cast<int64>(matrix.time(i * size_matrix + j) + 0.5);
      }
      // std::cout << std::endl;
    }

    return max_time;
  }

  int64 BuildDistanceMatrix(ortools_vrp::Matrix matrix) {
    int64 max_distance = 0;
    int32 size_matrix  = sqrt(matrix.distance_size());
    for (int64 i = 0; i < size_matrix; ++i) {
      for (int64 j = 0; j < size_matrix; ++j) {
        if (static_cast<int64>(matrix.distance(i * size_matrix + j)) < CUSTOM_MAX_INT)
          max_distance = std::max(
              max_distance, static_cast<int64>(matrix.distance(i * size_matrix + j)));
        SetMatrix(i, j) = static_cast<int64>(matrix.distance(i * size_matrix + j));
      }
    }
    return max_distance;
  }

  int64 BuildValueMatrix(ortools_vrp::Matrix matrix) {
    int64 max_value   = 0;
    int32 size_matrix = sqrt(matrix.value_size());
    for (int64 i = 0; i < size_matrix; ++i) {
      for (int64 j = 0; j < size_matrix; ++j) {
        if (static_cast<int64>(matrix.value(i * size_matrix + j)) < CUSTOM_MAX_INT)
          max_value =
              std::max(max_value, static_cast<int64>(matrix.value(i * size_matrix + j)));
        SetValueMatrix(i, j) = static_cast<int64>(matrix.value(i * size_matrix + j));
      }
    }
    return max_value;
  }

  int64 Horizon() const { return horizon_; }

  int64 MatrixIndex(RoutingIndexManager::NodeIndex i) const {
    return tsptw_clients_[i.value()].matrix_index;
  }

  int64 MaxTime() const { return max_time_; }

  int64 MaxDistance() const { return max_distance_; }

  int64 MaxValue() const { return max_value_; }

  int64 MaxServiceTime() const { return max_service_; }

  int64 MaxTimeCost() const { return max_time_cost_; }

  int64 MaxDistanceCost() const { return max_distance_cost_; }

  int64 MaxValueCost() const { return max_value_cost_; }

  int64 TWsCounter() const { return tws_counter_; }

  int64 TwiceTWsCounter() const { return multiple_tws_counter_; }

  int64 OrderCounter() const { return order_counter_; }

  int64 DeliveriesCounter() const { return deliveries_counter_; }

  int64 IdIndex(std::string id) const {
    std::map<std::string, int64>::const_iterator it = ids_map_.find(id);
    if (it != ids_map_.end())
      return it->second;
    else
      return -1;
  }

  int64 VehicleIdIndex(std::string id) const {
    std::map<std::string, int64>::const_iterator it = vehicle_ids_map_.find(id);
    if (it != vehicle_ids_map_.end())
      return it->second;
    else
      return -1;
  }

  int64 DayIndexToVehicleIndex(int64 day_index) const {
    if (day_index_to_vehicle_index_.count(day_index)) {
      return day_index_to_vehicle_index_.at(day_index);
    }
    return CUSTOM_MAX_INT;
  }

  int32 AlternativeSize(int32 problem_index) const {
    if (alternative_size_map_.count(problem_index))
      return alternative_size_map_.at(problem_index);
    return -1;
  }

  std::string ServiceId(RoutingIndexManager::NodeIndex i) const {
    return tsptw_clients_[i.value()].customer_id;
  }

  int32 ProblemIndex(RoutingIndexManager::NodeIndex i) const {
    return tsptw_clients_[i.value()].problem_index;
  }

  int32 AlternativeIndex(RoutingIndexManager::NodeIndex i) const {
    return tsptw_clients_[i.value()].alternative_index;
  }

  std::vector<int64> ReadyTime(RoutingIndexManager::NodeIndex i) const {
    return tsptw_clients_[i.value()].ready_time;
  }

  std::vector<int64> DueTime(RoutingIndexManager::NodeIndex i) const {
    return tsptw_clients_[i.value()].due_time;
  }

  int64 LateMultiplier(RoutingIndexManager::NodeIndex i) const {
    return tsptw_clients_[i.value()].late_multiplier;
  }

  int64 ServiceTime(RoutingIndexManager::NodeIndex i) const {
    return tsptw_clients_[i.value()].service_time;
  }

  int64 ServiceValue(RoutingIndexManager::NodeIndex i) const {
    return tsptw_clients_[i.value()].service_value;
  }

  int64 SetupTime(RoutingIndexManager::NodeIndex i) const {
    return tsptw_clients_[i.value()].setup_time;
  }

  int64 Priority(RoutingIndexManager::NodeIndex i) const {
    return (int64)tsptw_clients_[i.value()].priority;
  }

  int64 ExclusionCost(RoutingIndexManager::NodeIndex i) const {
    return (int64)tsptw_clients_[i.value()].exclusion_cost;
  }

  std::vector<int64> VehicleIndices(RoutingIndexManager::NodeIndex i) const {
    return tsptw_clients_[i.value()].vehicle_indices;
  }

  int32 TimeWindowsSize(int i) const { return tws_size_.at(i); }

  int32 Size() const { return size_; }

  int32 SizeMissions() const { return size_missions_; }

  int32 SizeMatrix() const { return size_matrix_; }

  int32 SizeProblem() const { return size_problem_; }

  int32 SizeRest() const { return size_rest_; }

  std::vector<bool> RefillQuantities(RoutingIndexManager::NodeIndex i) const {
    return tsptw_clients_[i.value()].refill_quantities;
  }

  int64 Quantity(std::size_t unit_i, RoutingIndexManager::NodeIndex from,
                 RoutingIndexManager::NodeIndex to) const {
    //    CheckNodeIsValid(from);
    //    CheckNodeIsValid(to);
    int64 index = from.value();
    if (unit_i < tsptw_clients_.at(index).quantities.size()) {
      if (tsptw_vehicles_[0]->counting.at(unit_i)) {
        if (tsptw_vehicles_[0]->stop == to ||
            tsptw_vehicles_[0]->Distance(from, to) > 0 ||
            tsptw_vehicles_[0]->Time(from, to) > 0)
          return tsptw_clients_.at(index).quantities.at(unit_i) -
                 tsptw_clients_.at(index).setup_quantities.at(unit_i);
        else
          return tsptw_clients_.at(index).quantities.at(unit_i);
      }
      return tsptw_clients_.at(index).quantities.at(unit_i);
    } else {
      return 0;
    }
  }

  std::vector<int64> Quantities(RoutingIndexManager::NodeIndex i) const {
    return tsptw_clients_[i.value()].quantities;
  }

  struct Vehicle {
    Vehicle(TSPTWDataDT* data_, int32 size_)
        : data(data_)
        , size(size_)
        , problem_matrix_index(0)
        , value_matrix_index(0)
        , vehicle_indices(0)
        , capacity(0)
        , overload_multiplier(0)
        , break_size(0)
        , time_start(0)
        , time_end(0)
        , late_multiplier(0) {}

    int32 SizeMatrix() const { return size_matrix; }

    int32 SizeRest() const { return size_rest; }

    void SetStart(RoutingIndexManager::NodeIndex s) {
      CHECK_LT(s, size);
      start = s;
    }

    void SetStop(RoutingIndexManager::NodeIndex s) {
      CHECK_LT(s, size);
      stop = s;
    }

    int64 ReturnZero(RoutingIndexManager::NodeIndex,
                     RoutingIndexManager::NodeIndex) const {
      return 0;
    }

    int64 Distance(RoutingIndexManager::NodeIndex i,
                   RoutingIndexManager::NodeIndex j) const {
      CheckNodeIsValid(i);
      CheckNodeIsValid(j);
      if (vehicle_indices[i.value()] == -1 || vehicle_indices[j.value()] == -1)
        return 0;
      if (j.value() >= data->SizeMissions() && j != Stop() && j != Start()) {
        IntVar* next_var = routing->NextVar(j.value());
        if (next_var->Bound() && next_var->Value() != j.value() &&
            next_var->Value() != i.value() && next_var->Value() < data->SizeMissions()) {
          return Distance(i, (RoutingIndexManager::NodeIndex)next_var->Value());
        } else {
          return Distance(i, Stop());
        }
      }
      if (i != Start() && j != Stop() && max_ride_distance_ > 0 &&
          data->distances_matrices_.at(problem_matrix_index)
                  ->Cost(RoutingIndexManager::NodeIndex(vehicle_indices[i.value()]),
                         RoutingIndexManager::NodeIndex(vehicle_indices[j.value()])) >
              max_ride_distance_)
        return CUSTOM_MAX_INT;
      return data->distances_matrices_.at(problem_matrix_index)
          ->Cost(RoutingIndexManager::NodeIndex(vehicle_indices[i.value()]),
                 RoutingIndexManager::NodeIndex(vehicle_indices[j.value()]));
    }

    int64 FakeDistance(RoutingIndexManager::NodeIndex i,
                       RoutingIndexManager::NodeIndex j) const {
      CheckNodeIsValid(i);
      CheckNodeIsValid(j);
      if (vehicle_indices[i.value()] == -1 || vehicle_indices[j.value()] == -1 ||
          (i == Start() && free_approach) || (j == Stop() && free_return))
        return 0;
      if (j.value() >= data->SizeMissions() && j != Stop() && j != Start()) {
        IntVar* next_var = routing->NextVar(j.value());
        if (next_var->Bound() && next_var->Value() != j.value() &&
            next_var->Value() != i.value() && next_var->Value() < data->SizeMissions()) {
          return Distance(i, (RoutingIndexManager::NodeIndex)next_var->Value());
        } else {
          return Distance(i, Stop());
        }
      }
      if (i != Start() && j != Stop() && max_ride_distance_ > 0 &&
          data->distances_matrices_.at(problem_matrix_index)
                  ->Cost(RoutingIndexManager::NodeIndex(vehicle_indices[i.value()]),
                         RoutingIndexManager::NodeIndex(vehicle_indices[j.value()])) >
              max_ride_distance_)
        return CUSTOM_MAX_INT;
      return data->distances_matrices_.at(problem_matrix_index)
          ->Cost(RoutingIndexManager::NodeIndex(vehicle_indices[i.value()]),
                 RoutingIndexManager::NodeIndex(vehicle_indices[j.value()]));
    }

    int64 Time(RoutingIndexManager::NodeIndex i, RoutingIndexManager::NodeIndex j) const {
      CheckNodeIsValid(i);
      CheckNodeIsValid(j);
      if (vehicle_indices[i.value()] == -1 || vehicle_indices[j.value()] == -1)
        return 0;
      if (j.value() >= data->SizeMissions() && j != Stop() && j != Start()) {
        IntVar* next_var = routing->NextVar(j.value());
        if (next_var->Bound() && next_var->Value() != j.value() &&
            next_var->Value() != i.value() && next_var->Value() < data->SizeMissions()) {
          return Time(i, (RoutingIndexManager::NodeIndex)next_var->Value());
        } else {
          return Time(i, Stop());
        }
      }
      if (i != Start() && j != Stop() && max_ride_time_ > 0 &&
          data->times_matrices_.at(problem_matrix_index)
                  ->Cost(RoutingIndexManager::NodeIndex(vehicle_indices[i.value()]),
                         RoutingIndexManager::NodeIndex(vehicle_indices[j.value()])) >
              max_ride_time_)
        return CUSTOM_MAX_INT;
      return data->times_matrices_.at(problem_matrix_index)
          ->Cost(RoutingIndexManager::NodeIndex(vehicle_indices[i.value()]),
                 RoutingIndexManager::NodeIndex(vehicle_indices[j.value()]));
    }

    int64 FakeTime(RoutingIndexManager::NodeIndex i,
                   RoutingIndexManager::NodeIndex j) const {
      CheckNodeIsValid(i);
      CheckNodeIsValid(j);
      if (vehicle_indices[i.value()] == -1 || vehicle_indices[j.value()] == -1 ||
          (i == Start() && free_approach) || (j == Stop() && free_return))
        return 0;
      if (j.value() >= data->SizeMissions() && j != Stop() && j != Start()) {
        IntVar* next_var = routing->NextVar(j.value());
        if (next_var->Bound() && next_var->Value() != j.value() &&
            next_var->Value() != i.value() && next_var->Value() < data->SizeMissions()) {
          return Time(i, (RoutingIndexManager::NodeIndex)next_var->Value());
        } else {
          return Time(i, Stop());
        }
      }
      if (i != Start() && j != Stop() && max_ride_time_ > 0 &&
          data->times_matrices_.at(problem_matrix_index)
                  ->Cost(RoutingIndexManager::NodeIndex(vehicle_indices[i.value()]),
                         RoutingIndexManager::NodeIndex(vehicle_indices[j.value()])) >
              max_ride_time_)
        return CUSTOM_MAX_INT;
      return data->times_matrices_.at(problem_matrix_index)
          ->Cost(RoutingIndexManager::NodeIndex(vehicle_indices[i.value()]),
                 RoutingIndexManager::NodeIndex(vehicle_indices[j.value()]));
    }

    int64 Value(RoutingIndexManager::NodeIndex i,
                RoutingIndexManager::NodeIndex j) const {
      CheckNodeIsValid(i);
      CheckNodeIsValid(j);
      if (vehicle_indices[i.value()] == -1 || vehicle_indices[j.value()] == -1)
        return 0;
      if (j.value() >= data->SizeMissions() && j != Stop() && j != Start()) {
        IntVar* next_var = routing->NextVar(j.value());
        if (next_var->Bound() && next_var->Value() != j.value() &&
            next_var->Value() != i.value() && next_var->Value() < data->SizeMissions()) {
          return Value(i, (RoutingIndexManager::NodeIndex)next_var->Value());
        } else {
          return Value(i, Stop());
        }
      }
      return data->values_matrices_.at(value_matrix_index)
          ->Cost(RoutingIndexManager::NodeIndex(vehicle_indices[i.value()]),
                 RoutingIndexManager::NodeIndex(vehicle_indices[j.value()]));
    }

    int64 TimeOrder(RoutingIndexManager::NodeIndex i,
                    RoutingIndexManager::NodeIndex j) const {
      CheckNodeIsValid(i);
      CheckNodeIsValid(j);
      if (vehicle_indices[i.value()] == -1 || vehicle_indices[j.value()] == -1)
        return 0;
      return 10 *
             std::sqrt(
                 data->times_matrices_.at(problem_matrix_index)
                     ->Cost(RoutingIndexManager::NodeIndex(vehicle_indices[i.value()]),
                            RoutingIndexManager::NodeIndex(vehicle_indices[j.value()])));
    }

    int64 DistanceOrder(RoutingIndexManager::NodeIndex i,
                        RoutingIndexManager::NodeIndex j) const {
      CheckNodeIsValid(i);
      CheckNodeIsValid(j);
      if (vehicle_indices[i.value()] == -1 || vehicle_indices[j.value()] == -1)
        return 0;
      return 100 *
             std::sqrt(
                 data->distances_matrices_.at(problem_matrix_index)
                     ->Cost(RoutingIndexManager::NodeIndex(vehicle_indices[i.value()]),
                            RoutingIndexManager::NodeIndex(vehicle_indices[j.value()])));
    }

    //  Transit quantity at a node "from"
    //  This is the quantity added after visiting node "from"
    int64 DistancePlusServiceTime(RoutingIndexManager::NodeIndex from,
                                  RoutingIndexManager::NodeIndex to) const {
      return Distance(from, to) + data->ServiceTime(from);
    }

    //  Transit quantity at a node "from"
    //  This is the quantity added after visiting node "from"
    int64 TimePlusServiceTime(RoutingIndexManager::NodeIndex from,
                              RoutingIndexManager::NodeIndex to) const {
      return Time(from, to) + coef_service * data->ServiceTime(from) +
             additional_service +
             (Time(from, to) > 0 ? coef_setup * data->SetupTime(to) +
                                       (data->SetupTime(to) > 0 ? additional_setup : 0)
                                 : 0);
      // FIXME:
      // (Time(from, to) == 0 ? 0
      // and
      // data->SetupTime(from) > 0 ? additional_setup
      // logics can be incorporated in data->SetupTime(from), with a wrapper function
      // called data->SetupTime(from, to)
    }

    int64 FakeTimePlusServiceTime(RoutingIndexManager::NodeIndex from,
                                  RoutingIndexManager::NodeIndex to) const {
      return FakeTime(from, to) + coef_service * data->ServiceTime(from) +
             additional_service +
             (FakeTime(from, to) > 0
                  ? coef_setup * data->SetupTime(to) +
                        (data->SetupTime(to) > 0 ? additional_setup : 0)
                  : 0);
    }

    int64 ValuePlusServiceValue(RoutingIndexManager::NodeIndex from,
                                RoutingIndexManager::NodeIndex to) const {
      return Time(from, to) + data->ServiceValue(from);
    }

    int64 TimePlus(RoutingIndexManager::NodeIndex from,
                   RoutingIndexManager::NodeIndex to) const {
      return Time(from, to);
    }

    RoutingIndexManager::NodeIndex Start() const { return start; }

    RoutingIndexManager::NodeIndex Stop() const { return stop; }

    void CheckNodeIsValid(const RoutingIndexManager::NodeIndex i) const {
      CHECK_GE(i.value(), 0) << "Internal node " << i.value()
                             << " should be greater than or equal to 0!";
      CHECK_LT(i.value(), size)
          << "Internal node " << i.value() << " should be less than " << size;
    }

    TSPTWDataDT* data;
    RoutingModel* routing;
    RoutingIndexManager* manager;
    std::string id;
    int64 vehicle_index;
    int32 size;
    int32 size_matrix;
    int32 size_rest;
    RoutingIndexManager::NodeIndex start;
    RoutingIndexManager::NodeIndex stop;
    int64 problem_matrix_index;
    int64 value_matrix_index;
    std::vector<int64> vehicle_indices;
    std::vector<int64> capacity;
    std::vector<bool> counting;
    std::vector<int64> overload_multiplier;
    int32 break_size;
    int64 time_start;
    int64 time_end;
    int64 late_multiplier;
    int64 cost_fixed;
    int64 cost_distance_multiplier;
    int64 cost_time_multiplier;
    int64 cost_waiting_time_multiplier;
    int64 cost_value_multiplier;
    float coef_service;
    int64 additional_service;
    float coef_setup;
    int64 additional_setup;
    int64 duration;
    int64 distance;
    ShiftPref shift_preference;
    int32 day_index;
    int64 max_ride_time_;
    int64 max_ride_distance_;
    bool free_approach;
    bool free_return;
  };

  std::vector<Vehicle*> Vehicles() const { return tsptw_vehicles_; }

  struct Route {
    Route(std::string v_id) : vehicle_id(v_id), vehicle_index(-1) {}
    Route(std::string v_id, int v_int, std::vector<std::string> s_ids)
        : vehicle_id(v_id), vehicle_index(v_int), service_ids(s_ids) {}
    std::string vehicle_id;
    int vehicle_index;
    std::vector<std::string> service_ids;
  };

  std::vector<Route*> Routes() const { return tsptw_routes_; }

  struct Relation {
    Relation(int relation_no)
        : relation_number(relation_no)
        , type(Order)
        , linked_ids(NULL)
        , linked_vehicle_ids(NULL)
        , lapse(-1) {}
    Relation(int relation_no, RelationType t, std::vector<std::string>* l_i)
        : relation_number(relation_no)
        , type(t)
        , linked_ids(l_i)
        , linked_vehicle_ids(NULL)
        , lapse(-1) {}
    Relation(int relation_no, RelationType t, std::vector<std::string>* l_i,
             std::vector<std::string>* l_v_i, int32 l)
        : relation_number(relation_no)
        , type(t)
        , linked_ids(l_i)
        , linked_vehicle_ids(l_v_i)
        , lapse(l) {}

    ~Relation() {
      delete linked_ids;
      delete linked_vehicle_ids;
    }

    int relation_number;
    RelationType type;
    std::vector<std::string>* linked_ids;
    std::vector<std::string>* linked_vehicle_ids;
    int32 lapse;
  };

  std::vector<Relation*> Relations() const { return tsptw_relations_; }

  std::vector<int> VehiclesDay() const { return vehicles_day_; }

  int VehicleDay(int64 index) const {
    if (index < 0) {
      return -1;
    }
    return vehicles_day_.at(index);
  }

  int VehicleDayAlt(int64 index) const {
    if (index < 0) {
      return CUSTOM_MAX_INT;
    }
    return vehicles_day_.at(index);
  }

  /*
    Vehicle VehicleGet(int64 v) const {
      return tsptw_vehicles_.at(v);
    }
  */
private:
  void ProcessNewLine(char* const line);

  struct TSPTWClient {
    // Depot definition
    TSPTWClient(std::string cust_id, int32 m_i, int32 p_i)
        : customer_id(cust_id)
        , matrix_index(m_i)
        , problem_index(p_i)
        , alternative_index(0)
        , ready_time({-CUSTOM_MAX_INT})
        , due_time({CUSTOM_MAX_INT})
        , service_time(0.0)
        , service_value(0.0)
        , setup_time(0.0)
        , priority(4)
        , late_multiplier(0)
        , is_break(false) {}
    // Mission definition
    TSPTWClient(std::string cust_id, int32 m_i, int32 p_i, int32 a_i,
                std::vector<int64> r_t, std::vector<int64> d_t, double s_t, double s_v,
                double st_t, int32 p_t, double l_m, std::vector<int64>& v_i,
                std::vector<int64>& q, std::vector<int64>& s_q, int64 e_c,
                std::vector<bool>& r_q)
        : customer_id(cust_id)
        , matrix_index(m_i)
        , problem_index(p_i)
        , alternative_index(a_i)
        , ready_time(r_t)
        , due_time(d_t)
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
    // Rests definition
    TSPTWClient(std::string cust_id, int32 m_i, int32 p_i, int32 a_i,
                std::vector<int64> r_t, std::vector<int64> d_t, double s_t, int32 p_t,
                double l_m, std::vector<int64>& v_i, int64 e_c, bool i_b)
        : customer_id(cust_id)
        , matrix_index(m_i)
        , problem_index(p_i)
        , alternative_index(a_i)
        , ready_time(r_t)
        , due_time(d_t)
        , service_time(s_t)
        , service_value(0)
        , setup_time(0)
        , priority(p_t)
        , late_multiplier(l_m)
        , vehicle_indices(v_i)
        , quantities({})
        , setup_quantities({})
        , exclusion_cost(e_c)
        , refill_quantities({})
        , is_break(i_b) {}
    std::string customer_id;
    int32 matrix_index;
    int32 problem_index;
    int32 alternative_index;
    std::vector<int64> ready_time;
    std::vector<int64> due_time;
    int64 service_time;
    int64 service_value;
    int64 setup_time;
    int64 priority;
    int64 late_multiplier;
    std::vector<int64> vehicle_indices;
    std::vector<int64> quantities;
    std::vector<int64> setup_quantities;
    int64 exclusion_cost;
    std::vector<bool> refill_quantities;
    bool is_break;
  };

  int32 size_;
  int32 size_missions_;
  int32 size_matrix_;
  int32 size_rest_;
  int32 size_problem_;
  std::vector<int32> tws_size_;
  std::vector<Vehicle*> tsptw_vehicles_;
  std::vector<Relation*> tsptw_relations_;
  std::vector<TSPTWClient> tsptw_clients_;
  std::map<int32, int32> alternative_size_map_;
  std::vector<Route*> tsptw_routes_;
  std::vector<CompleteGraphArcCost*> distances_matrices_;
  std::vector<CompleteGraphArcCost*> times_matrices_;
  std::vector<CompleteGraphArcCost*> values_matrices_;
  std::vector<int> vehicles_day_;
  std::string details_;
  int64 horizon_;
  int64 max_time_;
  int64 max_distance_;
  int64 max_value_;
  int64 max_time_cost_;
  int64 max_distance_cost_;
  int64 max_value_cost_;
  int64 max_service_;
  int64 max_rest_;
  int64 tws_counter_;
  int64 order_counter_;
  int64 deliveries_counter_;
  int64 multiple_tws_counter_;
  std::map<std::string, int64> ids_map_;
  std::map<std::string, int64> vehicle_ids_map_;
  std::map<int64, int64> day_index_to_vehicle_index_;
};

void TSPTWDataDT::LoadInstance(const std::string& filename) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ortools_vrp::Problem problem;

  {
    std::fstream input(filename, std::ios::in | std::ios::binary);
    if (!problem.ParseFromIstream(&input)) {
      VLOG(0) << "Failed to parse pbf." << std::endl;
    }
  }

  int32 node_index      = 0;
  tws_counter_          = 0;
  multiple_tws_counter_ = 0;
  deliveries_counter_   = 0;
  int32 matrix_index    = 0;
  order_counter_        = 0;
  size_problem_         = 0;
  std::vector<int64> matrix_indices;
  for (const ortools_vrp::Service& service : problem.services()) {
    if (!alternative_size_map_.count(service.problem_index()))
      alternative_size_map_[service.problem_index()] = 0;
    const int32 tws_size = service.time_windows_size();
    tws_size_.push_back(tws_size);
    std::vector<const ortools_vrp::TimeWindow*> timewindows;
    for (int32 tw = 0; tw < tws_size; ++tw) {
      timewindows.push_back(&service.time_windows().Get(tw));
    }

    std::vector<int64> q;
    for (const int64& quantity : service.quantities()) {
      if (quantity < 0)
        ++deliveries_counter_;
      q.push_back(quantity);
    }

    std::vector<int64> s_q;
    for (const int64& setup_quantity : service.setup_quantities()) {
      s_q.push_back(setup_quantity);
    }

    std::vector<bool> r_q;
    for (const bool refill : service.refill_quantities()) {
      r_q.push_back(refill);
    }

    std::vector<int64> v_i;
    for (const int64& index : service.vehicle_indices()) {
      v_i.push_back(index);
    }

    std::vector<int64> ready_time;
    std::vector<int64> due_time;

    for (const ortools_vrp::TimeWindow* timewindow : timewindows) {
      timewindow->start() > -CUSTOM_MAX_INT ? ready_time.push_back(timewindow->start())
                                            : ready_time.push_back(-CUSTOM_MAX_INT);
      timewindow->end() < CUSTOM_MAX_INT ? due_time.push_back(timewindow->end())
                                         : due_time.push_back(CUSTOM_MAX_INT);
    }
    tws_counter_ += timewindows.size();

    if (timewindows.size() > 1)
      multiple_tws_counter_ += 1;

    int timewindow_index = 0;

    if (service.late_multiplier() > 0) {
      do {
        matrix_indices.push_back(service.matrix_index());
        std::vector<int64> start;
        if (timewindows.size() > 0 &&
            timewindows[timewindow_index]->start() > -CUSTOM_MAX_INT)
          start.push_back(timewindows[timewindow_index]->start());
        else
          start.push_back(-CUSTOM_MAX_INT);

        std::vector<int64> end;
        if (timewindows.size() > 0 &&
            timewindows[timewindow_index]->end() < CUSTOM_MAX_INT)
          end.push_back(timewindows[timewindow_index]->end());
        else
          end.push_back(CUSTOM_MAX_INT);
        size_problem_ = std::max(size_problem_, service.problem_index());
        tsptw_clients_.push_back(TSPTWClient(
            (std::string)service.id(), matrix_index, service.problem_index(),
            alternative_size_map_[service.problem_index()], start, end,
            service.duration(), service.additional_value(), service.setup_duration(),
            service.priority(),
            timewindows.size() > 0 ? (int64)(service.late_multiplier() * 1000000) : 0,
            v_i, q, s_q,
            service.exclusion_cost() > 0 ? service.exclusion_cost() * 1000000 : -1, r_q));
        alternative_size_map_[service.problem_index()] += 1;
        ids_map_[(std::string)service.id()] = node_index;
        node_index++;
        ++timewindow_index;
      } while (timewindow_index < service.time_windows_size());
    } else {
      matrix_indices.push_back(service.matrix_index());
      size_problem_ = std::max(size_problem_, service.problem_index());
      tsptw_clients_.push_back(TSPTWClient(
          (std::string)service.id(), matrix_index, service.problem_index(),
          alternative_size_map_[service.problem_index()], ready_time, due_time,
          service.duration(), service.additional_value(), service.setup_duration(),
          service.priority(),
          timewindows.size() > 0 ? (int64)(service.late_multiplier() * 1000000) : 0, v_i,
          q, s_q, service.exclusion_cost() > 0 ? service.exclusion_cost() * 1000000 : -1,
          r_q));
      alternative_size_map_[service.problem_index()] += 1;
      ids_map_[(std::string)service.id()] = node_index;
      node_index++;
    }
    ++matrix_index;
  }

  size_rest_ = 0;
  for (const ortools_vrp::Vehicle& vehicle : problem.vehicles()) {
    size_rest_ += vehicle.rests().size();
  }
  size_matrix_ = matrix_index + 2;

  for (int rest_index = 0; rest_index < size_rest_; ++rest_index) {
    matrix_indices.push_back(size_matrix_);
  }
  size_missions_ = node_index;
  size_          = node_index + size_rest_ + 2;

  max_time_          = 0;
  max_distance_      = 0;
  max_value_         = 0;
  max_time_cost_     = 0;
  max_distance_cost_ = 0;
  max_value_cost_    = 0;

  for (const ortools_vrp::Matrix& matrix : problem.matrices()) {
    // + 2 In case vehicles have no depots
    int32 problem_size =
        std::max(std::max(sqrt(matrix.distance_size()), sqrt(matrix.time_size())),
                 sqrt(matrix.value_size())) +
        2 + (size_rest_ > 0 ? 1 : 0);
    CompleteGraphArcCost* distances = new CompleteGraphArcCost();
    distances->Create(std::max(problem_size, 3));
    CompleteGraphArcCost* times = new CompleteGraphArcCost();
    times->Create(std::max(problem_size, 3));
    CompleteGraphArcCost* values = new CompleteGraphArcCost();
    values->Create(std::max(problem_size, 3));

    distances_matrices_.push_back(distances);
    times_matrices_.push_back(times);
    values_matrices_.push_back(values);

    // Matrix default values
    for (int64 i = 0; i < std::max(problem_size, 3); ++i) {
      for (int64 j = 0; j < std::max(problem_size, 3); ++j) {
        SetTimeMatrix(i, j)  = 0;
        SetMatrix(i, j)      = 0;
        SetValueMatrix(i, j) = 0;
      }
    }

    if (matrix.time_size() > 0) {
      max_time_ = std::max(max_time_, BuildTimeMatrix(matrix));
    }

    if (matrix.distance_size() > 0) {
      max_distance_ = std::max(max_distance_, BuildDistanceMatrix(matrix));
    }

    if (matrix.value_size() > 0) {
      max_value_ = std::max(max_value_, BuildValueMatrix(matrix));
    }
  }

  int64 current_day_index        = 0;
  int v_idx                      = 0;
  day_index_to_vehicle_index_[0] = v_idx;
  for (const ortools_vrp::Vehicle& vehicle : problem.vehicles()) {
    Vehicle* v = new Vehicle(this, size_);
    // Every vehicle has its own matrix definition
    std::vector<int64> vehicle_indices(matrix_indices);
    vehicle_indices.push_back(vehicle.start_index());
    vehicle_indices.push_back(vehicle.end_index());

    for (const ortools_vrp::Capacity& capacity : vehicle.capacities()) {
      v->capacity.push_back(capacity.limit());
      v->overload_multiplier.push_back(capacity.overload_multiplier());
      v->counting.push_back(capacity.counting());
    }

    v->id                   = vehicle.id();
    v->vehicle_index        = v_idx;
    v->break_size           = vehicle.rests().size();
    v->problem_matrix_index = vehicle.matrix_index();
    v->value_matrix_index   = vehicle.value_matrix_index();
    v->vehicle_indices      = vehicle_indices;
    v->time_start           = vehicle.time_window().start() > -CUSTOM_MAX_INT
                        ? vehicle.time_window().start()
                        : -CUSTOM_MAX_INT;
    v->time_end = vehicle.time_window().end() < CUSTOM_MAX_INT
                      ? vehicle.time_window().end()
                      : CUSTOM_MAX_INT;
    v->late_multiplier          = (int64)(vehicle.cost_late_multiplier() * 1000000);
    v->cost_fixed               = (int64)(vehicle.cost_fixed() * 1000000);
    v->cost_distance_multiplier = (int64)(vehicle.cost_distance_multiplier() * 1000000);
    v->cost_time_multiplier     = (int64)(vehicle.cost_time_multiplier() * 1000000);
    v->cost_waiting_time_multiplier =
        (int64)(vehicle.cost_waiting_time_multiplier() * 1000000);
    v->cost_value_multiplier = (int64)(vehicle.cost_value_multiplier() * 1000000);
    v->coef_service          = vehicle.coef_service();
    v->additional_service    = vehicle.additional_service();
    v->coef_setup            = vehicle.coef_setup();
    v->additional_setup      = vehicle.additional_setup();
    v->duration              = (int64)(vehicle.duration());
    v->distance              = vehicle.distance();
    v->free_approach         = vehicle.free_approach();
    v->free_return           = vehicle.free_return();
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

    tsptw_vehicles_.push_back(v);
    vehicle_ids_map_[(std::string)vehicle.id()] = v_idx;
    if (current_day_index < vehicle.day_index()) {
      do {
        ++current_day_index;
        day_index_to_vehicle_index_[current_day_index] = v_idx;
      } while (current_day_index < vehicle.day_index());
    }
    v_idx++;
  }

  for (const ortools_vrp::Route& route : problem.routes()) {
    Route* r = new Route(route.vehicle_id());
    for (std::size_t i = 0; i < tsptw_vehicles_.size(); ++i) {
      if (tsptw_vehicles_.at(i)->id == route.vehicle_id())
        r->vehicle_index = i;
    }
    for (std::string service_id : route.service_ids()) {
      r->service_ids.push_back(service_id);
    }
    tsptw_routes_.push_back(r);
  }

  int v_index              = 0;
  int re_index             = 0;
  int32 problem_rest_index = size_problem_ + 1;
  for (const ortools_vrp::Vehicle& vehicle : problem.vehicles()) {
    for (const ortools_vrp::Rest& rest : vehicle.rests()) {
      if (!alternative_size_map_.count(problem_rest_index))
        alternative_size_map_[problem_rest_index] = 0;
      const int32 tws_size = rest.time_windows_size();
      tws_size_.push_back(tws_size);
      std::vector<const ortools_vrp::TimeWindow*> timewindows;
      for (int32 tw = 0; tw < tws_size; ++tw) {
        timewindows.push_back(&rest.time_windows().Get(tw));
      }

      std::vector<int64> v_i;
      v_i.push_back(v_index);

      std::vector<int64> ready_time;
      std::vector<int64> due_time;

      for (const ortools_vrp::TimeWindow* timewindow : timewindows) {
        timewindow->start() > -CUSTOM_MAX_INT ? ready_time.push_back(timewindow->start())
                                              : ready_time.push_back(-CUSTOM_MAX_INT);
        timewindow->end() < CUSTOM_MAX_INT ? due_time.push_back(timewindow->end())
                                           : due_time.push_back(CUSTOM_MAX_INT);
      }
      tws_counter_ += timewindows.size();

      if (timewindows.size() > 1)
        multiple_tws_counter_ += 1;

      tsptw_clients_.push_back(TSPTWClient(
          (std::string)rest.id(), size_matrix_, problem_rest_index,
          alternative_size_map_[problem_rest_index], ready_time, due_time,
          rest.duration(), 0,
          timewindows.size() > 0 ? (int64)(rest.late_multiplier() * 1000000) : 0, v_i,
          rest.exclusion_cost() > 0 ? rest.exclusion_cost() * 1000000 : -1, true));
      ids_map_[(std::string)rest.id()] = node_index;
      alternative_size_map_[problem_rest_index] += 1;
      ++node_index;
      ++problem_rest_index;
    }
    ++v_index;
  }

  // Setting start
  for (Vehicle* v : tsptw_vehicles_) {
    v->start = RoutingIndexManager::NodeIndex(node_index);
  }
  tsptw_clients_.push_back(TSPTWClient("vehicles_start", matrix_index, node_index));

  node_index++;
  // Setting stop
  for (Vehicle* v : tsptw_vehicles_) {
    v->stop = RoutingIndexManager::NodeIndex(node_index);
  }
  // node_index++;
  tsptw_clients_.push_back(TSPTWClient("vehicles_end", ++matrix_index, node_index));

  for (const ortools_vrp::Relation& relation : problem.relations()) {
    std::vector<std::string>* linked_ids = new std::vector<std::string>();
    for (const std::string linked_id : relation.linked_ids()) {
      linked_ids->push_back(linked_id);
    }
    std::vector<std::string>* linked_v_ids = new std::vector<std::string>();
    for (const std::string linked_v_id : relation.linked_vehicle_ids()) {
      linked_v_ids->push_back(linked_v_id);
    }

    RelationType relType;
    if (relation.type() == "sequence")
      relType = Sequence;
    else if (relation.type() == "order") {
      relType = Order;
      ++order_counter_;
    } else if (relation.type() == "same_route")
      relType = SameRoute;
    else if (relation.type() == "minimum_day_lapse")
      relType = MinimumDayLapse;
    else if (relation.type() == "maximum_day_lapse")
      relType = MaximumDayLapse;
    else if (relation.type() == "shipment")
      relType = Shipment;
    else if (relation.type() == "meetup")
      relType = MeetUp;
    else if (relation.type() == "maximum_duration_lapse")
      relType = MaximumDurationLapse;
    else if (relation.type() == "force_first")
      relType = ForceFirst;
    else if (relation.type() == "never_first")
      relType = NeverFirst;
    else if (relation.type() == "force_end")
      relType = ForceLast;
    else if (relation.type() == "vehicle_group_duration")
      relType = VehicleGroupDuration;
    else if (relation.type() == "vehicle_trips")
      relType = VehicleTrips;
    else
      throw "Unknown relation type";

    tsptw_relations_.push_back(
        new Relation(re_index, relType, linked_ids, linked_v_ids, relation.lapse()));
    ++re_index;
  }

  // Compute horizon
  horizon_     = 0;
  max_service_ = 0;
  for (int32 i = 0; i < size_missions_; ++i) {
    if (tsptw_clients_[i].due_time.size() > 0)
      horizon_ = std::max(
          horizon_, tsptw_clients_[i].due_time.at(tsptw_clients_[i].due_time.size() - 1));
    max_service_ = std::max(max_service_, tsptw_clients_[i].service_time);
  }
  for (std::size_t v = 0; v < tsptw_vehicles_.size(); ++v) {
    horizon_ = std::max(horizon_, tsptw_vehicles_.at(v)->time_end);
  }
  max_rest_ = 0;
}

} //  namespace operations_research

#endif //  OR_TOOLS_TUTORIALS_CPLUSPLUS_TSP_DATA_DT_H
