#ifndef OR_TOOLS_TUTORIALS_CPLUSPLUS_TSPTW_DATA_DT_H
#define OR_TOOLS_TUTORIALS_CPLUSPLUS_TSPTW_DATA_DT_H

#include <ostream>
#include <iomanip>
#include <vector>

#include "constraint_solver/routing.h"
#include "base/filelinereader.h"
#include "base/split.h"
#include "base/strtoint.h"


#include "ortools_vrp.pb.h"
#include "routing_common/routing_common.h"

#define CUSTOM_MAX_INT (int64)std::pow(2,30)

enum RelationType { MaximumDurationLapse = 7, MeetUp = 6, Shipment = 5, MaximumDayLapse = 4, MinimumDayLapse = 3, SameRoute = 2, Order = 1, Sequence = 0 };

namespace operations_research {

class TSPTWDataDT {
public:
  explicit TSPTWDataDT(std::string filename) {
    LoadInstance(filename);
  }
  void LoadInstance(const std::string & filename);

  //  Helper function
  int64& SetMatrix(int i, int j) {
    return distances_matrices_.back()->Cost(RoutingModel::NodeIndex(i), RoutingModel::NodeIndex(j));
  }

  int64& SetTimeMatrix(int i, int j) {
    return times_matrices_.back()->Cost(RoutingModel::NodeIndex(i), RoutingModel::NodeIndex(j));
  }

  int64& SetValueMatrix(int i, int j) {
    return values_matrices_.back()->Cost(RoutingModel::NodeIndex(i), RoutingModel::NodeIndex(j));
  }

  int64 BuildTimeMatrix(ortools_vrp::Matrix matrix) {
    int64 max_time_ = 0;
    int32 size_matrix = sqrt(matrix.time_size());
    for (int64 i = 0; i < size_matrix; ++i) {
      for (int64 j = 0; j < size_matrix; ++j) {
        if (static_cast<int64>(matrix.time(i * size_matrix + j)) < CUSTOM_MAX_INT)
          max_time_ = std::max(max_time_, static_cast<int64>(matrix.time(i * size_matrix + j) + 0.5));
        SetTimeMatrix(i, j) = static_cast<int64>(matrix.time(i * size_matrix + j) + 0.5);
      }
    }
    return max_time_;
  }

  int64 BuildDistanceMatrix(ortools_vrp::Matrix matrix) {
    int64 max_distance_ = 0;
    int32 size_matrix = sqrt(matrix.distance_size());
    for (int64 i = 0; i < size_matrix; ++i) {
      for (int64 j = 0; j < size_matrix; ++j) {
          if (static_cast<int64>(matrix.distance(i * size_matrix + j)) < CUSTOM_MAX_INT)
            max_distance_ = std::max(max_distance_, static_cast<int64>(matrix.distance(i * size_matrix + j)));
          SetMatrix(i, j) = static_cast<int64>(matrix.distance(i * size_matrix + j));
      }
    }
    return max_distance_;
  }

  int64 BuildValueMatrix(ortools_vrp::Matrix matrix) {
    int64 max_value_ = 0;
    int32 size_matrix = sqrt(matrix.value_size());
    for (int64 i = 0; i < size_matrix; ++i) {
      for (int64 j = 0; j < size_matrix; ++j) {
          if (static_cast<int64>(matrix.value(i * size_matrix + j)) < CUSTOM_MAX_INT)
            max_value_ = std::max(max_value_, static_cast<int64>(matrix.value(i * size_matrix + j)));
          SetValueMatrix(i, j) = static_cast<int64>(matrix.value(i * size_matrix + j));
      }
    }
    return max_value_;
  }

  int64 Horizon() const {
    return horizon_;
  }

  int64 MatrixIndex(RoutingModel::NodeIndex i) const {
    return tsptw_clients_[i.value()].matrix_index;
  }

  int64 MaxTime() const {
    return max_time_;
  }

  int64 MaxDistance() const {
    return max_distance_;
  }

  int64 MaxValue() const {
    return max_value_;
  }

  int64 MaxServiceTime() const {
    return max_service_;
  }

  int64 MaxTimeCost() const {
    return max_time_cost_;
  }

  int64 MaxDistanceCost() const {
    return max_distance_cost_;
  }

  int64 MaxValueCost() const {
    return max_value_cost_;
  }

  int64 TWsCounter() const {
    return tws_counter_;
  }

  int64 TwiceTWsCounter() const {
    return multiple_tws_counter_;
  }

  int64 DeliveriesCounter() const {
    return deliveries_counter_;
  }

  int64 IdIndex(std::string id) const {
    return ids_map_.find(id)->second;
  }

  std::string ServiceId(RoutingModel::NodeIndex i) const {
    return tsptw_clients_[i.value()].customer_id;
  }

  std::vector<int64> ReadyTime(RoutingModel::NodeIndex i) const {
    return tsptw_clients_[i.value()].ready_time;
  }

  std::vector<int64> DueTime(RoutingModel::NodeIndex i) const {
    return tsptw_clients_[i.value()].due_time;
  }

  int64 LateMultiplier(RoutingModel::NodeIndex i)  const {
    return tsptw_clients_[i.value()].late_multiplier;
  }

  int64 ServiceTime(RoutingModel::NodeIndex i)  const {
    return tsptw_clients_[i.value()].service_time;
  }

  int64 ServiceValue(RoutingModel::NodeIndex i)  const {
    return tsptw_clients_[i.value()].service_value;
  }

  int64 SetupTime(RoutingModel::NodeIndex i)  const {
    return tsptw_clients_[i.value()].setup_time;
  }

  int64 Priority(RoutingModel::NodeIndex i) const {
    return (int64)tsptw_clients_[i.value()].priority;
  }

  std::vector<int64> VehicleIndices(RoutingModel::NodeIndex i) const {
    return tsptw_clients_[i.value()].vehicle_indices;
  }

  int32 TimeWindowsSize(int i) const {
    return tws_size_.at(i);
  }

  bool isPickUp(RoutingModel::NodeIndex i) const {
    return tsptw_clients_[i.value()].type != "delivery";
  }

  int32 Size() const {
    return size_;
  }

  int32 SizeMatrix() const {
    return size_matrix_;
  }

  int32 SizeRest() const {
    return size_rest_;
  }

  int64 Quantity(_ConstMemberResultCallback_0_1<false, int64, RoutingModel, IntType<_RoutingModel_NodeIndex_tag_, int> >::base* nodeToIndex, int64 i, RoutingModel::NodeIndex from, RoutingModel::NodeIndex to) const {
//    CheckNodeIsValid(from);
//    CheckNodeIsValid(to);
    int64 index = nodeToIndex->Run(from);
    if (i < tsptw_clients_.at(index).quantities.size()) {
      if (tsptw_vehicles_[0]->counting.at(i)) {
        if (tsptw_clients_[index].type != "delivery") {
          return tsptw_clients_.at(index).quantities.at(i) + (tsptw_vehicles_[0]->stop == to || tsptw_vehicles_[0]->Distance(from, to) > 0 || tsptw_vehicles_[0]->Time(from, to) > 0 ? tsptw_clients_.at(index).setup_quantities.at(i) : 0);
        } else {
          return tsptw_clients_.at(index).quantities.at(i) + (tsptw_vehicles_[0]->stop == to || tsptw_vehicles_[0]->Distance(from, to) > 0 || tsptw_vehicles_[0]->Time(from, to) > 0 ? -tsptw_clients_.at(index).setup_quantities.at(i) : 0);
        }
      }
      else if (tsptw_clients_[index].type != "delivery") {
        return tsptw_clients_.at(index).quantities.at(i);
      }
      else {
        return -tsptw_clients_.at(index).quantities.at(i);
      }
    } else {
      return 0;
    }
  }

  bool IsDelivery(RoutingModel::NodeIndex i) const {
    return tsptw_clients_[i.value()].type == "delivery";
  }

  std::vector<int64> Quantities(RoutingModel::NodeIndex i) const {
    return tsptw_clients_[i.value()].quantities;
  }

  struct Vehicle {
    Vehicle(TSPTWDataDT* data_, int32 size_):
    data(data_), size(size_), capacity(0), overload_multiplier(0), break_size(0), time_start(0), time_end(0), late_multiplier(0), problem_matrix_index(0), value_matrix_index(0), vehicle_indices(0){
    }

    int32 SizeMatrix() const {
      return size_matrix;
    }

    int32 SizeRest() const {
      return size_rest;
    }

    void SetStart(RoutingModel::NodeIndex s) {
      CHECK_LT(s, size);
      start = s;
    }

    void SetStop(RoutingModel::NodeIndex s) {
      CHECK_LT(s, size);
      stop = s;
    }

    int64 Distance(RoutingModel::NodeIndex i, RoutingModel::NodeIndex j) const {
      CheckNodeIsValid(i);
      CheckNodeIsValid(j);
      if (vehicle_indices[i.value()] == -1 || vehicle_indices[j.value()] == -1) return 0;
      return data->distances_matrices_.at(problem_matrix_index)->Cost(RoutingModel::NodeIndex(vehicle_indices[i.value()]),
        RoutingModel::NodeIndex(vehicle_indices[j.value()]));
    }

    int64 Time(RoutingModel::NodeIndex i, RoutingModel::NodeIndex j) const {
      CheckNodeIsValid(i);
      CheckNodeIsValid(j);
      if (vehicle_indices[i.value()] == -1 || vehicle_indices[j.value()] == -1) return 0;
      return data->times_matrices_.at(problem_matrix_index)->Cost(RoutingModel::NodeIndex(vehicle_indices[i.value()]),
        RoutingModel::NodeIndex(vehicle_indices[j.value()]));
    }

    int64 Value(RoutingModel::NodeIndex i, RoutingModel::NodeIndex j) const {
      CheckNodeIsValid(i);
      CheckNodeIsValid(j);
      if (vehicle_indices[i.value()] == -1 || vehicle_indices[j.value()] == -1) return 0;
      return data->values_matrices_.at(value_matrix_index)->Cost(RoutingModel::NodeIndex(vehicle_indices[i.value()]),
        RoutingModel::NodeIndex(vehicle_indices[j.value()]));
    }

    int64 TimeOrder(RoutingModel::NodeIndex i, RoutingModel::NodeIndex j) const {
      CheckNodeIsValid(i);
      CheckNodeIsValid(j);
      if (vehicle_indices[i.value()] == -1 || vehicle_indices[j.value()] == -1) return 0;
      return 10 * std::sqrt(data->times_matrices_.at(problem_matrix_index)->Cost(RoutingModel::NodeIndex(vehicle_indices[i.value()]),
        RoutingModel::NodeIndex(vehicle_indices[j.value()])));
    }

    int64 DistanceOrder(RoutingModel::NodeIndex i, RoutingModel::NodeIndex j) const {
      CheckNodeIsValid(i);
      CheckNodeIsValid(j);
      if (vehicle_indices[i.value()] == -1 || vehicle_indices[j.value()] == -1) return 0;
      return 100 * std::sqrt(data->distances_matrices_.at(problem_matrix_index)->Cost(RoutingModel::NodeIndex(vehicle_indices[i.value()]),
        RoutingModel::NodeIndex(vehicle_indices[j.value()])));
    }

    //  Transit quantity at a node "from"
    //  This is the quantity added after visiting node "from"
    int64 DistancePlusServiceTime(RoutingModel::NodeIndex from, RoutingModel::NodeIndex to) const {
      return Distance(from, to) + data->ServiceTime(from);
    }

    //  Transit quantity at a node "from"
    //  This is the quantity added after visiting node "from"
    int64 TimePlusServiceTime(RoutingModel::NodeIndex from, RoutingModel::NodeIndex to) const {
      return Time(from, to) + data->ServiceTime(from) + (Time(from, to) > 0 ? data->SetupTime(from) : 0);
    }

    int64 ValuePlusServiceValue(RoutingModel::NodeIndex from, RoutingModel::NodeIndex to) const {
      return Time(from, to) + data->ServiceValue(from);
    }

    int64 TimePlus(RoutingModel::NodeIndex from, RoutingModel::NodeIndex to) const {
      return Time(from, to);
    }

    RoutingModel::NodeIndex Start() const {
      return start;
    }

    RoutingModel::NodeIndex Stop() const {
      return stop;
    }

    void CheckNodeIsValid(const RoutingModel::NodeIndex i) const {
      CHECK_GE(i.value(), 0) << "Internal node " << i.value() << " should be greater than 0!";
      CHECK_LT(i.value(), size) << "Internal node " << i.value() << " should be less than " << size;
    }

    TSPTWDataDT* data;
    int32 size;
    int32 size_matrix;
    int32 size_rest;
    RoutingModel::NodeIndex start;
    RoutingModel::NodeIndex stop;
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
    int64 duration;
    bool force_start;
    int32 day_index;
  };

  std::vector<Vehicle*> Vehicles() const {
    return tsptw_vehicles_;
  }

  struct Rest {
    Rest(int rest_no):
        rest_number(rest_no), rest_start(0), rest_end(0), rest_duration(0), vehicle(0){}
    Rest(int rest_no, double r_s, double r_e, double r_d, int v_i):
        rest_number(rest_no), rest_start(r_s), rest_end(r_e), rest_duration(r_d), vehicle(v_i){}
        int rest_number;
        int64 rest_start;
        int64 rest_end;
        int64 rest_duration;
        int vehicle;
  };

    std::vector<Rest*> Rests() const {
    return tsptw_rests_;
  }

  struct Relation {
    Relation(int relation_no):
        relation_number(relation_no), type(Order), linked_ids(NULL), lapse(-1){}
    Relation(int relation_no, RelationType t, std::vector<std::string>* l_i):
        relation_number(relation_no), type(t), linked_ids(l_i), lapse(-1){}
    Relation(int relation_no, RelationType t, std::vector<std::string>* l_i, int32 l):
        relation_number(relation_no), type(t), linked_ids(l_i), lapse(l){}
        int relation_number;
        RelationType type;
        std::vector<std::string>* linked_ids;
        int32 lapse;
  };

  std::vector<Relation*> Relations() const {
    return tsptw_relations_;
  }

  std::vector<int> VehiclesDay() const {
    return vehicles_day_;
  }
/*
  Vehicle VehicleGet(int64 v) const {
    return tsptw_vehicles_.at(v);
  }
*/
private:
  void ProcessNewLine(char* const line);

  struct TSPTWClient {
    TSPTWClient(std::string cust_id, int32 m_i):
    customer_id(cust_id), matrix_index(m_i), ready_time({-CUSTOM_MAX_INT}), due_time({CUSTOM_MAX_INT}), service_time(0.0), service_value(0.0), setup_time(0.0), priority(4), late_multiplier(0){
    }
    TSPTWClient(std::string cust_id, int32 m_i, std::vector<int64> r_t, std::vector<int64> d_t):
    customer_id(cust_id), matrix_index(m_i), ready_time(r_t), due_time(d_t), service_time(0.0), service_value(0.0), setup_time(0.0), priority(4), late_multiplier(0), type(""){
    }
    TSPTWClient(std::string cust_id, int32 m_i, std::vector<int64> r_t, std::vector<int64> d_t, double s_t, double l_m):
    customer_id(cust_id), matrix_index(m_i), ready_time(r_t), due_time(d_t), service_time(s_t), service_value(0.0), setup_time(0.0), priority(4), late_multiplier(l_m), type(""){
    }
    TSPTWClient(std::string cust_id, int32 m_i, std::vector<int64> r_t, std::vector<int64> d_t, double s_t, double l_m, std::vector<int64>& v_i):
    customer_id(cust_id), matrix_index(m_i), ready_time(r_t), due_time(d_t), service_time(s_t), service_value(0.0), setup_time(0.0), priority(4), late_multiplier(l_m), vehicle_indices(v_i), type(""){
    }
    TSPTWClient(std::string cust_id, int32 m_i, std::vector<int64> r_t, std::vector<int64> d_t, double s_t, double l_m, std::vector<int64>& v_i, std::vector<int64>& q):
    customer_id(cust_id), matrix_index(m_i), ready_time(r_t), due_time(d_t), service_time(s_t), service_value(0.0), setup_time(0.0), priority(4), late_multiplier(l_m), vehicle_indices(v_i), quantities(q), type(""){
    }
    TSPTWClient(std::string cust_id, int32 m_i, std::vector<int64> r_t, std::vector<int64> d_t, double s_t, int32 p_t, double l_m, std::vector<int64>& v_i, std::vector<int64>& q):
    customer_id(cust_id), matrix_index(m_i), ready_time(r_t), due_time(d_t), service_time(s_t), service_value(0.0), setup_time(0.0), priority(p_t), late_multiplier(l_m), vehicle_indices(v_i), quantities(q), type(""){
    }
    TSPTWClient(std::string cust_id, int32 m_i, std::vector<int64> r_t, std::vector<int64> d_t, double s_t, double st_t, int32 p_t, double l_m, std::vector<int64>& v_i, std::vector<int64>& q, std::string t):
    customer_id(cust_id), matrix_index(m_i), ready_time(r_t), due_time(d_t), service_time(s_t), service_value(0.0), setup_time(st_t), priority(p_t), late_multiplier(l_m), vehicle_indices(v_i), quantities(q), type(t){
    }
    TSPTWClient(std::string cust_id, int32 m_i, std::vector<int64> r_t, std::vector<int64> d_t, double s_t, double s_v, double st_t, int32 p_t, double l_m, std::vector<int64>& v_i, std::vector<int64>& q, std::vector<int64>& s_q, std::string t):
    customer_id(cust_id), matrix_index(m_i), ready_time(r_t), due_time(d_t), service_time(s_t), service_value(s_v), setup_time(st_t), priority(p_t), late_multiplier(l_m), vehicle_indices(v_i), quantities(q), setup_quantities(s_q), type(t){
    }
    std::string customer_id;
    int32 matrix_index;
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
    std::string type;
  };

  int32 size_;
  int32 size_matrix_;
  int32 size_rest_;
  std::vector<int32> tws_size_;
  std::vector<Vehicle*> tsptw_vehicles_;
  std::vector<Rest*> tsptw_rests_;
  std::vector<Relation*> tsptw_relations_;
  std::vector<TSPTWClient> tsptw_clients_;
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
  int64 deliveries_counter_;
  int64 multiple_tws_counter_;
  std::map<std::string, int64> ids_map_;
};

void TSPTWDataDT::LoadInstance(const std::string & filename) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ortools_vrp::Problem problem;

  {
    std::fstream input(filename, std::ios::in | std::ios::binary);
    if (!problem.ParseFromIstream(&input)) {
      VLOG(0) << "Failed to parse pbf." << std::endl;
    }
  }

  int s = 0;
  tws_counter_ = 0;
  multiple_tws_counter_ = 0;
  deliveries_counter_ = 0;
  int32 problem_index = 0;
  std::vector<int64> matrix_indices;
  for (const ortools_vrp::Service& service: problem.services()) {
    const int32 tws_size = service.time_windows_size();
    tws_size_.push_back(tws_size);
    std::vector<const ortools_vrp::TimeWindow*> timewindows;
    for (int32 tw = 0; tw < tws_size; ++tw) {
      timewindows.push_back(&service.time_windows().Get(tw));
    }

    std::vector<int64> q;
    for (const int64& quantity: service.quantities()) {
      q.push_back(quantity);
    }

    std::vector<int64> s_q;
    for (const int64& setup_quantity: service.setup_quantities()) {
      s_q.push_back(setup_quantity);
    }

    std::vector<int64> v_i;
    for (const int64& index: service.vehicle_indices()) {
      v_i.push_back(index);
    }

    std::vector<int64> ready_time;
    std::vector<int64> due_time;

    for (const ortools_vrp::TimeWindow* timewindow : timewindows) {
      timewindow->start() > -CUSTOM_MAX_INT ? ready_time.push_back(timewindow->start()) : ready_time.push_back(-CUSTOM_MAX_INT);
      timewindow->end() < CUSTOM_MAX_INT ? due_time.push_back(timewindow->end()) : due_time.push_back(CUSTOM_MAX_INT);
    }
    tws_counter_ += timewindows.size();
    std::string t = service.type();

    if (t == "delivery") ++deliveries_counter_;
    if (timewindows.size() > 1) multiple_tws_counter_ += 1;

    int timewindow_index = 0;

    if (service.late_multiplier() > 0) {
      do {
        matrix_indices.push_back(service.matrix_index());
        std::vector<int64> start;
        if (timewindows.size() > 0 && timewindows[timewindow_index]->start() > -CUSTOM_MAX_INT)
          start.push_back(timewindows[timewindow_index]->start());
        else
          start.push_back(-CUSTOM_MAX_INT);

        std::vector<int64>  end;
        if (timewindows.size() > 0 && timewindows[timewindow_index]->end() < CUSTOM_MAX_INT)
          end.push_back(timewindows[timewindow_index]->end());
        else
          end.push_back(CUSTOM_MAX_INT);
        tsptw_clients_.push_back(TSPTWClient((std::string)service.id(),
                                           problem_index,
                                           start,
                                           end,
                                           service.duration(),
                                           service.additional_value(),
                                           service.setup_duration(),
                                           service.priority(),
                                           timewindows.size() > 0 ? (int64)(service.late_multiplier() * 1000) : 0,
                                           v_i,
                                           q,
                                           s_q,
                                           t));
        ids_map_[(std::string)service.id()] = s;
        s++;
        ++timewindow_index;
      } while (timewindow_index < service.time_windows_size());
    } else {
      matrix_indices.push_back(service.matrix_index());
      tsptw_clients_.push_back(TSPTWClient((std::string)service.id(),
                                         problem_index,
                                         ready_time,
                                         due_time,
                                         service.duration(),
                                         service.additional_value(),
                                         service.setup_duration(),
                                         service.priority(),
                                         timewindows.size() > 0 ? (int64)(service.late_multiplier() * 1000) : 0,
                                         v_i,
                                         q,
                                         s_q,
                                         t));
      ids_map_[(std::string)service.id()] = s;
      s++;
    }
    ++problem_index;
  }

  size_rest_ = 0;
  size_matrix_ = problem_index + 2;
  for (const ortools_vrp::Vehicle& vehicle: problem.vehicles()) {
    size_rest_ += vehicle.rests().size();
  }
  size_ = s + 2;

  max_time_ = 0;
  max_distance_ = 0;
  max_value_ = 0;
  max_time_cost_ = 0;
  max_distance_cost_ = 0;
  max_value_cost_ = 0;

  for (const ortools_vrp::Matrix& matrix: problem.matrices()) {

    int32 problem_size = std::max(std::max(sqrt(matrix.distance_size()), sqrt(matrix.time_size())), sqrt(matrix.value_size()));
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
    for (int64 i=0; i < std::max(problem_size, 3); ++i) {
      for (int64 j=0; j < std::max(problem_size, 3); ++j) {
        SetTimeMatrix(i, j) = 0;
        SetMatrix(i, j) = 0;
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

  for (const ortools_vrp::Vehicle& vehicle: problem.vehicles()) {
    Vehicle* v = new Vehicle(this, size_);
    std::vector<int64> vehicle_indices(matrix_indices);
    vehicle_indices.push_back(vehicle.start_index());
    vehicle_indices.push_back(vehicle.end_index());

    for (const ortools_vrp::Capacity& capacity: vehicle.capacities()) {
      v->capacity.push_back(capacity.limit());
      v->overload_multiplier.push_back(capacity.overload_multiplier());
      v->counting.push_back(capacity.counting());
    }

    v->break_size = vehicle.rests().size();
    v->problem_matrix_index = vehicle.matrix_index();
    v->value_matrix_index = vehicle.value_matrix_index();
    v->vehicle_indices = vehicle_indices;
    v->time_start = vehicle.time_window().start() > -CUSTOM_MAX_INT ? vehicle.time_window().start() : -CUSTOM_MAX_INT;
    v->time_end = vehicle.time_window().end() < CUSTOM_MAX_INT ? vehicle.time_window().end() : CUSTOM_MAX_INT;
    v->late_multiplier = (int64)(vehicle.cost_late_multiplier() * 1000);
    v->cost_fixed = (int64)(vehicle.cost_fixed() * 1000);
    v->cost_distance_multiplier = (int64)(vehicle.cost_distance_multiplier() * 1000);
    v->cost_time_multiplier = (int64)(vehicle.cost_time_multiplier() * 1000);
    v->cost_waiting_time_multiplier = (int64)(vehicle.cost_waiting_time_multiplier() * 1000);
    v->cost_value_multiplier = (int64)(vehicle.cost_value_multiplier() * 1000);
    v->duration = (int64)(vehicle.duration());
    v->force_start = vehicle.force_start();
    v->day_index = vehicle.day_index();
    vehicles_day_.push_back(vehicle.day_index());

    max_distance_cost_ = std::max(max_distance_cost_, v->cost_distance_multiplier);
    max_time_cost_ = std::max(max_time_cost_, v->cost_time_multiplier);
    max_value_cost_ = std::max(max_value_cost_, v->cost_value_multiplier);

    tsptw_vehicles_.push_back(v);
  }

  // Setting start
  for (Vehicle* v: tsptw_vehicles_) {
    v->start = RoutingModel::NodeIndex(s);
  }
  s++;
  tsptw_clients_.push_back(TSPTWClient("vehicles_start", problem_index));

  // Setting stop
  for (Vehicle* v: tsptw_vehicles_) {
    v->stop = RoutingModel::NodeIndex(s);
  }
  s++;
  tsptw_clients_.push_back(TSPTWClient("vehicles_end",  ++problem_index));
  int v_index = 0;
  int r_index = 0;
  int re_index = 0;
  for (const ortools_vrp::Vehicle& vehicle: problem.vehicles()) {
    for (const ortools_vrp::Rest& rest: vehicle.rests()) {
      Rest* r = new Rest(r_index);

      const ortools_vrp::TimeWindow* tw0 = rest.time_windows_size() >= 1 ? &rest.time_windows().Get(0) : NULL;

      r->rest_start = tw0 && tw0->start() > -CUSTOM_MAX_INT ? tw0->start() : -CUSTOM_MAX_INT;
      r->rest_end = tw0->end() < CUSTOM_MAX_INT ? tw0->end() : CUSTOM_MAX_INT;

      r->rest_duration = rest.duration();
      r->vehicle = v_index;

      tsptw_rests_.push_back(r);
      ++r_index;
    }
    ++v_index;
  }

  for (const ortools_vrp::Relation& relation: problem.relations()) {
    std::vector<std::string>* linked_ids = new std::vector<std::string>();
    for (const std::string linked_id: relation.linked_ids()) {
      linked_ids->push_back(linked_id);
    }

    RelationType type;
    if (relation.type() == "sequence") type = Sequence;
    else if (relation.type() == "order") type = Order;
    else if (relation.type() == "same_route") type = SameRoute;
    else if (relation.type() == "minimum_day_lapse") type = MinimumDayLapse;
    else if (relation.type() == "maximum_day_lapse") type = MaximumDayLapse;
    else if (relation.type() == "shipment") type = Shipment;
    else if (relation.type() == "meetup") type = MeetUp;
    else if (relation.type() == "maximum_duration_lapse") type = MaximumDurationLapse;

    tsptw_relations_.push_back(new Relation(re_index,
                                        type,
                                        linked_ids,
                                        relation.lapse()));
    ++re_index;
  }

  // Compute horizon
  horizon_ = 0;
  max_service_ = 0;
  for (int32 i = 0; i < size_ - 2; ++i) {
    if (tsptw_clients_[i].due_time.size() > 0)
      horizon_ = std::max(horizon_, tsptw_clients_[i].due_time.at(tsptw_clients_[i].due_time.size() - 1));
    max_service_ = std::max(max_service_, tsptw_clients_[i].service_time);
  }
  for(int v = 0; v < tsptw_vehicles_.size(); ++v) {
    horizon_ = std::max(horizon_, tsptw_vehicles_.at(v)->time_end);
  }
  max_rest_ = 0;
  for (int32 i = 0; i < size_rest_; ++i) {
    max_rest_ = std::max(max_rest_, tsptw_rests_[i]->rest_duration);
  }
}

}  //  namespace operations_research

#endif //  OR_TOOLS_TUTORIALS_CPLUSPLUS_TSP_DATA_DT_H
