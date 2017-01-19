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

#define CUSTOM_MAX_INT std::pow(2,62)

namespace operations_research {

class TSPTWDataDT {
public:
  explicit TSPTWDataDT(std::string filename) {
    LoadInstance(filename);
  }
  void LoadInstance(const std::string & filename);

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

  int64 MaxServiceTime() const {
    return max_service_;
  }

  int64 MaxTimeCost() const {
    return max_time_cost_;
  }

  int64 MaxDistanceCost() const {
    return max_distance_cost_;
  }

  int64 TWsCounter() const {
    return tws_counter_;
  }

  int64 ReadyTime(RoutingModel::NodeIndex i) const {
    return tsptw_clients_[i.value()].ready_time;
  }

  int64 DueTime(RoutingModel::NodeIndex i) const {
    return tsptw_clients_[i.value()].due_time;
  }

  int64 LateMultiplier(RoutingModel::NodeIndex i)  const {
    return tsptw_clients_[i.value()].late_multiplier;
  }

  int64 ServiceTime(RoutingModel::NodeIndex i)  const {
    return tsptw_clients_[i.value()].service_time;
  }

  std::vector<int64> VehicleIndices(RoutingModel::NodeIndex i) const {
    return tsptw_clients_[i.value()].vehicle_indices;
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
      return tsptw_clients_.at(index).quantities.at(i);
    } else {
      return 0;
    }
  }

  struct Vehicle {
    Vehicle(TSPTWDataDT* data_, int32 size_):
    data(data_), size(size_), capacity(0), overload_multiplier(0), break_size(0), time_start(0), time_end(0), late_multiplier(0){
      distances.Create(size);
      times.Create(size);

      // Matrix default values
      for (int64 i=0; i < size; ++i) {
        for (int64 j=0; j < size; ++j) {
          SetMatrix(i, j) = 0;
          SetTimeMatrix(i, j) = 0;
        }
      }
    }

    int32 SizeMatrix() const {
      return size_matrix;
    }

    int32 SizeRest() const {
      return size_rest;
    }

    //  Helper function
    int64& SetMatrix(int i, int j) {
      return distances.Cost(RoutingModel::NodeIndex(i), RoutingModel::NodeIndex(j));
    }

    int64& SetTimeMatrix(int i, int j) {
      return times.Cost(RoutingModel::NodeIndex(i), RoutingModel::NodeIndex(j));
    }

    int64 BuildTimeMatrix(std::vector<int64> matrix_indices, ortools_vrp::Matrix matrix) {
      int64 max_time_ = 0;
      int32 size_matrix_ = sqrt(matrix.time_size());
      for (int64 i = 0; i < matrix_indices.size(); ++i) {
        for (int64 j = 0; j < matrix_indices.size(); ++j) {
          if (matrix_indices.at(i) != -1 && matrix_indices.at(j) != -1) {
            max_time_ = std::max(max_time_, static_cast<int64>(matrix.time(matrix_indices.at(i) * size_matrix_ + matrix_indices.at(j)) * 100 + 0.5));
            SetTimeMatrix(i, j) = static_cast<int64>(matrix.time(matrix_indices.at(i) * size_matrix_ + matrix_indices.at(j)) * 100 + 0.5);
          }
        }
      }
      return max_time_;
    }

    int64 BuildDistanceMatrix(std::vector<int64> matrix_indices, ortools_vrp::Matrix matrix) {
      int64 max_distance_ = 0;
      int32 size_matrix_ = sqrt(matrix.distance_size());
      for (int64 i = 0; i < matrix_indices.size(); ++i) {
        for (int64 j = 0; j < matrix_indices.size(); ++j) {
          if (matrix_indices.at(i) != -1 && matrix_indices.at(j) != -1) {
            max_distance_ = std::max(max_distance_, static_cast<int64>(matrix.distance(matrix_indices.at(i) * size_matrix_ + matrix_indices.at(j))));
            SetMatrix(i, j) = static_cast<int64>(matrix.distance(matrix_indices.at(i) * size_matrix_ + matrix_indices.at(j)));
          }
        }
      }
      return max_distance_;
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
      return distances.Cost(RoutingModel::NodeIndex(data->tsptw_clients_[i.value()].matrix_index),
        RoutingModel::NodeIndex(data->tsptw_clients_[j.value()].matrix_index));
    }

    int64 Time(RoutingModel::NodeIndex i, RoutingModel::NodeIndex j) const {
      CheckNodeIsValid(i);
      CheckNodeIsValid(j);
      return times.Cost(RoutingModel::NodeIndex(data->tsptw_clients_[i.value()].matrix_index),
        RoutingModel::NodeIndex(data->tsptw_clients_[j.value()].matrix_index));
    }

    int64 TimeOrder(RoutingModel::NodeIndex i, RoutingModel::NodeIndex j) const {
      CheckNodeIsValid(i);
      CheckNodeIsValid(j);
      return 100 * std::sqrt(times.Cost(RoutingModel::NodeIndex(data->tsptw_clients_[i.value()].matrix_index),
        RoutingModel::NodeIndex(data->tsptw_clients_[j.value()].matrix_index)));
    }

    int64 DistanceOrder(RoutingModel::NodeIndex i, RoutingModel::NodeIndex j) const {
      CheckNodeIsValid(i);
      CheckNodeIsValid(j);
      return 100 * std::sqrt(distances.Cost(RoutingModel::NodeIndex(data->tsptw_clients_[i.value()].matrix_index),
        RoutingModel::NodeIndex(data->tsptw_clients_[j.value()].matrix_index)));
    }

    //  Transit quantity at a node "from"
    //  This is the quantity added after visiting node "from"
    int64 DistancePlusServiceTime(RoutingModel::NodeIndex from, RoutingModel::NodeIndex to) const {
      return Distance(from, to) + data->ServiceTime(from);
    }

    //  Transit quantity at a node "from"
    //  This is the quantity added after visiting node "from"
    int64 TimePlusServiceTime(RoutingModel::NodeIndex from, RoutingModel::NodeIndex to) const {
      return Time(from, to) + data->ServiceTime(from);
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
    CompleteGraphArcCost distances;
    CompleteGraphArcCost times;
    RoutingModel::NodeIndex start;
    RoutingModel::NodeIndex stop;
    std::vector<int64> capacity;
    std::vector<int64> overload_multiplier;
    int32 break_size;
    int64 time_start;
    int64 time_end;
    int64 late_multiplier;
    int64 cost_fixed;
    int64 cost_distance_multiplier;
    int64 cost_time_multiplier;
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
/*
  Vehicle VehicleGet(int64 v) const {
    return tsptw_vehicles_.at(v);
  }
*/
private:
  void ProcessNewLine(char* const line);

  struct TSPTWClient {
    TSPTWClient(int cust_no, int32 m_i):
    customer_number(cust_no), matrix_index(m_i), ready_time(-CUSTOM_MAX_INT), due_time(CUSTOM_MAX_INT), service_time(0.0), late_multiplier(0){
    }
    TSPTWClient(int cust_no, int32 m_i, double r_t, double d_t):
    customer_number(cust_no), matrix_index(m_i), ready_time(r_t), due_time(d_t), service_time(0.0), late_multiplier(0){
    }
    TSPTWClient(int cust_no, int32 m_i, double r_t, double d_t, double s_t, double l_m):
    customer_number(cust_no), matrix_index(m_i), ready_time(r_t), due_time(d_t), service_time(s_t), late_multiplier(l_m){
    }
    TSPTWClient(int cust_no, int32 m_i, double r_t, double d_t, double s_t, double l_m, std::vector<int64>& v_i):
    customer_number(cust_no), matrix_index(m_i), ready_time(r_t), due_time(d_t), service_time(s_t), late_multiplier(l_m), vehicle_indices(v_i){
    }
    TSPTWClient(int cust_no, int32 m_i, double r_t, double d_t, double s_t, double l_m, std::vector<int64>& v_i, std::vector<int64>& q):
    customer_number(cust_no), matrix_index(m_i), ready_time(r_t), due_time(d_t), service_time(s_t), late_multiplier(l_m), vehicle_indices(v_i), quantities(q){
    }
    int customer_number;
    int32 matrix_index;
    int64 ready_time;
    int64 due_time;
    int64 service_time;
    int64 late_multiplier;
    std::vector<int64> vehicle_indices;
    std::vector<int64> quantities;
  };

  int32 size_;
  int32 size_matrix_;
  int32 size_rest_;
  std::vector<Vehicle*> tsptw_vehicles_;
  std::vector<Rest*> tsptw_rests_;
  std::vector<TSPTWClient> tsptw_clients_;
  std::string details_;
  int64 horizon_;
  int64 max_time_;
  int64 max_distance_;
  int64 max_time_cost_;
  int64 max_distance_cost_;
  int64 max_service_;
  int64 max_rest_;
  int64 tws_counter_;
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
  int32 problem_index = 0;
  std::vector<int64> matrix_indices;

  for (const ortools_vrp::Service& service: problem.services()) {
    const ortools_vrp::TimeWindow* tw0 = service.time_windows_size() >= 1 ? &service.time_windows().Get(0) : NULL;
    const ortools_vrp::TimeWindow* tw1 = service.time_windows_size() >= 2 ? &service.time_windows().Get(1) : NULL;

    matrix_indices.push_back(service.matrix_index());
    std::vector<int64> q;
    for (const int64& quantity: service.quantities()) {
      q.push_back(quantity);
    }
    std::vector<int64> v_i;
    for (const int64& index: service.vehicle_indices()) {
      v_i.push_back(index);
    }
    tsptw_clients_.push_back(TSPTWClient(s++,
                                         problem_index,
                                         tw0 && tw0->start() > -CUSTOM_MAX_INT/100 ? tw0->start()*100 : -CUSTOM_MAX_INT,
                                         tw0 && tw0->end() < CUSTOM_MAX_INT/100 ? tw0->end()*100 : CUSTOM_MAX_INT,
                                         service.duration()*100,
                                         tw0 ? tw0->late_multiplier() * 1000 : 0,
                                         v_i,
                                         q));
    if (tw1) {
      tsptw_clients_.push_back(TSPTWClient(s++,
                                         problem_index,
                                         tw1 && tw1->start() > -CUSTOM_MAX_INT/100 ? tw1->start()*100 : -CUSTOM_MAX_INT,
                                         tw1 && tw1->end() < CUSTOM_MAX_INT/100 ? tw1->end()*100 : CUSTOM_MAX_INT,
                                         service.duration()*100,
                                         tw0 ? tw0->late_multiplier() * 1000 : 0,
                                         v_i,
                                         q));
    }

    ++problem_index;
    if (tw0 && tw0->start() > -CUSTOM_MAX_INT/100 || tw0 && tw0->end() < CUSTOM_MAX_INT/100) {
      ++tws_counter_;
      if (tw1 && tw1->start() > -CUSTOM_MAX_INT/100 || tw1 && tw1->end() < CUSTOM_MAX_INT/100)
        ++tws_counter_;
    }
  }

  size_rest_ = 0;
  size_matrix_ = problem_index + 2;
  for (const ortools_vrp::Vehicle& vehicle: problem.vehicles()) {
    size_rest_ += vehicle.rests().size();
  }
  size_ = s + 2;

  max_time_ = 0;
  max_distance_ = 0;
  max_time_cost_ = 0;
  max_distance_cost_ = 0;

  for (const ortools_vrp::Vehicle& vehicle: problem.vehicles()) {
    Vehicle* v = new Vehicle(this, size_);
    std::vector<int64> vehicle_indices(matrix_indices);
    vehicle_indices.push_back(vehicle.start_index());
    vehicle_indices.push_back(vehicle.end_index());

    if (problem.matrices(vehicle.matrix_index()).time_size() > 0) {
      max_time_ = std::max(max_time_, v->BuildTimeMatrix(vehicle_indices, problem.matrices(vehicle.matrix_index())));
    }

    if (problem.matrices(vehicle.matrix_index()).distance_size() > 0) {
      max_distance_ = std::max(max_distance_, v->BuildDistanceMatrix(vehicle_indices, problem.matrices(vehicle.matrix_index())));
    }

    for (const ortools_vrp::Capacity& capacity: vehicle.capacities()) {
      v->capacity.push_back(capacity.limit());
      v->overload_multiplier.push_back(capacity.overload_multiplier());
    }

    v->break_size = vehicle.rests().size();
    v->time_start = vehicle.time_window().start() > -CUSTOM_MAX_INT/100 ? vehicle.time_window().start() * 100 : -CUSTOM_MAX_INT;
    v->time_end = vehicle.time_window().end() < CUSTOM_MAX_INT/100 ? vehicle.time_window().end() * 100 : CUSTOM_MAX_INT;
    v->late_multiplier = vehicle.time_window().late_multiplier() * 1000;
    v->cost_fixed = vehicle.cost_fixed() * 1000;
    v->cost_distance_multiplier = vehicle.cost_distance_multiplier() * 1000;
    v->cost_time_multiplier = vehicle.cost_time_multiplier() * 1000;

    max_distance_cost_ = std::max(max_distance_cost_, v->cost_distance_multiplier);
    max_time_cost_ = std::max(max_time_cost_, v->cost_time_multiplier);

    tsptw_vehicles_.push_back(v);
  }

  // Setting start
  for (Vehicle* v: tsptw_vehicles_) {
    v->start = RoutingModel::NodeIndex(s);
  }
  tsptw_clients_.push_back(TSPTWClient(s++, problem_index));

  // Setting stop
  for (Vehicle* v: tsptw_vehicles_) {
    v->stop = RoutingModel::NodeIndex(s);
  }
  tsptw_clients_.push_back(TSPTWClient(s++, ++problem_index));

  int v_index = 0;
  int r_index = 0;
  for (const ortools_vrp::Vehicle& vehicle: problem.vehicles()) {
    for (const ortools_vrp::Rest& rest: vehicle.rests()) {
      Rest* r = new Rest(r_index);

      const ortools_vrp::TimeWindow* tw0 = rest.time_windows_size() >= 1 ? &rest.time_windows().Get(0) : NULL;

      r->rest_start = tw0 && tw0->start() > -CUSTOM_MAX_INT/100 ? tw0->start()*100 : -CUSTOM_MAX_INT;
      r->rest_end = tw0->end() < CUSTOM_MAX_INT/100 ? tw0->end()*100 : CUSTOM_MAX_INT;

      r->rest_duration = rest.duration()*100;
      r->vehicle = v_index;

      tsptw_rests_.push_back(r);
      ++r_index;
    }
    ++v_index;
  }

  // Compute horizon
  horizon_ = 0;
  max_service_ = 0;
  for (int32 i = 0; i < size_; ++i) {
    horizon_ = std::max(horizon_, tsptw_clients_[i].due_time);
    max_service_ = std::max(max_service_, tsptw_clients_[i].service_time);
  }
  int v = 0;
  for (const ortools_vrp::Vehicle& vehicle: problem.vehicles()) {
    horizon_ = std::max(horizon_, (int64)vehicle.time_window().end());
    ++v;
  }
  max_rest_ = 0;
  for (int32 i = 0; i < size_rest_; ++i) {
    max_rest_ = std::max(max_rest_, (int64)tsptw_rests_[i]->rest_duration);
  }
}

}  //  namespace operations_research

#endif //  OR_TOOLS_TUTORIALS_CPLUSPLUS_TSP_DATA_DT_H
