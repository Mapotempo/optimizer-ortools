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

  int64 FirstTWReadyTime(RoutingModel::NodeIndex i) const {
    return tsptw_clients_[i.value()].first_ready_time;
  }

  int64 FirstTWDueTime(RoutingModel::NodeIndex i) const {
    return tsptw_clients_[i.value()].first_due_time;
  }

  int64 SecondTWReadyTime(RoutingModel::NodeIndex i) const {
    return tsptw_clients_[i.value()].second_ready_time;
  }

  int64 SecondTWDueTime(RoutingModel::NodeIndex i) const {
    return tsptw_clients_[i.value()].second_due_time;
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
    data(data_), size(size_), capacity(0), overload_multiplier(0), time_start(0), time_end(0), late_multiplier(0){
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
      return distances.Cost(i, j);
    }

    int64 Time(RoutingModel::NodeIndex i, RoutingModel::NodeIndex j) const {
      CheckNodeIsValid(i);
      CheckNodeIsValid(j);
      return times.Cost(i, j);
    }

    int64 TimeOrder(RoutingModel::NodeIndex i, RoutingModel::NodeIndex j) const {
      CheckNodeIsValid(i);
      CheckNodeIsValid(j);
      return 100*std::sqrt(times.Cost(i, j));
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
    int64 time_start;
    int64 time_end;
    int64 late_multiplier;
  };

  std::vector<Vehicle*> Vehicles() const {
    return tsptw_vehicles_;
  }
/*
  Vehicle VehicleGet(int64 v) const {
    return tsptw_vehicles_.at(v);
  }
*/
private:
  void ProcessNewLine(char* const line);

  struct TSPTWClient {
    TSPTWClient(int cust_no):
    customer_number(cust_no), first_ready_time(-2147483648), first_due_time(2147483647), second_ready_time(-2147483648), second_due_time(2147483647), service_time(0.0), late_multiplier(0){
    }
    TSPTWClient(int cust_no, double f_r_t, double f_d_t, double s_r_t, double s_d_t):
    customer_number(cust_no), first_ready_time(f_r_t), first_due_time(f_d_t), second_ready_time(s_r_t), second_due_time(s_d_t), service_time(0.0), late_multiplier(0){
    }
    TSPTWClient(int cust_no, double f_r_t, double f_d_t, double s_r_t, double s_d_t, double s_t, double l_m):
    customer_number(cust_no), first_ready_time(f_r_t), first_due_time(f_d_t), second_ready_time(s_r_t), second_due_time(s_d_t), service_time(s_t), late_multiplier(l_m){
    }
    TSPTWClient(int cust_no, double f_r_t, double f_d_t, double s_r_t, double s_d_t, double s_t, double l_m, std::vector<int64>& v_i):
    customer_number(cust_no), first_ready_time(f_r_t), first_due_time(f_d_t), second_ready_time(s_r_t), second_due_time(s_d_t), service_time(s_t), late_multiplier(l_m), vehicle_indices(v_i){
    }
    TSPTWClient(int cust_no, double f_r_t, double f_d_t, double s_r_t, double s_d_t, double s_t, double l_m, std::vector<int64>& v_i, std::vector<int64>& q):
    customer_number(cust_no), first_ready_time(f_r_t), first_due_time(f_d_t), second_ready_time(s_r_t), second_due_time(s_d_t), service_time(s_t), late_multiplier(l_m), vehicle_indices(v_i), quantities(q){
    }
    int customer_number;
    int64 first_ready_time;
    int64 first_due_time;
    int64 second_ready_time;
    int64 second_due_time;
    int64 service_time;
    int64 late_multiplier;
    std::vector<int64> vehicle_indices;
    std::vector<int64> quantities;
  };

    int32 size_;
    int32 size_matrix_;
    int32 size_rest_;
  std::vector<Vehicle*> tsptw_vehicles_;
  std::vector<TSPTWClient> tsptw_clients_;
  std::string details_;
  int64 horizon_;
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
  for (const ortools_vrp::Service& service: problem.services()) {
    const ortools_vrp::TimeWindow* tw0 = service.time_windows_size() >= 1 ? &service.time_windows().Get(0) : NULL;
    const ortools_vrp::TimeWindow* tw1 = service.time_windows_size() >= 2 ? &service.time_windows().Get(1) : NULL;

    std::vector<int64> q;
    for (const int64& quantity: service.quantities()) {
      q.push_back(quantity);
    }
    std::vector<int64> v_i;
    for (const int64& index: service.vehicle_indices()) {
      v_i.push_back(index);
    }

    tsptw_clients_.push_back(TSPTWClient(s++,
                                         tw0 && tw0->start() > -2147483648/100 ? tw0->start()*100 : -2147483648,
                                         tw0 && tw0->end() < 2147483647/100 ? tw0->end()*100 : 2147483647,
                                         tw1 && tw1->start() > -2147483648/100 ? tw1->start()*100 : -2147483648,
                                         tw1 && tw1->end() < 2147483647/100 ? tw1->end()*100 : 2147483647,
                                         service.duration()*100,
                                         service.late_multiplier(),
                                         v_i,
                                         q));
  }

  size_rest_ = 0;
  for (const ortools_vrp::Vehicle& vehicle: problem.vehicles()) {
    size_matrix_ = sqrt(vehicle.time_matrix().data_size());
    size_rest_ += vehicle.rests().size();
  }
  size_ = size_matrix_ + size_rest_;

  for (const ortools_vrp::Vehicle& vehicle: problem.vehicles()) {
    Vehicle* v = new Vehicle(this, size_);

    for (int i = 0; i < size_matrix_; ++i) {
      for (int j = 0; j < size_matrix_; ++j) {
        v->SetTimeMatrix(i, j) = static_cast<int64>(vehicle.time_matrix().data(i + j * size_matrix_) * 100 + 0.5);
      }
    }

    for (int i = 0; i < size_matrix_; ++i) {
      for (int j = 0; j < size_matrix_; ++j) {
        v->SetMatrix(i, j) = static_cast<int64>(vehicle.distance_matrix().data(i + j * size_matrix_));
      }
    }

    for (const ortools_vrp::Capacity& capacity: vehicle.capacities()) {
      v->capacity.push_back(capacity.limit());
      v->overload_multiplier.push_back(capacity.overload_multiplier());
    }

    v->time_start = vehicle.time_window().start() > -2147483648/100 ? vehicle.time_window().start() * 100 : -2147483648;
    v->time_end = vehicle.time_window().end() < 2147483647/100 ? vehicle.time_window().end() * 100 : 2147483647;
    v->late_multiplier = vehicle.time_window().late_multiplier();

    tsptw_vehicles_.push_back(v);
  }

  // Setting start
  for (Vehicle* v: tsptw_vehicles_) {
    v->start = RoutingModel::NodeIndex(s);
  }
  tsptw_clients_.push_back(TSPTWClient(s++));

  // Setting stop
  for (Vehicle* v: tsptw_vehicles_) {
    v->stop = RoutingModel::NodeIndex(s);
  }
  tsptw_clients_.push_back(TSPTWClient(s++));

  int v = 0;
  for (const ortools_vrp::Vehicle& vehicle: problem.vehicles()) {
    for (const ortools_vrp::Rest& rest: vehicle.rests()) {
      std::vector<int64> v_i;
      v_i.push_back(v);
      const ortools_vrp::TimeWindow* tw0 = rest.time_windows_size() >= 1 ? &rest.time_windows().Get(0) : NULL;
      const ortools_vrp::TimeWindow* tw1 = rest.time_windows_size() >= 2 ? &rest.time_windows().Get(1) : NULL;

      tsptw_clients_.push_back(TSPTWClient(s++,
                                           tw0 && tw0->start() > -2147483648/100 ? tw0->start()*100 : -2147483648,
                                           tw0 && tw0->end() < 2147483647/100 ? tw0->end()*100 : 2147483647,
                                           tw1 && tw1->start() > -2147483648/100 ? tw1->start()*100: -2147483648,
                                           tw1 && tw1->end() < 2147483647/100 ? tw1->end()*100 : 2147483647,
                                           rest.duration()*100,
                                           rest.time_windows().Get(0).late_multiplier(),
                                           v_i));
    }
    ++v;
  }

  // Compute horizon
  horizon_ = 0;
  for (int32 i = 0; i < size_; ++i) {
    horizon_ = std::max(horizon_, tsptw_clients_[i].first_due_time);
  }
}

}  //  namespace operations_research

#endif //  OR_TOOLS_TUTORIALS_CPLUSPLUS_TSP_DATA_DT_H