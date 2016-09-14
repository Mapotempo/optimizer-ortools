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
#include "routing_data_dt.h"

namespace operations_research {

class TSPTWDataDT : public RoutingDataDT {
public:
  explicit TSPTWDataDT(std::string filename) : RoutingDataDT(0), instantiated_(false) {
    LoadInstance(filename);
    SetRoutingDataInstanciated();
  }
  void LoadInstance(const std::string & filename);

  void SetStart(RoutingModel::NodeIndex s) {
    CHECK_LT(s, Size());
    start_ = s;
  }

  void SetStop(RoutingModel::NodeIndex s) {
    CHECK_LT(s, Size());
    stop_ = s;
  }

  RoutingModel::NodeIndex Start() const {
    return start_;
  }

  RoutingModel::NodeIndex Stop() const {
    return stop_;
  }

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

  int64 TimeOrder(RoutingModel::NodeIndex i, RoutingModel::NodeIndex j) const {
    CheckNodeIsValid(i);
    CheckNodeIsValid(j);
    return 100*std::sqrt(times_.Cost(i, j)) ;
  }

  // Override
  int64 Time(RoutingModel::NodeIndex i, RoutingModel::NodeIndex j) const {
      CheckNodeIsValid(i);
      CheckNodeIsValid(j);
      return times_.Cost(i, j);
  }

  // Override
  int64 Distance(RoutingModel::NodeIndex i, RoutingModel::NodeIndex j) const {
      CheckNodeIsValid(i);
      CheckNodeIsValid(j);
      return distances_.Cost(i, j);
  }

  // Override
  int64& InternalDistance(RoutingModel::NodeIndex i, RoutingModel::NodeIndex j) {
      CheckNodeIsValid(i);
      CheckNodeIsValid(j);
      return distances_.Cost(i,j);
  }

  //  Transit quantity at a node "from"
  //  This is the quantity added after visiting node "from"
  int64 DistancePlusServiceTime(RoutingModel::NodeIndex from,
                  RoutingModel::NodeIndex to) const {
    return Distance(from, to) + ServiceTime(from);
  }

  //  Transit quantity at a node "from"
  //  This is the quantity added after visiting node "from"
  int64 TimePlusServiceTime(RoutingModel::NodeIndex from,
                  RoutingModel::NodeIndex to) const {
    return Time(from, to) + ServiceTime(from);
  }

  int64 TimePlus(RoutingModel::NodeIndex from,
                  RoutingModel::NodeIndex to) const {
    return Time(from, to);
  }

  int64 Quantity(_ConstMemberResultCallback_0_1<false, int64, RoutingModel, IntType<_RoutingModel_NodeIndex_tag_, int> >::base* nodeToIndex, int64 i, RoutingModel::NodeIndex from, RoutingModel::NodeIndex to) const {
    CheckNodeIsValid(from);
    CheckNodeIsValid(to);
    int64 index = nodeToIndex->Run(from);
    if (i < tsptw_clients_.at(index).quantities.size()) {
      return tsptw_clients_.at(index).quantities.at(i);
    } else {
      return 0;
    }
  }

  void PrintLIBInstance(std::ostream& out) const;
  void PrintDSUInstance(std::ostream& out) const;
  void WriteLIBInstance(const std::string & filename) const;
  void WriteDSUInstance(const std::string & filename) const;

  int32 SizeMatrix() const {
    return size_matrix_;
  }

  int32 SizeRest() const {
    return size_rest_;
  }

  std::vector<int64> Capacity() const {
    return capacity_;
  }

  std::vector<int64> CostOverloadMultiplier() const {
    return cost_overload_multiplier_;
  }

  int64 VehicleTimeStart() const {
    return vehicle_time_start_;
  }

  int64 VehicleTimeEnd() const {
    return vehicle_time_end_;
  }

  int64 VehicleLateMultiplier() const {
    return vehicle_late_multiplier_;
  }

private:
  int32 size_matrix_;
  int32 size_rest_;
  void ProcessNewLine(char* const line);
  void InitLoadInstance() {
    line_number_ = 0;
    visualizable_ = false;
    two_dimension_ = false;
    symmetric_ = false;
    name_ = "";
    comment_ = "";
  }
  std::vector<int64> capacity_;
  std::vector<int64> cost_overload_multiplier_;
  int64 vehicle_time_start_;
  int64 vehicle_time_end_;
  int64 vehicle_late_multiplier_;

  //  Helper function
  int64& SetMatrix(int i, int j) {
    return distances_.Cost(RoutingModel::NodeIndex(i), RoutingModel::NodeIndex(j));
  }

  int64& SetTimeMatrix(int i, int j) {
    return times_.Cost(RoutingModel::NodeIndex(i), RoutingModel::NodeIndex(j));
  }


  bool instantiated_;
  RoutingModel::NodeIndex start_, stop_;
  struct TSPTWClient {
    TSPTWClient(int cust_no, double f_r_t, double f_d_t, double s_r_t, double s_d_t):
    customer_number(cust_no), first_ready_time(f_r_t), first_due_time(f_d_t), second_ready_time(s_r_t), second_due_time(s_d_t), service_time(0.0), late_multiplier(0){
    }
    TSPTWClient(int cust_no, double f_r_t, double f_d_t, double s_r_t, double s_d_t, double s_t, double l_m):
    customer_number(cust_no), first_ready_time(f_r_t), first_due_time(f_d_t), second_ready_time(s_r_t), second_due_time(s_d_t), service_time(s_t), late_multiplier(l_m){
    }
    TSPTWClient(int cust_no, double f_r_t, double f_d_t, double s_r_t, double s_d_t, double s_t, double l_m, std::vector<int64>& q):
    customer_number(cust_no), first_ready_time(f_r_t), first_due_time(f_d_t), second_ready_time(s_r_t), second_due_time(s_d_t), service_time(s_t), late_multiplier(l_m), quantities(q){
    }
    int customer_number;
    int64 first_ready_time;
    int64 first_due_time;
    int64 second_ready_time;
    int64 second_due_time;
    int64 service_time;
    int64 late_multiplier;
    std::vector<int64> quantities;
  };

  std::vector<TSPTWClient> tsptw_clients_;
  std::string details_;
  std::string filename_;
  int64 horizon_;
  bool visualizable_;
  bool two_dimension_;
  bool symmetric_;

  int line_number_;
  std::string comment_;
};

// Parses a file in López-Ibáñez-Blum or
// da Silva-Urrutia formats and loads the coordinates.
// Note that the format is only partially checked:
// bad inputs might cause undefined behavior.
void TSPTWDataDT::LoadInstance(const std::string & filename) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ortools_vrp::Problem problem;

  {
    std::fstream input(filename, std::ios::in | std::ios::binary);
    if (!problem.ParseFromIstream(&input)) {
      VLOG(0) << "Failed to parse pbf." << std::endl;
    }
  }

  InitLoadInstance();

  // Problem size
  size_matrix_ = sqrt(problem.time_matrix().data_size());
  size_rest_ = problem.vehicles(0).rests().size();
  size_ = size_matrix_ + size_rest_;

  CreateRoutingData(size_);
  // Matrix default values
  for (int64 i=0; i < size_; ++i) {
    for (int64 j=0; j < size_; ++j) {
      SetMatrix(i, j) = 0;
      SetTimeMatrix(i, j) = 0;
    }
  }

  for (int i = 0; i < size_matrix_; ++i) {
    for (int j = 0; j < size_matrix_; ++j) {
      SetTimeMatrix(i, j) = static_cast<int64>(problem.time_matrix().data(i + j * size_matrix_) * 100 + 0.5);
    }
  }

  for (int i = 0; i < size_matrix_; ++i) {
    for (int j = 0; j < size_matrix_; ++j) {
      SetMatrix(i, j) = static_cast<int64>(problem.distance_matrix().data(i + j * size_matrix_));
    }
  }

  int s = 0;
  tsptw_clients_.push_back(TSPTWClient(s++,
                                       -2147483648,
                                       2147483647,
                                       -2147483648,
                                       2147483647));

  for (const ortools_vrp::Service& service: problem.services()) {
    const ortools_vrp::TimeWindow* tw0 = service.time_windows_size() >= 1 ? &service.time_windows().Get(0) : NULL;
    const ortools_vrp::TimeWindow* tw1 = service.time_windows_size() >= 2 ? &service.time_windows().Get(1) : NULL;

    std::vector<int64> q(service.quantities_size());
    for (const int64& quantity: service.quantities()) {
      q.push_back(quantity);
    }

    tsptw_clients_.push_back(TSPTWClient(s++,
                                         tw0 ? tw0->start()*100 : -2147483648,
                                         tw0 ? tw0->end()*100 : 2147483647,
                                         tw1 ? tw1->start()*100 : -2147483648,
                                         tw1 ? tw1->end()*100 : 2147483647,
                                         service.duration()*100,
                                         service.late_multiplier(),
                                         q));
  }

  tsptw_clients_.push_back(TSPTWClient(s++,
                                       -2147483648,
                                       2147483647,
                                       -2147483648,
                                       2147483647));

  for (const ortools_vrp::Vehicle& vehicle: problem.vehicles()) {
    std::vector<int64> q(vehicle.capacities_size());
    std::vector<int64> cost_overload_multiplier(vehicle.capacities_size());
    for (const ortools_vrp::Capacity& capacity: vehicle.capacities()) {
      q.push_back(capacity.limit());
      q.push_back(capacity.cost_overload_multiplier());
    }

    capacity_ = q;
    cost_overload_multiplier_ = cost_overload_multiplier;

    vehicle_time_start_ = vehicle.time_window().start() * 100;
    vehicle_time_end_ = vehicle.time_window().end() * 100;
    vehicle_late_multiplier_ = vehicle.time_window().late_multiplier();

    for (const ortools_vrp::Rest& rest: vehicle.rests()) {
      const ortools_vrp::TimeWindow* tw0 = rest.time_windows_size() >= 1 ? &rest.time_windows().Get(0) : NULL;
      const ortools_vrp::TimeWindow* tw1 = rest.time_windows_size() >= 2 ? &rest.time_windows().Get(1) : NULL;

      tsptw_clients_.push_back(TSPTWClient(s++,
                                           tw0 ? tw0->start()*100 : -2147483648,
                                           tw0 ? tw0->end()*100 : 2147483647,
                                           tw1 ? tw1->start()*100: -2147483648,
                                           tw1 ? tw1->end()*100 : 2147483647,
                                           rest.duration()*100,
                                           rest.time_windows().Get(0).late_multiplier()));
    }
  }

  // Compute horizon
  horizon_ = 0;
  for (int32 i = 0; i < size_; ++i) {
    horizon_ = std::max(horizon_, tsptw_clients_[i].first_due_time);
  }

  // Setting start: always first matrix node
  start_ = RoutingModel::NodeIndex(tsptw_clients_[0].customer_number);

  // Setting stop: always last matrix node
  stop_ = RoutingModel::NodeIndex(tsptw_clients_[SizeMatrix() - 1].customer_number);

  filename_ = filename;
  instantiated_ = true;
}

}  //  namespace operations_research

#endif //  OR_TOOLS_TUTORIALS_CPLUSPLUS_TSP_DATA_DT_H