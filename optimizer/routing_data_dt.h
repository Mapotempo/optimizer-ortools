#ifndef OR_TOOLS_TUTORIALS_CPLUSPLUS_ROUTING_DATA_DT_H
#define OR_TOOLS_TUTORIALS_CPLUSPLUS_ROUTING_DATA_DT_H

#include "routing_common/routing_data.h"

DECLARE_int32(width_size);

namespace operations_research {

//  Forward declaration.
class TSPLIBReader;


class RoutingDataDT : public RoutingData {
public:
  explicit RoutingDataDT(int32 size = 0): RoutingData(size) {
  }
/*
  explicit RoutingDataDT(const RoutingDataGenerator & g): RoutingData(g) {
    if (Size() > 0) {
      for (RoutingModel::NodeIndex i(0); i < Size(); ++i) {
        Coordinate(i) = g.Coordinate(i);
        for (RoutingModel::NodeIndex j(0); j < Size(); ++j) {
          InternalTime(i,j) = g.Time(i,j);
        }
      }
    }
  }
*/
  explicit RoutingDataDT(const RoutingDataDT & other): RoutingData(other) {
    for (RoutingModel::NodeIndex i(RoutingModel::kFirstNode); i < Size(); ++i) {
      for (RoutingModel::NodeIndex j(RoutingModel::kFirstNode); j < Size(); ++j) {
        InternalTime(i,j) = other.Time(i,j);
      }
    }
  }

 
  virtual ~RoutingDataDT() {}

  int64 Time(RoutingModel::NodeIndex i, RoutingModel::NodeIndex j) const {
    CheckNodeIsValid(i);
    return times_.Cost(i,j);
  }
  
  void PrintTimeMatrix(std::ostream& out, const int32 & width = FLAGS_width_size) const;
  void WriteTimeMatrix(const std::string & filename, const int32 & width = FLAGS_width_size) const;

protected:  
  virtual void CreateRoutingData(int32 size) {
    RoutingData::CreateRoutingData(size);
    times_.Create(size);
  }
  
  virtual void SetRoutingDataInstanciated() {
    RoutingData::SetRoutingDataInstanciated();
    times_.SetIsInstanciated();
  }

  int64& InternalTime(RoutingModel::NodeIndex i, RoutingModel::NodeIndex j) {
    CheckNodeIsValid(i);
    CheckNodeIsValid(j);
    return times_.Cost(i,j);
  }

protected:
  CompleteGraphArcCost times_;
};

void RoutingDataDT::PrintTimeMatrix(std::ostream& out, const int32 & width) const {
  times_.Print(out, width);
}

void RoutingDataDT::WriteTimeMatrix(const std::string& filename, const int32 & width) const {
  WriteToFileP1<RoutingDataDT, const int> writer(this, filename);
  writer.SetMember(&operations_research::RoutingDataDT::PrintTimeMatrix);
  writer.Run(width);
}


}  //  namespace operations_research

#endif //  OR_TOOLS_TUTORIALS_CPLUSPLUS_ROUTING_DATA_DT_H