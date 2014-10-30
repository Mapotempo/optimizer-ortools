//
//  Common base for TSPTW solutions.

#ifndef OR_TOOLS_TUTORIALS_CPLUSPLUS_TSPTW_SOLUTION_DT_H
#define OR_TOOLS_TUTORIALS_CPLUSPLUS_TSPTW_SOLUTION_DT_H

#include <vector>

#include "constraint_solver/routing.h"
#include "base/split.h"
#include "base/filelinereader.h"
#include "base/join.h"
#include "base/bitmap.h"

#include "routing_common/routing_solution.h"
#include "tsptw_data_dt.h"

namespace operations_research {

  class TSPTWSolution : public RoutingSolution {
  public:
    typedef std::vector<RoutingModel::NodeIndex>::iterator iterator;
    typedef std::vector<RoutingModel::NodeIndex>::const_iterator const_iterator;
    explicit TSPTWSolution(const TSPTWDataDT & data) :  RoutingSolution(data), data_(data), depot_(RoutingModel::kFirstNode) {}
    TSPTWSolution(const TSPTWDataDT & data, std::string filename): RoutingSolution(data), data_(data), depot_(RoutingModel::kFirstNode),
    loaded_solution_obj_(-1) {
        LoadInstance(filename);
    }
    TSPTWSolution(const TSPTWDataDT & data, const RoutingModel * routing, const Assignment * sol): RoutingSolution(data), data_(data),
      depot_(RoutingModel::kFirstNode), loaded_solution_obj_(-1) {
      CHECK_NOTNULL(routing);
      CHECK_NOTNULL(sol);
      depot_ = routing->IndexToNode(routing->GetDepot());
      for (int64 node = routing->Start(0); !routing->IsEnd(node);
           node = sol->Value(routing->NextVar(node))) {
        RoutingModel::NodeIndex node_id = routing->IndexToNode(node);
        Add(node_id);
      }
    }

    virtual ~TSPTWSolution() {}

    RoutingModel::NodeIndex Depot() const {
      return depot_;
    }

    void SetDepot(RoutingModel::NodeIndex d) {
      depot_ = d;
    }

    std::string Name() const {
      return name_;
    }

    void SetName(const std::string & name) {
      name_ = name;
    }

    //  We only consider complete solutions.
    virtual void LoadInstance(std::string filename);
    virtual bool IsSolution() const;
    virtual bool IsFeasibleSolution() const;
    virtual int64 ComputeObjectiveValue() const;
    virtual bool Add(RoutingModel::NodeIndex i, int route_number = 0) {
      sol_.push_back(i);
      return true;
    }

    //iterators
    iterator begin() { return sol_.begin(); }
    const_iterator begin() const { return sol_.begin(); }
    iterator end() { return sol_.end(); }
    const_iterator end() const { return sol_.end(); }

    virtual void Print(std::ostream & out) const;
    virtual void Write(const std::string & filename) const ;
  protected:
    std::vector<RoutingModel::NodeIndex> sol_;
  private:
    const TSPTWDataDT & data_;
    RoutingModel::NodeIndex depot_;
    int line_number_;
    void ProcessNewLine(char* const line);
    void InitLoadInstance() {
      line_number_ = 0;
      sol_.clear();
      name_ = "";
      comment_ = "";
    }

    std::string name_;
    std::string comment_;
    int64 loaded_solution_obj_;
  };

  void TSPTWSolution::LoadInstance(std::string filename) {
    InitLoadInstance();
    FileLineReader reader(filename.c_str());
    reader.set_line_callback(NewPermanentCallback(
                             this,
                             &TSPTWSolution::ProcessNewLine));
    reader.Reload();
    if (!reader.loaded_successfully()) {
      LOG(FATAL) << "Could not open TSPTW solution file: " << filename;
    }
  }

  bool TSPTWSolution::IsSolution() const {
    // Test if same number of nodes
    if (data_.Size() != Size() || depot_ != sol_[0]) {
      return false;
    }

    // Test if all nodes are used
    Bitmap used(Size());

    for (int i = 0; i < Size(); ++i) {
      int32 index = sol_[i].value();
      if (used.Get(index)) {
        return false;
      } else {
        used.Set(index,true);
      }
    }

    return true;
  }

  bool TSPTWSolution::IsFeasibleSolution() const {
    if (!IsSolution()) {
      return false;
    }

    int64 total_time = 0;
    int64 waiting_time = 0;
    const_iterator iter = begin();
    
    RoutingModel::NodeIndex first_node = *iter;
    ++iter;
    RoutingModel::NodeIndex from_node = first_node;
    RoutingModel::NodeIndex to_node = first_node;
    const int width = 3;
    VLOG(1) << std::setw(11 + width) << std::left << "Actions:"
            << std::setw(6 + width) << std::right << "Nodes:"
            << std::setw(9 + width) << std::right << "Releases:"
            << std::setw(10 + width) << std::right << "Deadlines:"
            << std::setw(9 + width) << std::right << "Services:"
            << std::setw(10 + width) << std::right << "Durations:"
            << std::setw(5 + width) << std::right << "Time:" << std::endl;
    for (; iter != end(); ++iter) {
      to_node = *iter;
      total_time += data_.Distance(from_node, to_node);
      VLOG(1) << std::setw(11 + width) << std::left << "travel to"
              << std::setw(6 + width) << std::right << to_node.value()
              << std::setw(9 + width) << std::right << data_.ReadyTime(to_node)
              << std::setw(10 + width) << std::right << data_.DueTime(to_node)
              << std::setw(9 + width) << std::right << data_.ServiceTime(to_node)
              << std::setw(10 + width) << data_.Distance(from_node, to_node)
              << std::setw(5 + width) << total_time;
      //  test if we can service client immediatly or not
      waiting_time = data_.ReadyTime(to_node) - total_time;
      if (waiting_time > 0) {
        total_time = data_.ReadyTime(to_node);
        VLOG(1) << std::setw(11 + width) << std::left << "wait in"
                << std::setw(6 + width) << std::right << to_node.value()
                << std::setw(9 + width) << std::right << data_.ReadyTime(to_node)
                << std::setw(10 + width) << std::right << data_.DueTime(to_node)
                << std::setw(9 + width) << std::right << data_.ServiceTime(to_node)
                << std::setw(10 + width) << waiting_time
                << std::setw(5 + width) << total_time;
      }
      total_time += data_.ServiceTime(to_node);
      VLOG(1) << std::setw(11 + width) << std::left << "serve"
              << std::setw(6 + width) << std::right << to_node.value()
              << std::setw(9 + width) << std::right << data_.ReadyTime(to_node)
              << std::setw(10 + width) << std::right << data_.DueTime(to_node)
              << std::setw(9 + width) << std::right << data_.ServiceTime(to_node)
              << std::setw(10 + width) << data_.ServiceTime(to_node)
              << std::setw(5 + width) << total_time;
      if (total_time  > data_.DueTime(to_node)) {
        return false;
      }
      from_node = to_node;
    }

    //  Last arc
    total_time += data_.Distance(to_node, first_node);
    VLOG(1) << std::setw(11 + width) << std::left << "travel to"
            << std::setw(6 + width) << std::right << first_node.value()
            << std::setw(9 + width) << std::right << data_.ReadyTime(first_node)
            << std::setw(10 + width) << std::right << data_.DueTime(first_node)
            << std::setw(9 + width) << std::right << data_.ServiceTime(first_node)
            << std::setw(10 + width) << data_.Distance(to_node, first_node)
            << std::setw(5 + width) << total_time;
    //  test if we can service client immediatly or not
    waiting_time = data_.ReadyTime(first_node) - total_time;
    if (waiting_time > 0) {
      total_time = data_.ReadyTime(first_node);
      VLOG(1) << std::setw(11 + width) << std::left << "wait in"
              << std::setw(6 + width) << std::right << first_node.value()
              << std::setw(9 + width) << std::right << data_.ReadyTime(first_node)
              << std::setw(10 + width) << std::right << data_.DueTime(first_node)
              << std::setw(9 + width) << std::right << data_.ServiceTime(first_node)
              << std::setw(10 + width) << waiting_time
              << std::setw(5 + width) << total_time;
    }
    total_time += data_.ServiceTime(first_node);
    VLOG(1) << std::setw(11 + width) << std::left << "serve"
            << std::setw(6 + width) << std::right << first_node.value()
            << std::setw(9 + width) << std::right << data_.ReadyTime(first_node)
            << std::setw(10 + width) << std::right << data_.DueTime(first_node)
            << std::setw(9 + width) << std::right << data_.ServiceTime(first_node)
            << std::setw(10 + width) << data_.ServiceTime(first_node)
            << std::setw(5 + width) << total_time;
    if (total_time > data_.DueTime(first_node)) {
      return false;
    }

    return true;
  }

 
  int64 TSPTWSolution::ComputeObjectiveValue() const {
    int64 obj = 0;
    RoutingModel::NodeIndex first_node, from_node, to_node;

    first_node = sol_[0];
    from_node = first_node;

    for (int i = 1; i < Size(); ++i) {
      to_node = sol_[i];
      obj += data_.Distance(from_node, to_node);
      from_node = to_node;
    }

    //  Last arc
    obj += data_.Distance(to_node, first_node);

    return obj;
  }

  void TSPTWSolution::Print(std::ostream& out) const {
    for (const_iterator iter = begin(); iter != end(); ++iter) {
      out << (*iter).value() + 1 << " ";
    }
    out << std::endl;
    out << ComputeObjectiveValue();
  }

  void TSPTWSolution::Write(const std::string & filename) const {
    WriteToFile<TSPTWSolution> writer(this, filename);
    writer.SetMember(&operations_research::TSPTWSolution::Print);
    writer.Run();
  }

void TSPTWSolution::ProcessNewLine(char*const line) {
  ++line_number_;
  CHECK_LE(line_number_, 2) << "Solution file for TSPTW is of wrong format and contains more than 2 lines.";
  static const char kWordDelimiters[] = " ";
  std::vector<std::string> words = strings::Split(line, kWordDelimiters, strings::SkipEmpty());

  if (line_number_ == 1) {
    size_ = words.size();
    CHECK_GE(size_, 3);
    sol_.clear();
    sol_.reserve(size_);
    for (int node = 0; node < size_; ++node) {
      int32 node_id = atoi32(words[node]);
      CHECK_LE(node_id, size_) << "Node " << node_id << " is greater than size " << size_ << " of solution.";
      sol_.push_back(RoutingModel::NodeIndex(node_id - 1));
    }
    return;
  }

  if (line_number_ == 2) {
    CHECK_EQ(words.size(), 1) << "Only objective value allowed on second line of TSPTW solution file.";
    loaded_solution_obj_ = atoi64(words[0]);
    return;
  }
}  //  void ProcessNewLine(char* const line)


}  //  namespace operations_research

#endif //  OR_TOOLS_TUTORIALS_CPLUSPLUS_TSPTW_SOLUTION_H