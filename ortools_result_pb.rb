# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ortools_result.proto

require 'google/protobuf'

Google::Protobuf::DescriptorPool.generated_pool.build do
  add_file("ortools_result.proto", :syntax => :proto3) do
    add_message "ortools_result.Activity" do
      optional :index, :int32, 1
      repeated :quantities, :float, 2
      optional :start_time, :int32, 3
      optional :type, :string, 4
      optional :alternative, :int32, 5
      optional :current_distance, :int32, 6
      optional :id, :string, 7
      optional :lateness, :int64, 8
    end
    add_message "ortools_result.CostDetails" do
      optional :fixed, :float, 1
      optional :distance, :float, 2
      optional :distance_balance, :float, 3
      optional :distance_fake, :float, 4
      optional :distance_order, :float, 5
      optional :time, :float, 6
      optional :time_balance, :float, 7
      optional :time_fake, :float, 8
      optional :time_order, :float, 9
      optional :time_without_wait, :float, 10
      optional :value, :float, 11
      optional :lateness, :float, 12
      optional :overload, :float, 13
    end
    add_message "ortools_result.Route" do
      repeated :activities, :message, 1, "ortools_result.Activity"
      optional :cost_details, :message, 2, "ortools_result.CostDetails"
    end
    add_message "ortools_result.Result" do
      optional :cost, :float, 1
      optional :duration, :float, 2
      optional :iterations, :int32, 3
      repeated :routes, :message, 4, "ortools_result.Route"
    end
  end
end

module OrtoolsResult
  Activity = ::Google::Protobuf::DescriptorPool.generated_pool.lookup("ortools_result.Activity").msgclass
  CostDetails = ::Google::Protobuf::DescriptorPool.generated_pool.lookup("ortools_result.CostDetails").msgclass
  Route = ::Google::Protobuf::DescriptorPool.generated_pool.lookup("ortools_result.Route").msgclass
  Result = ::Google::Protobuf::DescriptorPool.generated_pool.lookup("ortools_result.Result").msgclass
end