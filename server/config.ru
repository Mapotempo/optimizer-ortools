require 'sinatra'
require 'tmpdir'

configure do
  set :server, :puma
  set :optimizer_exec, '../optimizer/tsp_simple'
  set :optimizer_tmp_dir, Dir.tmpdir
  set :optimizer_time, 20000
end

require './optimizer_api'
