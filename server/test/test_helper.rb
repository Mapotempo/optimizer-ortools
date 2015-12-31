ENV['RACK_ENV'] = 'test'
require 'minitest/autorun'
require 'rack/test'

require 'sinatra'
require 'tmpdir'

configure do
  set :server, :puma
  set :optimizer_exec, '../optimizer/tsp_simple'
  set :optimizer_tmp_dir, Dir.tmpdir
  set :optimizer_default_time, 100
  set :optimizer_soft_upper_bound, 3
end

require File.expand_path '../../optimizer_api.rb', __FILE__
