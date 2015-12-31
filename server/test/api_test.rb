require File.expand_path '../test_helper.rb', __FILE__

class APITest < MiniTest::Unit::TestCase
  include Rack::Test::Methods

  def app
    Sinatra::Application
  end

  def test_root
    get '/'
    assert last_response.ok?
    assert_equal 'optimizer-api', last_response.body
  end

  def test_api
    matrix = [
      [[0, 0], [1, 1], [1, 1]],
      [[1, 1], [0, 0], [1, 1]],
      [[1, 1], [1, 1], [0, 0]]
    ]
    time_window = [
      [0, 2147483647, 1],
      [0, 2147483647, 1]
    ]
    rest_window = [
      [0, 1, 1]
    ]
    post '/0.1/optimize_tsptw', data: {matrix: matrix, time_window: time_window, rest_window: rest_window, capacity: [], optimize_time: 500, soft_upper_bound: 3}.to_json
    assert last_response.ok?
  end
end
