require File.expand_path '../test_helper.rb', __FILE__

class Disjunc < MiniTest::Unit::TestCase
  include Rack::Test::Methods

  def app
    Sinatra::Application
  end

  def test_neg_window
    matrix = [
      [[0, 0], [64, 71], [853, 947], [787, 874], [747, 830], [0, 0]],
      [[94, 104], [0, 0], [789, 876], [801, 890], [761, 845], [94, 104]],
      [[857, 952], [826, 917], [0, 0], [819, 910], [746, 828], [857, 952]],
      [[837, 930], [882, 980], [823, 914], [0, 0], [247, 274], [837, 930]],
      [[802, 891], [846, 940], [743, 825], [240, 260], [0, 0], [802, 891]],
      [[0, 0], [64, 71], [853, 947], [787, 874], [747, 830], [0, 0]]
    ]

    time_window = [
      [0, 1600, 3200, 4800, 1],
      [800, 1600, 2100, 3000, 1],
      [400, 1200, 1600, 2400, 1],
      [-200, -50, -45, -2, 1],
      [-100, 300, 500, 900, 1]
    ]
    rest_window = [
      [600, 1200, 1800, 2400, 1200]
    ]
    post '/0.1/optimize_tsptw', data: {matrix: matrix, time_window: time_window, rest_window: rest_window, capacity: [], optimize_time: 500, soft_upper_bound: 3}.to_json
    assert last_response.ok?
    json = JSON.parse(last_response.body)
    puts "test_neg_window"
    assert_equal 'ok', json['status']
    assert_equal 7, json['optim'].collect{ |i| Integer(i) }.size
    assert_equal 0, json['optim'].first
    assert_equal (matrix.size-1), json['optim'].last
  end

  def test_standard
    matrix = [
      [[0, 0], [64, 71], [853, 947], [787, 874], [747, 830], [0, 0]],
      [[94, 104], [0, 0], [789, 876], [801, 890], [761, 845], [94, 104]],
      [[857, 952], [826, 917], [0, 0], [819, 910], [746, 828], [857, 952]],
      [[837, 930], [882, 980], [823, 914], [0, 0], [247, 274], [837, 930]],
      [[802, 891], [846, 940], [743, 825], [240, 260], [0, 0], [802, 891]],
      [[0, 0], [64, 71], [853, 947], [787, 874], [747, 830], [0, 0]]
    ]

    time_window = [
      [-2147483648, 2147483647, -2147483648, 2147483647, 1],
      [-2147483648, 2147483647, -2147483648, 2147483647, 1],
      [-2147483648, 2147483647, -2147483648, 2147483647, 1],
      [-2147483648, 2147483647, -2147483648, 2147483647, 1],
      [-2147483648, 2147483647, -2147483648, 2147483647, 1]
    ]
    rest_window = [
      [600, 1200, 1800, 2400, 1200]
    ]

    post '/0.1/optimize_tsptw', data: {matrix: matrix, time_window: time_window, rest_window: rest_window, capacity: [], optimize_time: 500, soft_upper_bound: 3}.to_json

    assert last_response.ok?
    json = JSON.parse(last_response.body)
    puts "test_standard"
    assert_equal 'ok', json['status']
    assert_equal 7, json['optim'].size
    assert_equal 0, json['optim'].first
    assert_equal matrix.size - 1, json['optim'].last
  end

  def test_no_rest
    matrix = [
      [[0, 0], [64, 71], [853, 947], [787, 874], [747, 830], [0, 0]],
      [[94, 104], [0, 0], [789, 876], [801, 890], [761, 845], [94, 104]],
      [[857, 952], [826, 917], [0, 0], [819, 910], [746, 828], [857, 952]],
      [[837, 930], [882, 980], [823, 914], [0, 0], [247, 274], [837, 930]],
      [[802, 891], [846, 940], [743, 825], [240, 260], [0, 0], [802, 891]],
      [[0, 0], [64, 71], [853, 947], [787, 874], [747, 830], [0, 0]]
    ]

    time_window = [
      [-2147483648, 2147483647, -2147483648, 2147483647, 1],
      [-2147483648, 2147483647, -2147483648, 2147483647, 1],
      [-2147483648, 2147483647, -2147483648, 2147483647, 1],
      [-2147483648, 2147483647, -2147483648, 2147483647, 1],
      [-2147483648, 2147483647, -2147483648, 2147483647, 1]
    ]
    rest_window = []

    post '/0.1/optimize_tsptw', data: {matrix: matrix, time_window: time_window, rest_window: rest_window, capacity: [], optimize_time: 500, soft_upper_bound: 3}.to_json

    assert last_response.ok?
    json = JSON.parse(last_response.body)
    puts "test_no_rest"
    assert_equal 'ok', json['status']
    assert_equal 6, json['optim'].size
    assert_equal 0, json['optim'].first
    assert_equal matrix.size - 1, json['optim'].last
  end

  def test_disjoint_tw
    matrix = [
      [[0, 0], [64, 71], [853, 947], [787, 874], [747, 830], [0, 0]],
      [[94, 104], [0, 0], [789, 876], [801, 890], [761, 845], [94, 104]],
      [[857, 952], [826, 917], [0, 0], [819, 910], [746, 828], [857, 952]],
      [[837, 930], [882, 980], [823, 914], [0, 0], [247, 274], [837, 930]],
      [[802, 891], [846, 940], [743, 825], [240, 260], [0, 0], [802, 891]],
      [[0, 0], [64, 71], [853, 947], [787, 874], [747, 830], [0, 0]]
    ]

    time_window = [
      [0, 1, 1000, 2000, 1],
      [0, 1, 3000, 4000, 1],
      [0, 1, 5000, 6000, 1],
      [0, 1, 7000, 8000, 1],
      [0, 1, 9000, 10000, 1]
    ]
    rest_window = []

    post '/0.1/optimize_tsptw', data: {matrix: matrix, time_window: time_window, rest_window: rest_window, capacity: [], optimize_time: 500, soft_upper_bound: 3}.to_json

    assert last_response.ok?
    json = JSON.parse(last_response.body)
    puts "test_disjoint_tw"
    assert_equal 'ok', json['status']
    assert_equal 6, json['optim'].size
    assert_equal 0, json['optim'].first
    assert_equal matrix.size - 1, json['optim'].last
  end
end
