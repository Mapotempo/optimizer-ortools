# Copyright © Mapotempo, 2016
#
# This file is part of Mapotempo.
#
# Mapotempo is free software. You can redistribute it and/or
# modify since you respect the terms of the GNU Affero General
# Public License as published by the Free Software Foundation,
# either version 3 of the License, or (at your option) any later version.
#
# Mapotempo is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE.  See the Licenses for more details.
#
# You should have received a copy of the GNU Affero General Public License
# along with Mapotempo. If not, see:
# <http://www.gnu.org/licenses/agpl.html>
#
require './test/test_helper'

require './api/root'


class Api::V01::VrpTest < Minitest::Test
  include Rack::Test::Methods

  def app
    Api::Root
  end

  def test_vrp
    vrp = {
      points: [{
        id: 'p1',
        location: {
          lat: 1,
          lon: 2
        }
      }],
      vehicles: [{
        id: 'v1'
      }],
      services: [{
        id: 's1',
        activity: {
          point_id: 'p1'
        }
      }],
      configuration: {
        resolution: {
          duration: 1
        }
      }
    }
    post '/0.1/vrp/submit', {api_key: 'ortools', vrp: vrp}
    assert_equal 201, last_response.status, last_response.body
    assert JSON.parse(last_response.body)['job']['id']
  end

  def test_vrp
    get '/0.1/vrp/jobs.json', {api_key: 'ortools'}
    assert_equal 201, last_response.status, last_response.body
    assert_not_nil last_response.body, last_response.body
  end

  def test_real_probleme_without_matrix
    vrp = {
      points: [{
        id: 'point_0',
        location: {lat: 45.288798, lon: 4.951565}
      }, {
        id: 'point_1',
        location: {lat: 45.6047844887, lon: 4.7589656711}
      }, {
        id: 'point_2',
        location: {lat: 45.6047844887, lon: 4.7589656711}
      }, {
        id: 'point_3',
        location: {lat: 45.344334, lon: 4.817731}
      }, {
        id: 'point_4',
        location: {lat: 45.5764120817, lon: 4.8056146502}
      }, {
        id: 'point_5',
        location: {lat: 45.5764120817, lon: 4.8056146502}
      }, {
        id: 'point_6',
        location: {lat: 45.2583248913, lon: 4.6873225272}
      }],
      vehicles: [{
        id: 'vehicle_0',
        start_point_id: 'point_0',
        end_point_id: 'point_0',
        router_mode: 'car',
        router_dimension: 'distance',
      }],
      services: [{
        id: 'service_1',
        type: 'service',
        activity: {
          point_id: 'point_1'
        }
      }, {
        id: 'service_2',
        type: 'service',
        activity: {
          point_id: 'point_2'
        }
      }, {
        id: 'service_3',
        type: 'service',
        activity: {
          point_id: 'point_3'
        }
      }, {
        id: 'service_4',
        type: 'service',
        activity: {
          point_id: 'point_4'
        }
      }, {
        id: 'service_5',
        type: 'service',
        activity: {
          point_id: 'point_5'
        }
      }, {
        id: 'service_6',
        type: 'service',
        activity: {
          point_id: 'point_6'
        }
      }],
    }

    post '/0.1/vrp/submit', {api_key: 'vroom', vrp: vrp}
    assert_equal 200, last_response.status, last_response.body
    assert_equal 1.upto(6).collect{ |i| "point_#{i}"}, JSON.parse(last_response.body)['solutions'][0]['routes'][0]['activities'][1..-2].collect{ |p| p['point_id'] }
  end
end
