# Copyright © Mapotempo, 2013-2014
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
require 'sinatra'
require 'tmpdir'

configure do
  set :server, :puma
  set :optimizer_exec, '../optimizer/tsp_simple'
  set :optimizer_tmp_dir, Dir.tmpdir
  set :optimizer_default_time, 30000
  set :optimizer_soft_upper_bound, 3
end

require './optimizer_api'
