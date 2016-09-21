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
require 'json'
require 'tempfile'


get '/' do
  'optimizer-api'
end

post '/0.1/optimize_tsptw' do
  jdata = JSON.parse(params[:data])
  begin
    optim = optimize(jdata['capacity'], jdata['matrix'], jdata['time_window'], jdata['rest_window'], {iterations_without_improvement: jdata['iterations_without_improvement'], optimize_time: jdata['optimize_time'], soft_upper_bound: jdata['soft_upper_bound'], initial_time_out: jdata['initial_time_out'], time_out_multiplier: jdata['time_out_multiplier']})
    if !optim
      puts "No optim result !"
      halt(500)
    end
    {status: :ok, optim: optim}.to_json
  rescue Exception => e
    puts e
    puts e.backtrace.join("\n")
    halt(500, e.to_s)
  end
end


def optimize(capacity, matrix, time_window, rest_window, options = {})
  @exec = settings.optimizer_exec
  @tmp_dir = settings.optimizer_tmp_dir
  @iterations_without_improvement = options[:iterations_without_improvement] || settings.optimizer_default_iterations_without_improvement
  @time = options[:optimize_time]
  @soft_upper_bound = options[:soft_upper_bound] || settings.optimizer_soft_upper_bound
  @initial_time_out = options[:initial_time_out]
  @time_out_multiplier = options[:time_out_multiplier]

  input = Tempfile.new('optimize-route-input', tmpdir=@tmp_dir)
  output = Tempfile.new('optimize-route-output', tmpdir=@tmp_dir)

  begin
    output.close

    input.write(matrix.size)
    input.write("\n")
    input.write(rest_window.size)
    input.write("\n")
    input.write(matrix.collect{ |a| a.collect{ |b| b.join(" ") }.join(" ") }.join("\n"))
    input.write("\n")
    input.write((time_window + [[0, 2147483647, -2147483648, 2147483647, 0]]).collect{ |a|
      [a[0] ? a[0]:-2147483648, a[1]? a[1]:2147483647, a[2] ? a[2]:-2147483648, a[3]? a[3]:2147483647, a[4]].join(" ")
    }.join("\n"))
    input.write("\n")
    input.write(rest_window.collect{ |a|
      [a[0] ? a[0]:-2147483648, a[1]? a[1]:2147483647, a[2] ? a[2]:-2147483648, a[3]? a[3]:2147483647, a[4]].join(" ")
    }.join("\n"))
    input.write("\n")

    input.close

    cmd = "#{@exec} -no_solution_improvement_limit #{@iterations_without_improvement} -time_out_multiplier 2" + (@time ? " -time_limit_in_ms #{@time}" : '') + " -soft_upper_bound #{@soft_upper_bound} -nearby" + (@initial_time_out ? " -initial_time_out_no_solution_improvement #{@initial_time_out}" : '') + (@time_out_multiplier ? " -time_out_multiplier #{@time_out_multiplier}" : '') + " -instance_file '#{input.path}' > '#{output.path}'"
    puts cmd
    system(cmd)
    puts $?.exitstatus
    if $?.exitstatus == 0
      result = File.read(output.path)
      result = result.split("\n")[-1]
      puts result.inspect
      result.split(' ').collect{ |i| Integer(i) }
    end
  ensure
    input.unlink
    output.unlink
  end
end