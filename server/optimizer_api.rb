require 'json'
require 'tempfile'


get '/' do
  'optimizer-api'
end

post '/0.1/optimize_tsptw' do
  jdata = JSON.parse(params[:data])
  begin
    optim = optimize(jdata['capacity'], jdata['matrix'], jdata['time_window'])
    if !optim
      halt(500)
    end
    {status: :ok, optim: optim}.to_json
  rescue Exception => e
    halt(500, e.to_s)
  end
end


def optimize(capacity, matrix, time_window)
  @exec = settings.optimizer_exec
  @tmp_dir = settings.optimizer_tmp_dir
  @time = settings.optimizer_time

  input = Tempfile.new('optimize-route-input', tmpdir=@tmp_dir)
  output = Tempfile.new('optimize-route-output', tmpdir=@tmp_dir)

  begin
    output.close

    input.write(matrix.size)
    input.write("\n")
    input.write(matrix.collect{ |a| a.collect{ |b| b.join(" ") }.join(" ") }.join("\n"))
    input.write("\n")
    input.write(time_window.collect{ |a| [a[0] ? a[0]:0, a[1]? a[1]:2147483647, a[2]].join(" ") }.join("\n"))
    input.write("\n")

    input.close

    cmd = "#{@exec} -time_limit_in_ms #{@time} -instance_file '#{input.path}' > '#{output.path}'"
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
