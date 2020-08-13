using YAML
using ArgParse
using Sockets
using JSON
using LivelyIK
using PyCall

rclpy = pyimport("rclpy")

s = ArgParseSettings()
@add_arg_table! s begin
    "--info_file", "-i"
        arg_type = String
        help = "Full path to the info file"
        required = true
end

parsed_args = parse_args(ARGS, s)
rclpy.init()
info_path = parsed_args["info_file"]
info_data = YAML.load(open(info_path))

lik = LivelyIK.get_standard(info_data)
#
# @async begin
#     server = listen(2000)
#     while true
#         sock = accept(server)
#         @async while isopen(sock)
#             data = JSON.parse(readline(sock))
#
#             write(sock, readline(sock, keep=true))
#         end
#     end
# end
