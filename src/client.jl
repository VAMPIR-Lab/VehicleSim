struct VehicleCommand
    steering_angle::Float64
    velocity::Float64
    persist::Bool
    shutdown::Bool
end

function get_c()
    ret = ccall(:jl_tty_set_mode, Int32, (Ptr{Cvoid},Int32), stdin.handle, true)
    ret == 0 || error("unable to switch to raw mode")
    c = read(stdin, Char)
    ccall(:jl_tty_set_mode, Int32, (Ptr{Cvoid},Int32), stdin.handle, false)
    c
end

function keyboard_client(host::IPAddr=IPv4(0), port=4444; v_step = 1.0, s_step = π/10)
    socket = Sockets.connect(host, port)
    (peer_host, peer_port) = getpeername(socket)
    msg = deserialize(socket) # Visualization info
    @info msg

    local state_msg
    @async while isopen(socket)
        state_msg = deserialize(socket)
    end
    
    target_velocity = 0.0
    steering_angle = 0.0
    persist = true
    shutdown = false
    @info "Press 'q' at any time to terminate vehicle. Press 's' to shutdown simulator server."
    while persist && !shutdown && isopen(socket)
        key = get_c()
        if key == 'q'
            # terminate vehicle
            persist = false
        elseif key == 's'
            # shutdown server
            shutdown = true
        elseif key == 'i'
            # increase target velocity
            target_velocity += v_step
            @info "Target velocity: $target_velocity"
        elseif key == 'k'
            # decrease forward force
            target_velocity -= v_step
            @info "Target velocity: $target_velocity"
        elseif key == 'j'
            # increase steering angle
            steering_angle += s_step
            @info "Target steering angle: $steering_angle"
        elseif key == 'l'
            # decrease steering angle
            steering_angle -= s_step
            @info "Target steering angle: $steering_angle"
        end
        cmd = VehicleCommand(steering_angle, target_velocity, persist, shutdown)        
        serialize(socket, cmd)
    end
end

function example_client(host::IPAddr=IPv4(0), port=4444)
    socket = Sockets.connect(host, port)
    map_segments = training_map()
    (; chevy_base) = load_mechanism()

    state_msg = VehicleState(zeros(13), zeros(12), false)
    @async while isopen(socket)
        state_msg = deserialize(socket)
    end
   
    shutdown = false
    persist = true
    while isopen(socket)
        position = state_msg.q[5:7]
        @info position
        if norm(position) >= 100
            shutdown = true
            persist = false
        end
        cmd = VehicleCommand(0.0, 2.5, persist, shutdown)
        serialize(socket, cmd) 
    end

end
