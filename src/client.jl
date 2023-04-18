struct VehicleCommand
    steering_angle::Float64
    velocity::Float64
    controlled::Bool
end

function get_c()
    ret = ccall(:jl_tty_set_mode, Int32, (Ptr{Cvoid},Int32), stdin.handle, true)
    ret == 0 || error("unable to switch to raw mode")
    c = read(stdin, Char)
    ccall(:jl_tty_set_mode, Int32, (Ptr{Cvoid},Int32), stdin.handle, false)
    c
end

function keyboard_client(host::IPAddr=IPv4(0), port=4444; v_step = 1.0, s_step = π/10)
    start_time = time()
    
    socket = Sockets.connect(host, port)

    end_time = time()

    map_segments = training_map()

    response_time = (end_time - start_time) * 1000  # convert to milliseconds
    println("Response time: $response_time ms")

    (peer_host, peer_port) = getpeername(socket)
    msg = deserialize(socket) # Visualization info
    @info msg

    last_target = -1
    current_id = -1
    current_pos = SVector(0.0, 0.0, 0.0)

    @async try 
        while isopen(socket)
            sleep(0.001)
            state_msg = deserialize(socket)
    
            target_map_segment = state_msg.target_segment

            if target_map_segment != last_target
                println("Target segment: $target_map_segment")
                last_target = target_map_segment
            end

            measurements = state_msg.measurements
            num_cam = 0
            num_imu = 0
            num_gps = 0
            num_gt = 0
    
            for meas in measurements
                if meas isa GroundTruthMeasurement
                    num_gt += 1
                    
                    # get curr pos
                    if current_id != meas.vehicle_id
                        current_id = meas.vehicle_id
                        println("Vehicle id: $current_id")
                    end
                    current_pos = meas.position
    
                    # scan map for segment with pos
                    # for i in 1:length(map_segments)
                    #     seg = map_segments[i]
                    #     if seg.start <= curr_pos <= seg.stop
                    #         println("curr seg: ", i)
                    #         break
                    #     end
                    # end
    
                    # println(meas.position)
                elseif meas isa CameraMeasurement
                    num_cam += 1
                elseif meas isa IMUMeasurement
                    num_imu += 1
                elseif meas isa GPSMeasurement
                    num_gps += 1
                end
            end
      #      @info "Measurements received: $num_gt gt; $num_cam cam; $num_imu imu; $num_gps gps"
        end
    catch e
        print("Error encountered: $e")
        @info "Client disconnected."
    end
    
    target_velocity = 0.0
    steering_angle = 0.0
    controlled = true
    @info "Press 'q' at any time to terminate vehicle."
    while controlled && isopen(socket)
        key = get_c()
        if key == 'q'
            # terminate vehicle
            controlled = false
            target_velocity = 0.0
            steering_angle = 0.0
            @info "Terminating Keyboard Client."
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
        elseif key == 'v'
            # print segment
            @info "Current pos: $current_pos"
            seg = get_segment(map_segments, current_pos)
            @info "Segment id: $seg"


        end
        cmd = VehicleCommand(steering_angle, target_velocity, controlled)
        serialize(socket, cmd)
    end
end

function example_client(host::IPAddr=IPv4(0), port=4444)
    socket = Sockets.connect(host, port)
    map_segments = training_map()
    (; chevy_base) = load_mechanism()

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
