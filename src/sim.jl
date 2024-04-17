struct EndOfSimException <: Exception end
struct EndOfVehicleException <: Exception end

function vehicle_simulate(state::MechanismState{X}, mviss, vehicle_id, final_time, internal_control!, external_control!, publisher!; Δt=1e-3, stabilization_gains=RigidBodyDynamics.default_constraint_stabilization_gains(X), max_realtime_rate=Inf) where X
    T = RigidBodyDynamics.cache_eltype(state)
    result = DynamicsResult{T}(state.mechanism)
    control_torques = similar(velocity(state))
    external_wrenches = Dict{RigidBodyDynamics.BodyID, RigidBodyDynamics.Wrench{T}}()
    closed_loop_dynamics! = let result=result, control_torques=control_torques, stabilization_gains=stabilization_gains 
        function (v̇::AbstractArray, ṡ::AbstractArray, t, state)
            internal_control!(control_torques, t, state)
            external_control!(external_wrenches, t, state)
            publisher!(t, state)
            dynamics!(result, state, control_torques, external_wrenches; stabilization_gains=stabilization_gains)
            copyto!(v̇, result.v̇)
            copyto!(ṡ, result.ṡ)
            nothing
        end
    end
    tableau = RigidBodyDynamics.runge_kutta_4(T)
    sink = FollowCamSink(mviss, vehicle_id)
    integrator = RigidBodyDynamics.MuntheKaasIntegrator(state, closed_loop_dynamics!, tableau, sink)
    RigidBodyDynamics.integrate(integrator, final_time, Δt; max_realtime_rate)
end

function load_mechanism(; no_mesh=false)
    if no_mesh
        urdf_path = joinpath(dirname(pathof(VehicleSim)), "assets", "chevy_nomesh.urdf")
    else
        urdf_path = joinpath(dirname(pathof(VehicleSim)), "assets", "chevy.urdf")
    end
    chevy_base = parse_urdf(urdf_path, floating=true)
    chevy_joints = joints(chevy_base)
    (; urdf_path, chevy_base, chevy_joints)
end

function logger(info_channel, shutdown_channel, num_vehicles)
    sleep(1.0)
    frequencies = Dict(id => Dict("gps"=>0.0, "imu" => 0.0, "cam" => 0.0, "gt"=>0.0, "sent" => 0.0) for id in 1:num_vehicles)
    foreach(i->println(), 1:num_vehicles+3)

    sim_view_str = ""
    sim_info_str = ""

    vehicles_connected = [false for i in 1:num_vehicles]
    vehicle_connection_str = "Vehicles connected: "
    for i in 1:num_vehicles
        vehicle_connection_str *= "$i : $(vehicles_connected[i]). "
    end
    
    while true
        if isready(shutdown_channel)
            break
        end
        sleep(0.001)
        while isready(info_channel)
            meas = take!(info_channel)
            if meas.msg_type == "freq"
                frequencies[meas.id]["gps"] = meas.gps
                frequencies[meas.id]["imu"] = meas.imu
                frequencies[meas.id]["cam"] = meas.cam
                frequencies[meas.id]["gt"] = meas.gt
                frequencies[meas.id]["sent"] = meas.sent
            elseif meas.msg_type == "sim_status"
                sim_info_str = meas.str
            elseif meas.msg_type == "connection_status"
                vehicles_connected[meas.id] = meas.status
                vehicle_connection_str = "Vehicles connected: "
                for i in 1:num_vehicles
                    vehicle_connection_str *= "$i : $(vehicles_connected[i]). "
                end
            elseif meas.msg_type == "view_status"
                sim_view_str = meas.str
            end
        end
        str = repeat("\033[F", num_vehicles+3)
        print(str)
        for i in 1:num_vehicles
            gps = frequencies[i]["gps"]
            imu = frequencies[i]["imu"]
            cam = frequencies[i]["cam"]
            gt = frequencies[i]["gt"]
            sent = frequencies[i]["sent"]
            @printf("Vehicle %i: gps freq: %6.3f, imu freq: %6.3f, cam freq: %6.3f, gt freq: %6.3f, send freq: %6.3f \u1b[0K \n", i, gps, imu, cam, gt, sent)
        end
        @printf("%s \u1b[0K \n", sim_info_str)
        @printf("%s \u1b[0K \n", vehicle_connection_str)
        @printf("%s \u1b[0K \n", sim_view_str)
        flush(stdout)
    end
end

function server(max_vehicles=1, 
        port=4444; 
        full_state=true,
        open_vis=true,
        no_mesh=false,
        map_type=:city,
        rng=MersenneTwister(1), 
        measure_gps=false, 
        measure_imu=false, 
        measure_cam=false, 
        measure_gt=false,
        measure_all=false,
        monitor_rates=true)

    host = getipaddr()
    if map_type == :city
        map = city_map()
    else
        @assert max_vehicles==1
        map = training_map()
    end
    server_visualizer = get_vis(map, open_vis, host)
    server_info_string = 
        "********************
      CONNECTING TO SERVER. V3
      ********************
        -Connect a keyboard client by running (in a new REPL):
            using VehicleSim, Sockets
            keyboard_client(ip\"$host\");
        -Port for manual clients is $port"
    @info server_info_string
    #@info inform_hostport(server_visualizer, "Server visualizer")
    #client_visualizers = [get_vis(map, false, host) for _ in 1:max_vehicles]
    #all_visualizers = [client_visualizers; server_visualizer]
    all_visualizers = [server_visualizer,]

    (; urdf_path, chevy_base, chevy_joints) = load_mechanism(; no_mesh)
    chevy_visuals = URDFVisuals(urdf_path, package_path=[dirname(pathof(VehicleSim))])

    viable_segments = Set(keys(map))
    spawn_points = Dict()
    vehicles = Dict()

    state_channels = Dict(id=>Channel{MechanismState}(1) for id in 1:max_vehicles)
    cmd_channels = Dict(id=>Channel{VehicleCommand}(1) for id in 1:max_vehicles)
    meas_channels = Dict(id=>Channel{MeasurementMessage}(1) for id in 1:max_vehicles)
    watch_channel = Channel{Int}(1)
    shutdown_channel = Channel{Bool}(1)
    info_channel = Channel{Any}(32)
    put!(watch_channel, 0)

    for vehicle_id in 1:max_vehicles
        local seg
        while true
            seg_id = rand(rng, viable_segments)
            if map_type ≠ :city
                seg_id = 1
                seg = map[seg_id]
                break
            end
            delete!(viable_segments, seg_id)
            if contains_lane_type(map[seg_id], intersection, stop_sign)
                continue
            else
                seg = map[seg_id]
                break
            end
        end
        spawn_points[vehicle_id] = seg
        vehicle = spawn_car_on_map(all_visualizers, seg, chevy_base, chevy_visuals, chevy_joints, vehicle_id)
        @async sim_car(cmd_channels[vehicle_id], state_channels[vehicle_id], shutdown_channel, vehicle, vehicle_id)
        vehicles[vehicle_id] = vehicle
    end

    @async visualize_vehicles(vehicles, state_channels, shutdown_channel, watch_channel)

    measure_gps |= measure_all
    measure_imu |= measure_all
    measure_cam |= measure_all
    measure_gt |= measure_all

    measure_vehicles(map,
                     vehicles,
                     state_channels,
                     meas_channels,
                     info_channel,
                     shutdown_channel;
                     rng,
                     measure_gps,
                     measure_imu,
                     measure_cam,
                     measure_gt)

    print_line_offset = monitor_rates ? max_vehicles : 0

    client_connections = [false for _ in 1:max_vehicles]

    client_count = 0
    sim_task = errormonitor(@async begin
        server = listen(host, port)
        @async begin
            while true
                if isready(shutdown_channel)
                    shutdown = fetch(shutdown_channel)
                    if shutdown
                        put!(info_channel, (; msg_type="sim_status", str="Shutting down TCP server."))
                        #@info "Shutting down TCP server."
                        close(server)
                        break
                    end
                end
                sleep(0.1)
            end
        end
        while true
            try
                put!(info_channel, (; msg_type="sim_status", str="Waiting for client."))
                sock = accept(server)
                #put!(info_channel, (; msg_type="sim_status", str="Client accepted."))
                client_count = mod1(client_count+1, max_vehicles)
                put!(info_channel, (; msg_type="connection_status", id=client_count, status=true))
                if client_connections[client_count]
                    put!(info_channel, (; msg_type="sim_status", str="ERROR! Requested vehicle already in use!"))
                    close(sock)
                    break
                end
                serialize(sock, inform_hostport(all_visualizers[1], "Visualizer"))
                let vehicle_id=client_count
                    @async begin
                        try
                            while isopen(sock)
                                sleep(0.001)
                                if isready(shutdown_channel) && fetch(shutdown_channel)
                                    close(sock)
                                    break
                                end
                                local car_cmd
                                received = false
                                while true
                                    t = @async eof(sock)
                                    if bytesavailable(sock) > 0
                                        car_cmd = deserialize(sock)
                                        received = true
                                    else
                                        #!istaskdone(t) && schedule(t, InterruptException(); error=true)
                                        break
                                    end
                                end
                                #raw_cmd = deserialize(sock)
                                #received = true

                                !received && continue
                                #car_cmd =  VehicleCommand(raw_cmd...)
                                car_cmd =  VehicleCommand(car_cmd...)
                                put!(cmd_channels[vehicle_id], car_cmd)
                                if !car_cmd.controlled
                                    put!(info_channel, (; msg_type="connection_status", id=vehicle_id, status=false))
                                    close(sock)
                               end
                            end
                        catch e
                            put!(info_channel, (; msg_type="sim_status", str="WARNING! Error receiving command for vehicle $vehicle_id. Client may have failed. Closing socket connection to client."))
                            close(sock)
                        end
                    end
                    @async begin
                        try
                            while isopen(sock)
                                sleep(0.001)
                                if isready(shutdown_channel) && fetch(shutdown_channel)
                                    break
                                end
                                if isready(meas_channels[vehicle_id])
                                    msg = take!(meas_channels[vehicle_id])
                                    serialize(sock, msg)
                                end
                            end
                        catch e
                            put!(info_channel, (; msg_type="sim_status", str="WARNING! Error sending measurements to vehicle $vehicle_id. Did client fail?."))
                        end
                    end
                end
            catch e
                break
            end
        end
    end)
    @async logger(info_channel, shutdown_channel, max_vehicles)
    update_viewer(watch_channel, shutdown_channel, max_vehicles, info_channel)
end

function update_viewer(watch_channel, shutdown_channel, max_vehicles, info_channel)
    info_string = 
        "***************
      VIEWER COMMANDS
      ***************
            -Make sure focus is on this terminal window. Then:
            -Press 'q' to shutdown server. 
            -Press '0' to switch to bird's-eye view and release controls to user.
            -Press a number '1'-'9' to view the follow-cam for the associated vehicle. Will default to '0' if vehicle doesn't exist.
            -Use the 'shift' modifier to follow-cam from top-down (e.g. '!' for vehicle 1)."
    @info info_string
    while true
        sleep(0.1)
        key = get_c()

        if key == '!'
            intkey = 11
        elseif key == '@'
            intkey = 12
        elseif key == '#'
            intkey = 13
        elseif key == '$'
            intkey = 14
        elseif key == '%'
            intkey = 15
        elseif key == '^'
            intkey = 16
        elseif key == '&'
            intkey = 17
        elseif key == '*'
            intkey = 18
        elseif key == '('
            intkey = 19
        else
            intkey = try
                parse(Int, key)
            catch err
                nothing
            end
        end
        if key == 'q'
            # terminate vehicle 
            put!(info_channel, (; msg_type="view_status", str="Shutting down server."))
            shutdown!(shutdown_channel)
            break
        elseif key == '0' || (!isnothing(intkey) && mod(intkey,10) > max_vehicles)
            put!(info_channel, (; msg_type="view_status", str="Switching to server view."))
            take!(watch_channel)
            put!(watch_channel, 0)
        elseif key ∈ ['!','@','#','$','%','^','&','*','(']
            put!(info_channel, (; msg_type="view_status", str="Switching view to vehicle $(mod(intkey, 10)) (top-down)"))
            take!(watch_channel)
            put!(watch_channel, intkey)
        elseif key ∈ ['1','2','3','4','5','6','7','8','9']
            put!(info_channel, (; msg_type="view_status", str="Switching view to vehicle $intkey (follow-cam)"))
            take!(watch_channel)
            put!(watch_channel, intkey)
        else
            put!(info_channel, (; msg_type="view_status", str="Unrecognized command, try again."))
        end
    end
end

function shutdown!(shutdown_channel)
    put!(shutdown_channel, true)
    nothing
end

function reset_vehicle!(vehicle, seg)
    mviss = vehicle.mviss
    chevy = vehicle.chevy
    state = vehicle.state

    config = get_initialization_point(seg)
    foreach(mvis->configure_car!(mvis, state, joints(chevy), config), mviss)
end
    
function spawn_car_on_map(visualizers, seg, chevy_base, chevy_visuals, chevy_joints, vehicle_id)
    chevy = deepcopy(chevy_base)
    chevy.graph.vertices[2].name="chevy_$vehicle_id"
    configure_contact_points!(chevy)
    state = MechanismState(chevy)

    mviss = map(visualizers) do vis
        MechanismVisualizer(chevy, chevy_visuals, vis)
    end

    vehicle = (; chevy, state, mviss)
    reset_vehicle!(vehicle, seg)
    vehicle
end

function sim_car(cmd_channel, state_channel, shutdown_channel, vehicle, vehicle_id)

    chevy = vehicle.chevy
    state = vehicle.state
    mviss = vehicle.mviss

    v = 0.0
    δ = 0.0
    controlled = false

    set_reference! = (cmd) -> begin
        v = cmd.velocity # rename
        δ = cmd.steering_angle
        controlled = cmd.controlled
    end

    control! = (torques, t, state) -> begin
            torques .= 0.0
            steering_control!(torques, t, state; reference_angle=δ)
            suspension_control!(torques, t, state)
    end

    wrenches! = (bodyid_to_wrench, t, state) -> begin
        RigidBodyDynamics.update_transforms!(state)
        wheel_control!(bodyid_to_wrench, chevy, t, state; reference_velocity=v)
    end

    publisher! = (t, state) -> begin
        if isready(shutdown_channel)
            fetch(shutdown_channel) && throw(error("Shutdown!"))
        end
        if isready(state_channel)
            stale = take!(state_channel)
        end
        put!(state_channel, state)
    end

    @async while true
        sleep(0.001)
        if isready(shutdown_channel)
            fetch(shutdown_channel) && break
        end
        if isready(cmd_channel)
            car_cmd = take!(cmd_channel)
            set_reference!(car_cmd)
        end
    end

    try 
        vehicle_simulate(state, 
                         mviss,
                         vehicle_id,
                         Inf, 
                         control!, 
                         wrenches!,
                         publisher!;
                         max_realtime_rate=1.0)
    catch e
        @warn "Terminating simulation for vehicle $vehicle_id."
        foreach(mvis->delete_vehicle!(mvis), mviss)
    end
end

