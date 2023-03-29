
struct EndOfSimException <: Exception end
struct EndOfVehicleException <: Exception end

function vehicle_simulate(state::MechanismState{X}, mviss, vehicle_id, final_time, internal_control!, external_control!, publisher!; Δt=1e-4, stabilization_gains=RigidBodyDynamics.default_constraint_stabilization_gains(X), max_realtime_rate=Inf) where X
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

function vis_updater(mvisualizers, channels;
                     follow_dist=35.0, 
                     follow_height=6.0,
                     follow_offset=6.0)
    while true
        for (id, channel) in channels
            if isready(channel)
                vehicle_state = take!(channel)
                if vehicle_state.persist
                    for (e, mvis) in enumerate(mvisualizers)
                        set_configuration!(mvis, configuration(state))
                        if e == id
                            config = configuration(state)
                            quat = config[1:4]
                            pose = config[5:7]
                            yaw = extract_yaw_from_quaternion(quat) 
                            offset = [sink.follow_dist * [cos(yaw), sin(yaw)]; -sink.follow_height] + sink.follow_offset * [sin(yaw), -cos(yaw), 0]
                            setcameratarget!(sink.vis[sink.follow_cam_id].visualizer, pose)
                            setcameraposition!(sink.vis[sink.follow_cam_id].visualizer, pose-offset)
                        end
                    end
                else
                    foreach(mvis->delete_car(mvis), mvisualizers)
                end
            end
        end
    end
end

function load_mechanism()
    urdf_path = joinpath(dirname(pathof(VehicleSim)), "assets", "chevy.urdf")
    chevy_base = parse_urdf(urdf_path, floating=true)
    chevy_joints = joints(chevy_base)
    (; urdf_path, chevy_base, chevy_joints)
end

function server(max_vehicles=1, port=4444; full_state=true, rng=MersenneTwister(1))
    host = getipaddr()
    map = training_map()
    server_visualizer = get_vis(map, true, host)
    @info "Server can be connected to at $host and port $port"
    @info inform_hostport(server_visualizer, "Server visualizer")
    client_visualizers = [get_vis(map, false, host) for _ in 1:max_vehicles]
    all_visualizers = [client_visualizers; server_visualizer]

    (; urdf_path, chevy_base, chevy_joints) = load_mechanism()
    chevy_visuals = URDFVisuals(urdf_path, package_path=[dirname(pathof(VehicleSim))])

    viable_segments = Set(keys(map))
    spawn_points = Dict()
    vehicles = Dict()

    state_channels = Dict(id=>Channel{MechanismState}(1) for id in 1:max_vehicles)
    cmd_channels = Dict(id=>Channel{VehicleCommand}(1) for id in 1:max_vehicles)
    meas_channels = Dict(id=>Channel{MeasurementMessage}(1) for id in 1:max_vehicles)

    for vehicle_id in 1:max_vehicles
        local seg
        while true
            seg_id = rand(rng, viable_segments)
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
        @async sim_car(all_visualizers, cmd_channels[vehicle_id], state_channels[vehicle_id], vehicle, vehicle_id)
        vehicles[vehicle_id] = vehicle
    end
    @infiltrate

    shutdown_channel = Channel{Bool}(1)

    @async measure_vehicles(map, vehicles, state_channels, meas_channels, shutdown_channel; rng)

    client_connections = [false for _ in 1:max_vehicles]

    client_count = 0
    sim_task = errormonitor(@async begin
        server = listen(host, port)
        @async begin
            while true
                if isready(shutdown_channel)
                    shutdown = fetch(shutdown_channel)
                    if shutdown
                        close(server)
                        break
                    end
                end
                sleep(0.1)
            end
        end
        while true
            try
                sock = accept(server)
                @info "Client accepted."
                client_count = mod1(client_count+1, max_vehicles)
                if client_connections[client_count]
                    @error "Requested vehicle already in use!"
                    close(sock)
                    break
                end
                serialize(sock, inform_hostport(client_visualizers[client_count], "Client follow-cam"))
                let vehicle_id=client_count
                    @async begin
                        while isopen(sock)
                            car_cmd = deserialize(sock)
                            put!(cmd_channels[vehicle_id], car_cmd)
                            if !car_cmd.controlled
                                close(sock)
                            end
                        end
                    end
                    @async begin
                        while isopen(sock)
                            msg = take!(meas_channels[vehicle_id])
                            serialize(sock, msg)
                        end
                    end  
                end
                #@info "Client accepted."
                #let client_count=client_count
                #    errormonitor(@async begin
                #        active_simulations[client_count] = true
                #        sim_car(all_visualizers, 
                #                sock, 
                #                sim_channels[client_count], 
                #                vehicles[client_count], 
                #                client_count, 
                #                server)
                #        active_simulations[client_count] = false
                #        reset_vehicle!(vehicles[client_count], spawn_points[client_count])
                #    end)
                #end
            catch e
                @info "Shutting down server."
                close(server)
                break
            end
        end
    end)
    shutdown_channel
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

function sim_car(visualizers, cmd_channel, state_channel, vehicle, vehicle_id)

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
            !persist && throw(EndOfVehicleException())
            torques .= 0.0
            steering_control!(torques, t, state; reference_angle=δ)
            suspension_control!(torques, t, state)
    end
    wrenches! = (bodyid_to_wrench, t, state) -> begin
        RigidBodyDynamics.update_transforms!(state)
        wheel_control!(bodyid_to_wrench, chevy, t, state; reference_velocity=v)
    end
    publisher! = (t, state) -> begin
        if isready(state_channel)
            stale = take!(state_channel)
        end
        put!(state_channel, state)
    end

    @async while true
        car_cmd = take!(cmd_channel)
        set_reference!(car_cmd)
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
        foreach(mvis->delete_vehicle!(mvis), mviss)
    end
end

