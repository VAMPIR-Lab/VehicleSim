struct VehicleState
    q::Vector{Float64}
    v::Vector{Float64}
    persist::Bool
end

struct EndOfSimException <: Exception end
struct EndOfVehicleException <: Exception end

function vehicle_simulate(state::MechanismState{X}, channel, final_time, internal_control!, external_control!, publisher!; Δt=1e-4, stabilization_gains=RigidBodyDynamics.default_constraint_stabilization_gains(X), max_realtime_rate=Inf) where X
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
    sink = PublisherSink(channel)
    integrator = RigidBodyDynamics.MuntheKaasIntegrator(state, closed_loop_dynamics!, tableau, sink)
    RigidBodyDynamics.integrate(integrator, final_time, Δt; max_realtime_rate)
end

function vis_updater(visualizers, channels;
            follow_dist=35.0, 
            follow_height=6.0,
            follow_offset=6.0)
    while true
        for (id, channel) in channels
            if isready(channel)
                state = take!(channel)
            end
        end
    end
end



function server(max_clients=4, host::IPAddr = IPv4(0), port=4444)
    map = training_map()
    server_visualizer = get_vis(map, true)
    inform_hostport(server_visualizer, "Server visualizer")
    client_visualizers = [get_vis(map, false) for _ in 1:max_clients]
    client_channels = Dict(i=>Channel{VehicleState}(1) for i = 1:max_clients)
    all_visualizers = [client_visualizers; server_visualizer]

    urdf_path = joinpath(dirname(pathof(VehicleSim)), "assets", "chevy.urdf")
    chevy_base = parse_urdf(urdf_path, floating=true)
    chevy_visuals = URDFVisuals(urdf_path, package_path=[dirname(pathof(VehicleSim))])
    chevy_joints = joints(chevy_base)

    vehicle_count = 0
    sim_task = errormonitor(@async begin
        server = listen(host, port)
        while true
            try
                sock = accept(server)
                @info "Client accepted."
                vehicle_count += 1
                channel = channels[vehicle_count]
                open(client_visualizers[vehicle_count])
                inform_hostport(client_visualizers[vehicle_count], "Client follow-cam")
                @async spawn_car(channel, sock, chevy_base, chevy_visuals, chevy_joints, vehicle_count, server)
            catch e
                break
            end
        end
    end)
     
end

function spawn_car(channel, sock, chevy_base, chevy_visuals, chevy_joints, vehicle_id, server)
    chevy = deepcopy(chevy_base)
    chevy.graph.vertices[2].name="chevy_$vehicle_id"
    configure_contact_points!(chevy)
    state = MechanismState(chevy)

    config = CarConfig(SVector(-7,12,2.5), 0.0, 0.0, 0.0, 0.0)
    configure_car!(nothing, state, joints(chevy), config)

    v = 0.0
    θ = 0.0
    state_q = state.q
    state_v = state.v
    persist = true
    shutdown = false
    set_reference! = (cmd) -> begin
        if !cmd.persist 
            @info "Destroying vehicle."
            close(sock)
        end
        if cmd.shutdown
            @info "Shutting down server."
            close(sock)
        end
            
        v = cmd.forward_force # rename
        θ = cmd.steering_angle
        shutdown=cmd.shutdown
        persist=cmd.persist
    end
    control! = (torques, t, state) -> begin
            !persist && throw(EndOfVehicleException())
            shutdown && throw(EndOfSimException())
            torques .= 0.0
            steering_control!(torques, t, state; reference_angle=θ)
            suspension_control!(torques, t, state)
            nothing
    end
    wrenches! = (bodyid_to_wrench, t, state) -> begin
        RigidBodyDynamics.update_transforms!(state)
        wheel_control!(bodyid_to_wrench, chevy, t, state; reference_velocity=v)
    end
    publisher! = (t, state) -> begin
        state_q .= state.q
        state_v .= state.v
    end

    @async while isopen(sock)
        car_cmd = deserialize(sock)
        set_reference!(car_cmd)
    end 
    
    @async while isopen(sock)
        state_msg = VehicleState(state_q, state_v)
        serialize(sock, state_msg)
    end

    try 
        println(mvisualizers)
        vehicle_simulate(state, 
                         channel,
                         Inf, 
                         control!, 
                         wrenches!,
                         publisher!;
                         max_realtime_rate=1.0)
    catch e
        #delete_vehicle(vis, chevy) TODO send on channel?
        put!(channel, VehicleState(state.q, state.v, false))
        if e isa EndOfSimException
            close(server)
        end
    end
end

function client(host::IPAddr=IPv4(0), port=4444, control=keyboard_controller)
    socket = Sockets.connect(host, port)

    state_msg = VehicleState(zeros(0), zeros(0))

    @async while isopen(socket)
        state_msg = deserialize(socket)
    end

    control(socket)
end
