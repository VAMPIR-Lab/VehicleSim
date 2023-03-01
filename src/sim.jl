function vehicle_simulate(state::MechanismState{X}, mvis, final_time, internal_control!, external_control!; Δt=1e-4, stabilization_gains=RigidBodyDynamics.default_constraint_stabilization_gains(X), max_realtime_rate=Inf) where X
    T = RigidBodyDynamics.cache_eltype(state)
    result = DynamicsResult{T}(state.mechanism)
    control_torques = similar(velocity(state))
    external_wrenches = Dict{RigidBodyDynamics.BodyID, RigidBodyDynamics.Wrench{T}}()
    closed_loop_dynamics! = let result=result, control_torques=control_torques, stabilization_gains=stabilization_gains 
        function (v̇::AbstractArray, ṡ::AbstractArray, t, state)
            internal_control!(control_torques, t, state)
            external_control!(external_wrenches, t, state)
            dynamics!(result, state, control_torques, external_wrenches; stabilization_gains=stabilization_gains)
            copyto!(v̇, result.v̇)
            copyto!(ṡ, result.ṡ)
            nothing
        end
    end
    tableau = RigidBodyDynamics.runge_kutta_4(T)
    sink = MeshCatMechanisms.MeshCatSink(mvis)
    integrator = RigidBodyDynamics.MuntheKaasIntegrator(state, closed_loop_dynamics!, tableau, sink)
    RigidBodyDynamics.integrate(integrator, final_time, Δt; max_realtime_rate)
end

function car_sim_server(vis=nothing, host::IPAddr = IPv4(0), port=4444)
    if isnothing(vis)
        vis = get_vis()
    end
    urdf_path = joinpath(dirname(pathof(VehicleSim)), "assets", "chevy.urdf")
    chevy_base = parse_urdf(urdf_path, floating=true)
    chevy_visuals = URDFVisuals(urdf_path, package_path=[dirname(pathof(VehicleSim))])
    chevy_joints = joints(chevy_base)

    vehicle_count = 0
    errormonitor(@async begin
        server = listen(host, port)
        while true
            sock = accept(server)
            println("Client accepted!", sock)
            errormonitor(
                         @async spawn_car(vis, sock, chevy_base, chevy_visuals, chevy_joints, vehicle_count+=1))
        end
    end)
end
