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


