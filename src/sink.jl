mutable struct FollowCamSink <: RigidBodyDynamics.OdeIntegrators.OdeResultsSink
    mviss::Vector{MechanismVisualizer}
    follow_cam_id::Int
    min_wall_Δt::Float64
    last_update_wall_time::Float64
    follow_dist::Float64
    follow_height::Float64
    follow_offset::Float64

    function FollowCamSink(mviss, follow_cam_id; 
            max_fps::Float64 = 60., 
            follow_dist=35.0, 
            follow_height=6.0,
            follow_offset=6.0)
        new(mviss, follow_cam_id, 1 / max_fps, -Inf, follow_dist, follow_height, follow_offset)
    end
end

function RigidBodyDynamics.OdeIntegrators.initialize(sink::FollowCamSink, t, state)
    sink.last_update_wall_time = -Inf
    RigidBodyDynamics.OdeIntegrators.process(sink, t, state)
end

function RigidBodyDynamics.OdeIntegrators.process(sink::FollowCamSink, t, state)
    wall_Δt = time() - sink.last_update_wall_time
    if wall_Δt > sink.min_wall_Δt
        sink.last_update_wall_time = time()
    end
    nothing
end
