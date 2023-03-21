mutable struct FollowCamSink{M <: MechanismState} <: RigidBodyDynamics.OdeIntegrators.OdeResultsSink
    vis::Vector{MechanismVisualizer{M}}
    follow_cam_id::Int
    min_wall_Δt::Float64
    last_update_wall_time::Float64
    follow_dist::Float64
    follow_height::Float64
    follow_offset::Float64

    function FollowCamSink(vis::Vector{MechanismVisualizer{M}}, follow_cam_id; 
            max_fps::Float64 = 60., 
            follow_dist=35.0, 
            follow_height=6.0,
            follow_offset=6.0) where {M <: MechanismState}
        println("Made it here!")
        new{M}(vis, follow_cam_id, 1 / max_fps, -Inf, follow_dist, follow_height, follow_offset)
    end
end

mutable struct PublisherSink <: RigidBodyDynamics.OdeIntegrators.OdeResultsSink
    channel::Channel{VehicleState}
    min_wall_Δt::Float64
    last_update_wall_time::Float64

    function PublisherSink(channel; max_fps::Float64 = 60.)
        new(channel, 1 / max_fps, -Inf)
    end
end

function RigidBodyDynamics.OdeIntegrators.initialize(sink::Union{PublisherSink,FollowCamSink}, t, state)
    sink.last_update_wall_time = -Inf
    RigidBodyDynamics.OdeIntegrators.process(sink, t, state)
end

function RigidBodyDynamics.OdeIntegrators.process(sink::FollowCamSink, t, state)
    wall_Δt = time() - sink.last_update_wall_time
    if wall_Δt > sink.min_wall_Δt
        config = configuration(state)
        quat = config[1:4]
        pose = config[5:7]
        yaw = extract_yaw_from_quaternion(quat) 

        offset = [sink.follow_dist * [cos(yaw), sin(yaw)]; -sink.follow_height] +
                 sink.follow_offset * [sin(yaw), -cos(yaw), 0]

        setcameratarget!(sink.vis[sink.follow_cam_id].visualizer, pose)
        setcameraposition!(sink.vis[sink.follow_cam_id].visualizer, pose-offset)

        foreach(vis->set_configuration!(vis, configuration(state)), sink.vis)

        sink.last_update_wall_time = time()
    end
    nothing
end

function RigidBodyDynamics.OdeIntegrators.process(sink::PublisherSink, t, state)
    wall_Δt = time() - sink.last_update_wall_time
    if wall_Δt > sink.min_wall_Δt
        put!(sink.channel, VehicleState(state.q, state.v, true))
        sink.last_update_wall_time = time()
    end
    nothing
end

function extract_yaw_from_quaternion(q)
    atan(2(q[1]*q[4]+q[2]*q[3]), 1-2*(q[3]^2+q[4]^2))
end

