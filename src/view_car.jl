function get_vis(host::IPAddr = ip"127.0.0.1", default_port=8700)
    vis = Visualizer(MeshCat.CoreVisualizer(host, default_port), ["meshcat"])
    open(vis)
    return vis
end

function configure_car!(mvis, state, joints, config)
    wb = joints[1]
    fam = joints[2]
    ram = joints[3]
    fa = joints[4]
    ra = joints[5]
    lsl = joints[6]
    rsl = joints[7]
    
    axis_roll = [1, 0, 0]
    axis_pitch = [0, 1, 0]
    axis_yaw = [0, 0, 1]
    
    fxz = fam.joint_to_predecessor.x.mat[[1,3],4] .+ [0, config.front_travel]
    rxz = ram.joint_to_predecessor.x.mat[[1,3],4] .+ [0, config.rear_travel]
    dxz = fxz-rxz
    pitch = atan(dxz[2], dxz[1])
    
    q1 = [cos(config.roll_angle/2); sin(config.roll_angle/2)*axis_roll]
    q2 = [cos(pitch/2); sin(pitch/2)*axis_pitch]
    q3 = [cos(config.yaw_angle/2); sin(config.yaw_angle/2)*axis_yaw]
    s1 = q1[1]
    s2 = q2[1]
    s3 = q3[1]
    v1 = q1[2:4]
    v2 = q2[2:4]
    v3 = q3[2:4]
    s = s1*s2 - v1'*v2
    v = s1*v2+s2*v1+v1×v2
    q = [s; v]
    s12 = q[1]
    v12 = q[2:4]
    s = s12*s3 - v12'*v3
    v = s12*v3+s3*v12+v12×v3
    q = [s; v]

    set_configuration!(state, wb, [q; config.position])
    set_configuration!(state, fa, -config.roll_angle)
    set_configuration!(state, ra, -config.roll_angle)
    set_configuration!(state, fam, config.front_travel)
    set_configuration!(state, ram, config.rear_travel)
    set_configuration!(state, lsl, config.steering_angle)
    set_configuration!(state, rsl, config.steering_angle)
    set_configuration!(mvis, configuration(state))
end


function view_car(vis)
    delete!(vis)
    delete!(vis["/Grid"])
    delete!(vis["/Axes"])
    urdf_path = joinpath(dirname(pathof(VehicleSim)), "assets", "chevy.urdf")
    chevy = parse_urdf(urdf_path, floating=true)

    state = MechanismState(chevy)
    
    chevy_visuals = URDFVisuals(urdf_path, package_path=[dirname(pathof(VehicleSim))])
    
    mvis = MechanismVisualizer(chevy, chevy_visuals, vis)

    all_joints = joints(chevy)

    @infiltrate

    config = CarConfig(SVector(10.0,20,2.6), 0.04, 0.0, -0.2, 0.1, 0.1)
    configure_car!(mvis, state, all_joints, config)
end

struct CarConfig
    position::SVector{3, Float64}
    roll_angle::Float64
    yaw_angle::Float64
    front_travel::Float64
    rear_travel::Float64
    steering_angle::Float64
end

function spawn_car(vis, sock, chevy_base, chevy_visuals, chevy_joints, vehicle_id)
    chevy = deepcopy(chevy_base)
    chevy.graph.vertices[2].name="chevy_$vehicle_id"
    state = MechanismState(chevy)
    mvis = MechanismVisualizer(chevy, chevy_visuals, vis)

    println("car_spawned")

    function drive_control!(torques::AbstractVector, t, x::MechanismState)
        0    
    end


    while isopen(sock)
        car_config = Serialization.deserialize(sock)
        configure_car!(mvis, state, chevy_joints, car_config)
        println("updated config")
    end
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


