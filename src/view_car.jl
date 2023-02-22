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
    
    fxz = fam.joint_to_predecessor.x.mat[[1,3],4] .+ [0, config.front_travel]
    rxz = ram.joint_to_predecessor.x.mat[[1,3],4] .+ [0, config.rear_travel]
    dxz = fxz-rxz
    pitch = atan(dxz[2], dxz[1])
    
    q1 = [cos(config.roll_angle/2); sin(config.roll_angle/2)*axis_roll]
    q2 = [cos(pitch/2); sin(pitch/2)*axis_pitch]
    s1 = q1[1]
    s2 = q2[1]
    v1 = q1[2:4]
    v2 = q2[2:4]
    s = s1*s2 - v1'*v2
    v = s1*v2+s2*v1+v1×v2
    q = [s; v]

    set_configuration!(state, wb, [q; zeros(3)])
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
    chevy = parse_urdf(urdf_path)
    all_joints = joints(chevy)
    state = MechanismState(chevy)
    mvis = MechanismVisualizer(chevy, URDFVisuals(urdf_path, package_path=[dirname(pathof(VehicleSim))]), vis)
   
    roll_angle = 0.04
    front_travel = 0.1
    rear_travel = -0.3
    steering_angle = 0.1

    configure_car!(mvis, state, all_joints, roll_angle, front_travel, rear_travel, steering_angle)
end

struct CarConfig
    roll_angle::Float64
    front_travel::Float64
    rear_travel::Float64
    steering_angle::Float64
end

function spawn_car(vis, sock)
    urdf_path = joinpath(dirname(pathof(VehicleSim)), "assets", "chevy.urdf")
    chevy = parse_urdf(urdf_path)
    all_joints = joints(chevy)
    state = MechanismState(chevy)
    mvis = MechanismVisualizer(chevy, URDFVisuals(urdf_path, package_path=[dirname(pathof(VehicleSim))]), vis)

    println("car_spawned")

    while isopen(sock)
        car_config = Serialization.deserialize(sock)
        configure_car!(mvis, state, all_joints, car_config)
        println("updated config")
    end
end
        

function car_sim_server(vis, host::IPAddr = IPv4(0), port=3000)
    errormonitor(@async begin
        server = listen(host, port)
        while true
            sock = accept(server)
            println("Client accepted!", sock)
            @async spawn_car(vis, sock)

            #@async while isopen(sock)
            #    write(sock, readline(sock, keep=true))
            #end
        end
    end)
end
     


#function sim_car(vis)
#    urdf_path = joinpath(dirname(pathof(VehicleSim)), "assets", "chevy.urdf")
#    chevy = parse_urdf(urdf_path)
#    state = MechanismState(chevy)
#    body_to_world = first(joints(chevy))
#    mvis = MechanismVisualizer(chevy, URDFVisuals(urdf_path, package_path=[dirname(pathof(VehicleSim))]), vis)
#    @infiltrate
#    for t = 1:100
#        θ = (t-1)*2*pi/100
#        rot = RotXYZ(θ, 0, θ)
#        quat = convert(UnitQuaternion{Float64}, rot)
#        set_configuration!(state, body_to_world, [quat.w, quat.x, quat.y, quat.z, t, 0, 0])
#        set_configuration!(mvis, configuration(state))
#        sleep(0.01)
#    end
#end



