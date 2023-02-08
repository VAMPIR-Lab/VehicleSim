function view_car(vis)
    urdf_path = joinpath(dirname(pathof(VehicleSim)), "assets", "chevy.urdf")
    chevy = parse_urdf(urdf_path)
    body_to_world = first(joints(chevy))
    state = MechanismState(chevy)
    set_configuration!(state, body_to_world, [0,0,0,0,0,0,1])
    mvis = MechanismVisualizer(chevy, URDFVisuals(urdf_path, package_path=[dirname(pathof(VehicleSim))]), vis)
end


function sim_car(vis)
    urdf_path = joinpath(dirname(pathof(VehicleSim)), "assets", "chevy.urdf")
    chevy = parse_urdf(urdf_path)
    state = MechanismState(chevy)
    body_to_world = first(joints(chevy))
    mvis = MechanismVisualizer(chevy, URDFVisuals(urdf_path, package_path=[dirname(pathof(VehicleSim))]), vis)
    @infiltrate
    for t = 1:100
        θ = (t-1)*2*pi/100
        rot = RotXYZ(θ, 0, θ)
        quat = convert(UnitQuaternion{Float64}, rot)
        set_configuration!(state, body_to_world, [quat.w, quat.x, quat.y, quat.z, t, 0, 0])
        set_configuration!(mvis, configuration(state))
        sleep(0.01)
    end
end



