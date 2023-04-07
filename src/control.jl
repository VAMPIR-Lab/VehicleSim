function wheel_control!(bodyid_to_wrench, chevy, t, state::MechanismState;
        reference_velocity=0.0, k₁=-1000.0)
    forward_velocity = state.v[4]
    drive_force = k₁ * (forward_velocity - reference_velocity)

    for i = 7:8
        bodyid = BodyID(i)
        wheel = bodies(chevy)[i]
        frame = wheel.frame_definitions[1].from
        body_to_root = transform_to_root(state, bodyid, false)
        wrench = Wrench{Float64}(frame, [0.0,0,0], [drive_force,0,0])
        bodyid_to_wrench[BodyID(i)] = transform(wrench, body_to_root)
    end
    nothing
end

function steering_control!(torques::AbstractVector, t, state::MechanismState;
        reference_angle=0.0, k₁=-6500.0, k₂=-2500.0)
    js = joints(state.mechanism)
    linkage_left = js[6]
    linkage_right = js[7]

    actual = configuration(state, linkage_left)

    torques[velocity_range(state, linkage_left)] .= k₁ * (configuration(state, linkage_left) .- reference_angle) + k₂ * velocity(state, linkage_left)
    torques[velocity_range(state, linkage_right)] .= k₁ * (configuration(state, linkage_right) .- reference_angle) + k₂ * velocity(state, linkage_right)
end

function suspension_control!(torques::AbstractVector, t, state::MechanismState; k₁=-6500.0, k₂=-2500.0, k₃ = -30000.0, k₄=-15000.0)
    js = joints(state.mechanism)
    front_axle_mount = js[2]
    rear_axle_mount = js[3]
    front_axle_roll = js[4]
    rear_axle_roll = js[5]

    torques[velocity_range(state, front_axle_mount)] .= k₁ * configuration(state, front_axle_mount) + k₂ * velocity(state, front_axle_mount)
    torques[velocity_range(state, rear_axle_mount)] .= k₁ * configuration(state, rear_axle_mount) + k₂ * velocity(state, rear_axle_mount)
    torques[velocity_range(state, front_axle_roll)] .= k₃ * configuration(state, front_axle_roll) + k₄ * velocity(state, front_axle_roll)
    torques[velocity_range(state, rear_axle_roll)] .= k₃ * configuration(state, rear_axle_roll) + k₄ * velocity(state, rear_axle_roll)
end
