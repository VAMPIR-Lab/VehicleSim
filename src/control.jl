struct VehicleCommand
    steering_angle::Float64
    forward_force::Float64 # Rename to target_velocity or similar
    persist::Bool
    shutdown::Bool
end

struct EndOfSimException <: Exception end
struct EndOfVehicleException <: Exception end


function get_c()
    ret = ccall(:jl_tty_set_mode, Int32, (Ptr{Cvoid},Int32), stdin.handle, true)
    ret == 0 || error("unable to switch to raw mode")
    c = read(stdin, Char)
    ccall(:jl_tty_set_mode, Int32, (Ptr{Cvoid},Int32), stdin.handle, false)
    c
end

function keyboard_controller(host::IPAddr=IPv4(0), port=4444; f_step = 2.0, s_step = 0.1)

    socket = Sockets.connect(host, port)

    forward_force = 0.0
    steering_angle = 0.0
    persist = true
    shutdown = false
    @info "Press 'q' at any time to terminate vehicle. Press 's' to shutdown simulator server."
    while persist && !shutdown && isopen(socket)
        key = get_c()
        if key == 'q'
            # terminate vehicle
            persist = false
        elseif key == 's'
            # shutdown server
            shutdown = true
        elseif key == 'i'
            # increase forward force
            forward_force += f_step
            @info "Target velocity: $forward_force"
        elseif key == 'k'
            # decrease forward force
            forward_force -= f_step
            @info "Target velocity: $forward_force"
        elseif key == 'j'
            # increase steering angle
            steering_angle += s_step
        elseif key == 'l'
            # decrease steering angle
            steering_angle -= s_step
        end

        cmd = VehicleCommand(steering_angle, forward_force, persist, shutdown)
        
        Serialization.serialize(socket, cmd)
    end
    #close(socket) # this should be closed by sim process
end

function delete_vehicle(vis, vehicle)
    name = vehicle.graph.vertices[2].name
    path = "/meshcat/world/$name"
    delete!(vis[path])
    open(vis)
    setcameratarget!(vis, [0,0,0])
    setcameraposition!(vis, [0, -3, 1])

    nothing
end

function spawn_car(vis, sock, chevy_base, chevy_visuals, chevy_joints, vehicle_id, server)
    chevy = deepcopy(chevy_base)
    chevy.graph.vertices[2].name="chevy_$vehicle_id"
    configure_contact_points!(chevy)
    state = MechanismState(chevy)
    mvis = MechanismVisualizer(chevy, chevy_visuals, vis)

    config = CarConfig(SVector(0.0,0,2.5), 0.0, 0.0, 0.0, 0.0)
    configure_car!(mvis, state, joints(chevy), config)

    v = 0.0
    θ = 0.0
    persist = true
    shutdown = false
    set_reference! = (cmd::VehicleCommand) -> begin
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

    @async while isopen(sock)
        car_cmd = Serialization.deserialize(sock)
        set_reference!(car_cmd)
    end 

    try 
        vehicle_simulate(state, 
                         mvis, 
                         Inf, 
                         control!, 
                         wrenches!; 
                         max_realtime_rate=1.0)
    catch e
        delete_vehicle(vis, chevy)
        if e isa EndOfSimException
            close(server)
        end
    end
end

function wheel_control!(bodyid_to_wrench, chevy, t, state::MechanismState;
        reference_velocity=0.0, k₁=-1000.0)
    q = (; w = state.q[1], x = state.q[2], y = state.q[3], z = state.q[4])
    yaw = atan(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)
    forward = [cos(yaw), sin(yaw), 0]
    forward_velocity = state.v[4:6]'*forward
    #total_velocity = norm(state.v[4:6])
    #@info "forward vel: $forward_velocity, total_vel: $total_velocity"
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
    #@info "Actual angle: $actual, reference angle: $reference_angle"

    torques[velocity_range(state, linkage_left)] .= k₁ * (configuration(state, linkage_left) .- reference_angle) + k₂ * velocity(state, linkage_left)
    torques[velocity_range(state, linkage_right)] .= k₁ * (configuration(state, linkage_right) .- reference_angle) + k₂ * velocity(state, linkage_right)
end

function suspension_control!(torques::AbstractVector, t, state::MechanismState; k₁=-6500.0, k₂=-2500.0, k₃ = -25000.0, k₄=-10000.0)
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


