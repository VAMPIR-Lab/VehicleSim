struct SimpleVehicleState
    p1::Float64
    p2::Float64
    θ::Float64
    v::Float64
    l::Float64
    w::Float64
    h::Float64
end

struct FullVehicleState
    position::SVector{3, Float64}
    velocity::SVector{3, Float64}
    orientation::SVector{3, Float64}
    angular_vel::SVector{3, Float64}
end

struct MyLocalizationType
    last_update::Float64
    x::FullVehicleState
end

struct MyPerceptionType
    last_update::Float64
    x::Vector{SimpleVehicleState}
end

# camera stream is continuous, but localization is one time
# so you have to extrapolate the localization state to the 
# camera time that you want to process for
# localization is whatever it is

# channels are conveyer belts, coming in sorted
# but localization channels are just one at a time 

# Take everything off the camera channel, put in a box, 
# Sort them by time, if there are any measurements from before
# last measurement, throw that away because we're past that
# We take all of them in the box, see localization state, extend forward
# or backward, and then do EKF
# Might need to adjust EKF to only do it on most recent box

# WHat happens when thing disappears?

function localize(gps_channel, imu_channel, localization_state_channel)
    # Set up algorithm / initialize variables
    while true
        fresh_gps_meas = []
        while isready(gps_channel)
            meas = take!(gps_channel)
            push!(fresh_gps_meas, meas)
        end
        fresh_imu_meas = []
        while isready(imu_channel)
            meas = take!(imu_channel)
            push!(fresh_imu_meas, meas)
        end
        
        # process measurements

        localization_state = MyLocalizationType(0,0.0)
        if isready(localization_state_channel)
            take!(localization_state_channel)
        end
        put!(localization_state_channel, localization_state)
    end 
end


function f(x, u, ω, Δ)
    [
        x[1] + Δ * cos(
    ]

"""
Measurement func -> map real state to measurement

Camera projection function? Yes
"""
function h(x)

end

"""
The process model 
P(x | x₋₁, u)

Do we actually know the u's?

The measurement model


"""
function perception(cam_meas_channel, localization_state_channel, perception_state_channel)
    # set up stuff
    while true
        fresh_cam_meas = []
        while isready(cam_meas_channel)
            meas = take!(cam_meas_channel)
            push!(fresh_cam_meas, meas)
        end

        latest_localization_state = fetch(localization_state_channel)

        # TODO: Need to understand how the camera transformation works
        #       Ah, this is using in h functions

        x = latest_localization_state.x

        # NO u OR m!
        # uₖ = u_constant
        # mₖ = uₖ + sqrt(proc_cov) * randn(rng, 2) # Noisy IMU measurement.
        Δ = meas_freq + meas_jitter * (2*rand(rng) - 1)
        ω_true = sqrt(dist_cov) * randn(rng, 2)
        xₖ = f(x_prev, uₖ, ω_true, Δ)
        x_prev = xₖ
        # u_prev = uₖ
        zₖ = h(xₖ) + sqrt(meas_var) * randn(rng, 2)
        
        # TODO: iterate through the bounding boxes -- do we do an update for each 
        #       bounding boxes? 

        # process bounding boxes / run ekf / do what you think is good

        # TODO: Define f, μ, m, and calc jacobians

        # TODO: Is F rigid body dynamics, or function from lecture?

        # A = jac_fx(μ, mₖ, zeros(2), Δ)
        # B = jac_fu(μ, mₖ, zeros(2), Δ)
        # L = jac_fω(μ, mₖ, zeros(2), Δ) 

        # μ̂ = f(μ, mₖ, zeros(2), Δ)
        # Σ̂ = A*Σ*A' + B*proc_cov*B' + L*dist_cov*L'
        # 
        # C = jac_hx(μ̂)
        # d = h(μ̂) - C*μ̂

        # Σ = inv(inv(Σ̂) + C'*inv(meas_var)*C)
        # μ = Σ * (inv(Σ̂) * μ̂ + C'*inv(meas_var) * (zₖ - d))

        # TODO: How do we reconcile bounding boxes that reference the same 
        #       car from two cameras? 
        #       When we push the vehicle states, do we just see if there are  
        #       vehicles that seem to have the same dimensions and position?
        #       Or do we figure out which bounding boxes refer to the same car
        #       at the beginning and then include them in the same state?

        # TODO: How do we track when cars enter and leave

        perception_state = MyPerceptionType(0,0.0)
        if isready(perception_state_channel)
            take!(perception_state_channel)
        end
        put!(perception_state_channel, perception_state)
    end
end

function decision_making(localization_state_channel, 
        perception_state_channel, 
        map, 
        target_road_segment_id, 
        socket)
    # do some setup
    while true
        latest_localization_state = fetch(localization_state_channel)
        latest_perception_state = fetch(perception_state_channel)

        # figure out what to do ... setup motion planning problem etc
        steering_angle = 0.0
        target_vel = 0.0
        cmd = VehicleCommand(steering_angle, target_vel, true)
        serialize(socket, cmd)
    end
end

function isfull(ch:Channel)
    length(ch.data) ≥ ch.sz_max
end


function my_client(host::IPAddr=IPv4(0), port=4444)
    socket = Sockets.connect(host, port)
    map_segments = training_map()

    gps_channel = Channel{GPSMeasurement}(32)
    imu_channel = Channel{IMUMeasurement}(32)
    cam_channel = Channel{CameraMeasurement}(32)
    gt_channel = Channel{GroundTruthMeasurement}(32)

    localization_state_channel = Channel{MyLocalizationType}(1)
    perception_state_channel = Channel{MyPerceptionType}(1)

    target_map_segment = 0 # (not a valid segment, will be overwritten by message)
    ego_vehicle_id = 0 # (not a valid id, will be overwritten by message. This is used for discerning ground-truth messages)

    @async while true
        measurement_msg = deserialize(socket)
        target_map_segment = meas.target_segment
        ego_vehicle_id = meas.vehicle_id
        for meas in measurement_msg.measurements
            if meas isa GPSMeasurement
                !isfull(gps_channel) && put!(gps_channel, meas)
            elseif meas isa IMUMeasurement
                !isfull(imu_channel) && put!(imu_channel, meas)
            elseif meas isa CameraMeasurement
                !isfull(cam_channel) && put!(cam_channel, meas)
            elseif meas isa GroundTruthMeasurement
                !isfull(gt_channel) && put!(gt_channel, meas)
            end
        end
    end

    @async localize(gps_channel, imu_channel, localization_state_channel)
    @async perception(cam_channel, localization_state_channel, perception_state_channel)
    @async decision_making(localization_state_channel, perception_state_channel, map, socket)
end
