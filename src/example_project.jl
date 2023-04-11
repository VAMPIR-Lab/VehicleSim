struct MyLocalizationType
    field1::SVector{2,Float64} # Lat and Long
    field2::SVector{2,Float64} # Linear and angular velocity
    field3::Float64 #time stamp
end

struct MyPerceptionType
    time::Float64
    vehicle_id::Int
    position::SVector{3,Float64} # position of center of vehicle
    orientation::SVector{4,Float64} # represented as quaternion
    velocity::SVector{3,Float64}
    # angular_velocity::SVector{3, Float64} # angular velocity around x,y,z axes
    size::SVector{3,Float64} # length, width, height of 3d bounding box centered at (position/orientation)
end


function test_algorithms(gt_channel,
    localization_state_channel,
    perception_state_channel,
    ego_vehicle_id)
    estimated_vehicle_states = Dict{Int,Tuple{Float64,Union{SimpleVehicleState,FullVehicleState}}}
    gt_vehicle_states = Dict{Int,GroundTruthMeasuremen}

    t = time()
    while true

        while isready(gt_channel)
            meas = take!(gt_channel)
            id = meas.vehicle_id
            if meas.time > gt_vehicle_states[id].time
                gt_vehicle_states[id] = meas
            end
        end

        # Test Localization
        latest_estimated_ego_state = fetch(localization_state_channel)
        latest_true_ego_state = gt_vehicle_states[ego_vehicle_id]
        if latest_estimated_ego_state.last_update < latest_true_ego_state.time - 0.5
            @warn "Localization algorithm stale."
        else
            estimated_xyz = latest_estimated_ego_state.position
            true_xyz = latest_true_ego_state.position
            position_error = norm(estimated_xyz - true_xyz)
            t2 = time()
            if t2 - t > 5.0
                @info "Localization position error: $position_error"
                t = t2
            end
        end

        # Test Perception
        latest_perception_state = fetch(perception_state_channel)
        last_perception_update = latest_perception_state.last_update
        vehicles = last_perception_state.x

        for vehicle in vehicles
            xy_position = [vehicle.p1, vehicle.p2]
            closest_id = 0
            closest_dist = Inf
            for (id, gt_vehicle) in gt_vehicle_states
                if id == ego_vehicle_id
                    continue
                else
                    gt_xy_position = gt_vehicle_position[1:2]
                    dist = norm(gt_xy_position - xy_position)
                    if dist < closest_dist
                        closest_id = id
                        closest_dist = dist
                    end
                end
            end
            paired_gt_vehicle = gt_vehicle_states[closest_id]

            # compare estimated to GT

            if last_perception_update < paired_gt_vehicle.time - 0.5
                @info "Perception upate stale"
            else
                # compare estimated to true size
                estimated_size = [vehicle.l, vehicle.w, vehicle.h]
                actual_size = paired_gt_vehicle.size
                @info "Estimated size error: $(norm(actual_size-estimated_size))"
            end
        end
    end
end


function localize(gps_channel, imu_channel, localization_state_channel, gt_channel)
    # Set up algorithm / initialize variables
    for n in 1:100
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

        localization_state = MyLocalizationType([0,0], [0,0], 0)
        # if isready(gt_channel)
            # @info "test"
            # @info take!(gt_channel)
        # end

        if isready(localization_state_channel)
            take!(localization_state_channel)
        end
        put!(localization_state_channel, localization_state)
    end
end

function perception(cam_meas_channel, localization_state_channel, perception_state_channel)
    # set up stuff

    # struct CameraMeasurement <: Measurement
    #     time::Float64
    #     camera_id::Int
    #     focal_length::Float64
    #     pixel_length::Float64
    #     image_width::Int # pixels
    #     image_height::Int # pixels
    #     bounding_boxes::Vector{SVector{4, Int}}
    # end

    while true

        """
            1. Get Camera measurements and ego car's localization
        """
        fresh_cam_meas = []
        while isready(cam_meas_channel)
            meas = take!(cam_meas_channel)
            push!(fresh_cam_meas, meas)
        end
        latest_localization_state = fetch(localization_state_channel)

        """
            2. Determine where the camera is looking at
        """
        # process bounding boxes / run ekf / do what you think is good


        """
            3. Run ekf
        """
        # NOTE: ONE EKF PER VEHICLE SEEN

        """
            4. Output the new perception state
        """
        perception_state = MyPerceptionType(0, 0.0)
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

function isfull(ch::Channel)
    length(ch.data) ≥ ch.sz_max
end


function my_client(host::IPAddr=IPv4(0), port=4444)
    socket = Sockets.connect(host, port)
    map_segments = training_map()
    msg = deserialize(socket) # Visualization info
    @info msg

    gps_channel = Channel{GPSMeasurement}(32)
    imu_channel = Channel{IMUMeasurement}(32)
    cam_channel = Channel{CameraMeasurement}(32)
    gt_channel = Channel{GroundTruthMeasurement}(32)

    localization_state_channel = Channel{MyLocalizationType}(1)
    perception_state_channel = Channel{MyPerceptionType}(1)

    target_map_segment = 0 # (not a valid segment, will be overwritten by message)
    ego_vehicle_id = 0 # (not a valid id, will be overwritten by message. This is used for discerning ground-truth messages)

    @async for n in 1:100
        measurement_msg = deserialize(socket)
        target_map_segment = measurement_msg.target_segment
        ego_vehicle_id = measurement_msg.vehicle_id
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

    @async localize(gps_channel, imu_channel, localization_state_channel, gt_channel) #FIXME: Remove gt channel once ready
    # @async perception(cam_channel, localization_state_channel, perception_state_channel)
    # @async decision_making(localization_state_channel, perception_state_channel, map, socket)
    # @async test_algorithms(gt_channel, localization_state_channel, perception_state_channel, ego_vehicle_id)
end
