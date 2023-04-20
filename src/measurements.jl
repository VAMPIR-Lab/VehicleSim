abstract type Measurement end

"""
Records lat/long measurements (expressed as first and second coordinates of map frame)
of GPS sensor.
"""
struct GPSMeasurement <: Measurement
    time::Float64
    lat::Float64
    long::Float64
    heading::Float64
end

"""
Records linear and angular velocity experienced instantaneously by IMU sensor,
in local frame of IMU.
"""
struct IMUMeasurement <: Measurement
    time::Float64
    linear_vel::SVector{3, Float64}
    angular_vel::SVector{3, Float64}
end

"""
Records list of bounding boxes represented by top-left and bottom-right pixel coordinates.
Camera_id is 1 or 2, indicating which camera produced the bounding boxes.

Pixels start from (1,1) in top-left corner of image, and proceed right (first axis) and down (second axis)
to image_width and image_height pixels.

Each pixel corresponds to a pixel_len x pixel_len patch at focal_len away from a pinhole model.
"""
struct CameraMeasurement <: Measurement
    time::Float64
    camera_id::Int
    focal_length::Float64
    pixel_length::Float64
    image_width::Int # pixels
    image_height::Int # pixels
    bounding_boxes::Vector{SVector{4, Int}}
end

"""
Records position, orientation, linear and angular velocity, and size of 3D bounding box of vehicle
specified by vehicle ID
"""
struct GroundTruthMeasurement <: Measurement
    time::Float64
    vehicle_id::Int
    position::SVector{3, Float64} # position of center of vehicle
    orientation::SVector{4, Float64} # represented as quaternion
    velocity::SVector{3, Float64}
    angular_velocity::SVector{3, Float64} # angular velocity around x,y,z axes
    size::SVector{3, Float64} # length, width, height of 3d bounding box centered at (position/orientation)
end

"""
Vehicle ID is the id of the vehicle receiving this message.
target segment is the map segment that should be driven to.
mesaurements are the latest sensor measurements.
"""
struct MeasurementMessage
    vehicle_id::Int
    target_segment::Int
    measurements::Vector{Measurement}
end

function Rot_from_quat(q)
    qw = q[1]
    qx = q[2]
    qy = q[3]
    qz = q[4]

    R = [qw^2+qx^2-qy^2-qz^2 2(qx*qy-qw*qz) 2(qw*qy+qx*qz);
         2(qx*qy+qw*qz) qw^2-qx^2+qy^2-qz^2 2(qy*qz-qw*qx);
         2(qx*qz-qw*qy) 2(qw*qx+qy*qz) qw^2-qx^2-qy^2+qz^2]
end

function J_R_q(q)
    qw = q[1]
    qx = q[2]
    qy = q[3]
    qz = q[4]

    dRdq1 = 2*[qw -qz qy;
             qz qw -qx;
             -qy qx qw]
    dRdq2 = 2*[qx qy qz;
               qy -qx -qw;
               qz qw -qx]
    dRdq3 = 2*[-qy qx qw;
               qx qy qz;
               -qw qz -qy]
    dRdq4 = 2*[-qz -qw qx;
               qw -qz qy;
               qx qy qz]
    (dRdq1, dRdq2, dRdq3, dRdq4)
end

"""
Can be used as process model for EKF
which estimates xₖ = [position; quaternion; velocity; angular_vel]

We haven't discussed quaternions in class much, but interfacing with GPS/IMU
will be much easier using this representation, which is used internally by the simulator.
"""
function rigid_body_dynamics(position, quaternion, velocity, angular_vel, Δt)
    r = angular_vel
    mag = norm(r)

    if mag < 1e-5
        sᵣ = 1.0
        vᵣ = zeros(3)
    else
        sᵣ = cos(mag*Δt / 2.0)
        vᵣ = sin(mag*Δt / 2.0) * (r / mag)
    end

    sₙ = quaternion[1]
    vₙ = quaternion[2:4]

    s = sₙ*sᵣ - vₙ'*vᵣ
    v = sₙ*vᵣ+sᵣ*vₙ+vₙ×vᵣ

    R = Rot_from_quat(quaternion)  

    new_position = position + Δt * R * velocity
    new_quaternion = [s; v]
    new_velocity = velocity
    new_angular_vel = angular_vel
    return [new_position; new_quaternion; new_velocity; new_angular_vel]
end

function f(x, Δt)
    rigid_body_dynamics(x[1:3], x[4:7], x[8:10], x[11:13], Δt)
end

function Jac_x_f(x, Δt)
    J = zeros(13, 13)

    r = x[11:13]
    mag = norm(r)
    if mag < 1e-5
        sᵣ = 1.0
        vᵣ = zeros(3)
    else
        sᵣ = cos(mag*Δt / 2.0)
        vᵣ = sin(mag*Δt / 2.0) * (r / mag)
    end

    sₙ = x[4]
    vₙ = x[5:7]

    s = sₙ*sᵣ - vₙ'*vᵣ
    v = sₙ*vᵣ+sᵣ*vₙ+vₙ×vᵣ

    R = Rot_from_quat([sₙ; vₙ])  
    (J_R_q1, J_R_q2, J_R_q3, J_R_q4) = J_R_q([sₙ; vₙ])
    
    #new_position = position + Δt * R * velocity
    #new_quaternion = [s; v]
    #new_velocity = velocity
    #new_angular_vel = angular_vel
    
    velocity = x[8:10]

    J[1:3, 1:3] = I(3)
    J[1:3, 4] = Δt * J_R_q1*velocity
    J[1:3, 5] = Δt * J_R_q2*velocity
    J[1:3, 6] = Δt * J_R_q3*velocity
    J[1:3, 7] = Δt * J_R_q4*velocity
    J[1:3, 8:10] = Δt * R
    J[4, 4] = sᵣ
    J[4, 5:7] = -vᵣ'
    J[5:7, 4] = vᵣ
    J[5:7, 5:7] = [sᵣ vᵣ[3] -vᵣ[2];
                   -vᵣ[3] sᵣ vᵣ[1];
                   vᵣ[2] -vᵣ[1] sᵣ]
 
    Jsv_srvr = [sₙ -vₙ'
                vₙ [sₙ -vₙ[3] vₙ[2];
                    vₙ[3] sₙ -vₙ[1];
                    -vₙ[2] vₙ[1] sₙ]]
    Jsrvr_mag = [-sin(mag*Δt / 2.0) * Δt / 2; sin(mag*Δt/2.0) * (-r / mag^2) + cos(mag*Δt/2)*Δt/2 * r/mag]
    Jsrvr_r = [zeros(1,3); sin(mag*Δt / 2) / mag * I(3)]
    Jmag_r = 1/mag * r'

    J[4:7, 11:13] = Jsv_srvr * (Jsrvr_mag*Jmag_r + Jsrvr_r)
    J[8:10, 8:10] = I(3)
    J[11:13, 11:13] = I(3)
    J
end

function get_rotated_camera_transform()
    R = [0 0 1.;
         -1 0 0;
         0 -1 0]
    t = zeros(3)
    [R t]
end

function get_cam_transform(camera_id)
    # TODO load this from URDF
    R_cam_to_body = RotY(0.02)
    t_cam_to_body = [1.35, 1.7, 2.4]
    if camera_id == 2
        t_cam_to_body[2] = -1.7
    end

    T = [R_cam_to_body t_cam_to_body]
end

function get_gps_transform()
    # TODO load this from URDF
    R_gps_to_body = one(RotMatrix{3, Float64})
    t_gps_to_body = [-3.0, 1, 2.6]
    T = [R_gps_to_body t_gps_to_body]
end

function get_imu_transform()
    R_imu_to_body = RotY(0.02)
    t_imu_to_body = [0, 0, 0.7]

    T = [R_imu_to_body t_imu_to_body]
end

function get_body_transform(quat, loc)
    R = Rot_from_quat(quat)
    [R loc]
end

function J_Tbody(x)
    J_Tbody_xyz = (zeros(3,4), zeros(3,4), zeros(3,4))
    for i = 1:3
       J_Tbody_xyz[i][i,4] = 1.0
    end
    return (J_Tbody_xyz, [[dR zeros(3)] for dR in J_R_q(x[4:7])])
end

function invert_transform(T)
    R = T[1:3,1:3]
    t = T[1:3,end]
    R\[I(3) -t]
end

"""
T = T1 * T2

i.e. apply T2 then T1
"""
function multiply_transforms(T1, T2)
    T1f = [T1; [0 0 0 1.]] 
    T2f = [T2; [0 0 0 1.]]

    T = T1f * T2f
    T = T[1:3, :]
end

function h_gps(x)
    T = get_gps_transform()
    gps_loc_body = T*[zeros(3); 1.0]
    xyz_body = x[1:3] # position
    q_body = x[4:7] # quaternion
    Tbody = get_body_transform(q_body, xyz_body)
    xyz_gps = Tbody * [gps_loc_body; 1]
    yaw = extract_yaw_from_quaternion(q_body)
    meas = [xyz_gps[1:2]; yaw]
end

function Jac_h_gps(x)
    T = get_gps_transform()
    gps_loc_body = T*[zeros(3); 1.0]
    xyz_body = x[1:3] # position
    q_body = x[4:7] # quaternion
    Tbody = get_body_transform(q_body, xyz_body)
    xyz_gps = Tbody * [gps_loc_body; 1]
    yaw = extract_yaw_from_quaternion(q_body)
    #meas = [xyz_gps[1:2]; yaw]
    J = zeros(3, 13)
    (J_Tbody_xyz, J_Tbody_q) = J_Tbody(x)
    for i = 1:3
        J[1:2,i] = (J_Tbody_xyz[i]*[gps_loc_body; 1])[1:2]
    end
    for i = 1:4
	J[1:2,3+i] = (J_Tbody_q[i]*[gps_loc_body; 1])[1:2]
    end
    w = q_body[1]
    x = q_body[2]
    y = q_body[3]
    z = q_body[4]
    J[3,4] = -(2 * z * (-1 + 2 * (y^2 + z^2)))/(4 * (x * y + w * z)^2 + (1 - 2 * (y^2 + z^2))^2)
    J[3,5] = -(2 * y * (-1 + 2 * (y^2 + z^2)))/(4 * (x * y + w * z)^2 + (1 - 2 * (y^2 + z^2))^2)
    J[3,6] = (2 * (x + 2 * x * y^2 + 4 * w * y * z - 2 * x * z^2))/(1 + 4 * y^4 + 8 * w * x * y * z + 4 * (-1 + w^2) * z^2 + 4 * z^4 + 4 * y^2 * (-1 + x^2 + 2 * z^2))
    J[3,7] = (2 * (w - 2 * w * y^2 + 4 * x * y * z + 2 * w * z^2))/(1 + 4 * y^4 + 8 * w * x * y * z + 4 * (-1 + w^2) * z^2 + 4 * z^4 + 4 * y^2 * (-1 + x^2 + 2 * z^2))
    J
end
 
function gps(vehicle, state_channel, meas_channel; sqrt_meas_cov = Diagonal([1.0, 1.0, 0.1]), max_rate=10.0)
    min_Δ = 1.0/max_rate
    t = time()
    while true
        sleep(0.0001)
        state = fetch(state_channel)
        tnow = time()
        if tnow - t > min_Δ
            t = tnow
	    x = [state.q[5:7]; state.q[1:4]; state.v[4:6]; state.v[1:3]]
	    meas = h_gps(x) + sqrt_meas_cov*randn(3)
            gps_meas = GPSMeasurement(t, meas[1], meas[2], meas[3])
            put!(meas_channel, gps_meas)
        end
    end
end

function imu(vehicle, state_channel, meas_channel; sqrt_meas_cov = Diagonal([0.001, 0.001, 0.001, 0.001, 0.001, 0.001]), max_rate=10.0) # Don't use 
    min_Δ = 1.0/max_rate
    t = time()
    T_body_imu = get_imu_transform()
    T_imu_body = invert_transform(T_body_imu)
    R = T_imu_body[1:3,1:3]
    p = T_imu_body[1:3,end]

    while true
        sleep(0.0001)
        state = fetch(state_channel)
        tnow = time()
        if tnow - t > min_Δ
            t = tnow
            v_body = state.v[4:6] # velocity
            ω_body = state.v[1:3] # angular_vel

            ω_imu = R * ω_body
            v_imu = R * v_body + p × ω_imu

            meas = [v_imu; ω_imu] + sqrt_meas_cov*randn(6)

            imu_meas = IMUMeasurement(t, meas[1:3], meas[4:6])
            put!(meas_channel, imu_meas)
        end
    end
end

function get_3d_bbox_corners(state, box_size)
    quat = state.q[1:4] # quatnerion
    xyz = state.q[5:7] # position
    T = get_body_transform(quat, xyz)
    corners = []
    for dx in [-box_size[1]/2, box_size[1]/2]
        for dy in [-box_size[2]/2, box_size[2]/2]
            for dz in [-box_size[3]/2, box_size[3]/2]
                push!(corners, T*[dx, dy, dz, 1])
            end
        end
    end
    corners
end

function convert_to_pixel(num_pixels, pixel_len, px)
    min_val = -pixel_len*num_pixels/2
    pix_id = cld(px - min_val, pixel_len)+1 |> Int
    return pix_id
end

function cameras(vehicles, state_channels, cam_channels; max_rate=10.0, focal_len = 0.01, pixel_len = 0.001, image_width = 640, image_height = 480)
    min_Δ = 1.0/max_rate
    t = time()
    num_vehicles = length(vehicles)
    vehicle_size = SVector(13.2, 5.7, 5.3)

    T_body_cam1 = get_cam_transform(1)
    T_body_cam2 = get_cam_transform(2)
    T_cam_camrot = get_rotated_camera_transform()

    T_body_camrot1 = multiply_transforms(T_body_cam1, T_cam_camrot)
    T_body_camrot2 = multiply_transforms(T_body_cam2, T_cam_camrot)

    while true
        sleep(0.0001)
        states = [fetch(state_channels[id]) for id in 1:num_vehicles]
        corners_body = [get_3d_bbox_corners(state, vehicle_size) for state in states]
        tnow = time()
        if tnow - t > min_Δ
            t = tnow
            for i = 1:num_vehicles             
                ego_state = states[i]
                T_world_body = get_body_transform(ego_state.q[1:4], ego_state.q[5:7])
                T_world_camrot1 = multiply_transforms(T_world_body, T_body_camrot1)
                T_world_camrot2 = multiply_transforms(T_world_body, T_body_camrot2)
                T_camrot1_world = invert_transform(T_world_camrot1)
                T_camrot2_world = invert_transform(T_world_camrot2)
                for (camera_id, transform) in zip((1,2), (T_camrot1_world, T_camrot2_world))
                    bboxes = []
                    
                    for j = 1:num_vehicles
                        j == i && continue
                        other_vehicle_corners = [transform * [pt;1] for pt in corners_body[j]]
                        visible = false

                        left = image_width/2
                        right = -image_width/2
                        top = image_height/2
                        bot = -image_height/2

                        for corner in other_vehicle_corners
                            if corner[3] < focal_len
                                break
                            end
                            px = focal_len*corner[1]/corner[3]
                            py = focal_len*corner[2]/corner[3]
                            left = min(left, px)
                            right = max(right, px)
                            top = min(top, py)
                            bot = max(bot, py)
                        end
                        if top ≈ bot || left ≈ right || top > bot || left > right
                            # out of frame
                            continue
                        else 
                            top = convert_to_pixel(image_height, pixel_len, top)
                            bot = convert_to_pixel(image_height, pixel_len, bot)
                            left = convert_to_pixel(image_width, pixel_len, left)
                            right = convert_to_pixel(image_width, pixel_len, right)
                            push!(bboxes, SVector(top, left, bot, right))
                        end
                    end
                
                    meas = CameraMeasurement(t, camera_id, focal_len, pixel_len, image_width, image_height, bboxes)
                    put!(cam_channels[i], meas)
                end
            end
        end
    end
end

function ground_truth(vehicles, state_channels, gt_channels; max_rate=10.0) 
    min_Δ = 1.0/max_rate
    t = time()
    num_vehicles = length(vehicles)
    vehicle_size = SVector(13.2, 5.7, 5.3)
    while true
        sleep(0.0001)
        states = [fetch(state_channels[i]) for i = 1:num_vehicles]
        tnow = time()
        if tnow - t > min_Δ
            t = tnow
            measurements = [GroundTruthMeasurement(t, i, 
                                                   states[i].q[5:7], 
                                                   states[i].q[1:4], 
                                                   states[i].v[4:6], 
                                                   states[i].v[1:3],
                                                   vehicle_size) for i = 1:num_vehicles]
            for i = 1:num_vehicles
                foreach(m->put!(gt_channels[i], m), measurements)
            end
        end
    end
end

function update_targets(target_channels, state_channels, target_segments, map)
    scores = zeros(Int, length(target_channels))
    while true
        sleep(0.001)
        for i = 1:length(target_channels)
            current_target = fetch(target_channels[i])
            state = fetch(state_channels[i])
            pos = state.q[5:6]
            vel = state.v[4]
            if reached_target(pos, vel, map[current_target])
                scores[i] += 1
                new_target = rand(setdiff(target_segments, current_target))
		@info "Vehicle $i reached target! New target is $new_target"
		for i = 1:length(target_channels)
		    @info "Scores: team $i has $(scores[i]) successful pickup/dropoffs"
		end
                take!(target_channels[i])
		put!(target_channels[i], new_target)
            end
        end
    end
end

function measure_vehicles(map,
        vehicles, 
        state_channels, 
        meas_channels, 
        shutdown_channel;
        measure_gps = true,
        measure_imu = true,
        measure_cam = true,
        measure_gt = true,
        rng=MersenneTwister(1)
    )
    
    target_segments = identify_loading_segments(map)

 
    num_vehicles = length(state_channels)
    @assert num_vehicles == length(meas_channels)

    gps_channels = [Channel{GPSMeasurement}(32) for _ in 1:num_vehicles]
    imu_channels = [Channel{IMUMeasurement}(32) for _ in 1:num_vehicles]
    cam_channels = [Channel{CameraMeasurement}(32) for _ in 1:num_vehicles]
    gt_channels = [Channel{GroundTruthMeasurement}(32) for _ in 1:num_vehicles]
    
    shutdown = false
    @async while true
        if isready(shutdown_channel)
            shutdown = fetch(shutdown_channel)
            if shutdown
                foreach(c->close(c), [gps_channels; imu_channels; cam_channels; gt_channels])
                foreach(c->close(c), values(meas_channels))
                foreach(c->close(c), values(state_channels))
                break
            end
        end
        sleep(0.1)
    end
    
    target_channels = [Channel{Int}(1) for _ in 1:num_vehicles]
    for id in 1:num_vehicles
	target = rand(target_segments)
	@info "Target for vehicle $id: $target"
        put!(target_channels[id], target)
    end

    errormonitor(@async update_targets(target_channels, state_channels, target_segments, map))

    # Centralized Measurements
    measure_gt && @async ground_truth(vehicles, state_channels, gt_channels)
    measure_cam && @async cameras(vehicles, state_channels, cam_channels)

    for id in 1:num_vehicles
        # Intrinsic Measurements
        measure_gps && @async gps(vehicles[id], state_channels[id], gps_channels[id])
        measure_imu && @async imu(vehicles[id], state_channels[id], imu_channels[id])
 
        let id=id
            @async begin
                gps_measurements = GPSMeasurement[]
                imu_measurements = IMUMeasurement[]
                cam_measurements = CameraMeasurement[]
                gt_measurements = GroundTruthMeasurement[]
                meas_lists = (gps_measurements, imu_measurements, cam_measurements, gt_measurements)
                channels = (gps_channels[id], imu_channels[id], cam_channels[id], gt_channels[id])
                while true
                    sleep(0.0001)
                    for (channel, meas_list) in zip(channels, meas_lists)
                        while isready(channel)
                            msg = take!(channel)
                            push!(meas_list, msg)
                        end
                    end
                    if isempty(meas_channels[id]) && !all(isempty.(meas_lists))
                        target = fetch(target_channels[id])
                        msg = MeasurementMessage(id, target, vcat(meas_lists...))
                        put!(meas_channels[id], msg)
                        for meas_list in meas_lists
                            deleteat!(meas_list, 1:length(meas_list))
                        end
                    end
                    for meas_list in meas_lists
                        L = length(meas_list)
                        if L > 20
                            deleteat!(meas_list, 1:L)
                        end
                    end 
                end
            end
        end
    end
end
