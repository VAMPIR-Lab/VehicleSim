abstract type Measurement end

"""
Records lat/long measurements (expressed as first and second coordinates of map frame)
of GPS sensor.
"""
struct GPSMeasurement <: Measurement
    time::Float64
    lat::Float64
    long::Float64
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

function gps(vehicle, state_channel, meas_channel; sqrt_meas_cov = Diagonal([1.0, 1.0]), max_rate=60.0)
    min_Δ = 1.0/max_rate
    t = time()
    T = get_gps_transform()
    gps_loc_body = T*[zeros(3); 1.0]
    while true
        state = fetch(state_channel)
        tnow = time()
        if tnow - t > min_Δ
            t = tnow
            xyz_body = state.q[5:7]
            q_body = state.q[1:4]
            Tbody = get_body_transform(q_body, xyz_body)
            xyz_gps = Tbody * [gps_loc_body; 1]
            meas = xyz_gps[1:2] + sqrt_meas_cov*randn(2)
            gps_meas = GPSMeasurement(t, meas[1], meas[2])
            put!(meas_channel, gps_meas)
        end
    end
end

function imu(vehicle, state_channel, meas_channel; sqrt_meas_cov = Diagonal([0.01, 0.01, 0.01, 0.01, 0.01, 0.01]), max_rate=60.0)
    min_Δ = 1.0/max_rate
    t = time()
    T_body_imu = get_imu_transform()
    T_imu_body = invert_transform(T_imu_body)
    R = T_imu_body[1:3,1:3]
    p = T_imu_body[1:3,end]

    while true
        state = fetch(state_channel)
        tnow = time()
        if tnow - t > min_Δ
            t = tnow
            v_body = state.v[4:6]
            ω_body = state.v[1:3]

            ω_imu = R * ω_body
            v_imu = R * v_body + p × ω_imu

            meas = [v_imu; ω_imu] * sqrt_meas_cov*randn(6)

            imu_meas = IMUMeasurement(t, meas[1:3], meas[4:6])
            put!(meas_channel, imu_meas)
        end
    end
end

function get_3d_bbox_corners(xyz, quat, box_size)
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
        states = [fetch(state_channel) for state_channel in state_channels]
        corners_body = [get_3d_bbox_corners(state, vehicle_size) for state in states]
        tnow = time()
        if tnow - t > min_Δ
            t = tnow
            for i = 1:num_vehicles             
                bboxes = []
                ego_state = states[i]
                T_world_body = get_body_transform(ego_state.q[1:4], ego_state.q[5:7])
                T_world_camrot1 = multiply_transforms(T_world_body, T_body_camrot1)
                T_world_camrot2 = multiply_transforms(T_world_body, T_body_camrot2)
                T_camrot1_world = invert_transform(T_world_camrot1)
                T_camrot2_world = invert_transform(T_world_camrot2)
                for (camera_id, transform) in zip((1,2), (T_camrot1_world, T_camrot2_world))
                    
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
                            else
                            px = focal_len*corner[1]/corner[3]
                            py = focal_len*corner[2]/corner[3]
                            left = min(left, px)
                            right = max(right, px)
                            top = min(top, py)
                            bot = max(bot, py)
                        end
                        if top ≈ bot || left ≈ right
                            # out of frame
                            continue
                        else
                            top = convert_to_pixel(image_height, pixel_len, top)
                            bot = convert_to_pixel(image_height, pixel_len, bot)
                            left = convert_to_pixel(image_width, pixel_len, left)
                            top = convert_to_pixel(image_width, pixel_len, right)
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

function ground_truth(vehicles, state_channels, gt_channels; max_rate=20.0) 
    min_Δ = 1.0/max_rate
    t = time()
    num_vehicles = length(vehicles)
    vehicle_size = SVector(13.2, 5.7, 5.3)
    while true
        states = [fetch(state_channel) for state_channel in state_channels]
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

function measure_vehicles(map,
        vehicles, 
        state_channels, 
        meas_channels, 
        shutdown_channel;
        measure_gps = false,
        measure_imu = false,
        measure_cam = false,
        measure_gt = true
        rng=MersenneTwister(1),
    )
    
    target_segments = identify_loading_segments(map)

    shutdown = false
    @async while true
        if isready(shutdown_channel)
            shutdown = fetch(shutdown_channel)
            shutdown && break
        end
        sleep(0.1)
    end
 
    num_vehicles = length(state_channels)
    @assert num_vehicles == length(meas_channels)

    gps_channels = [Channel{GPSMeasurement}(32) for _ in 1:num_vehicles]
    imu_channels = [Channel{GPSMeasurement}(32) for _ in 1:num_vehicles]
    cam_channels = [Channel{CameraMeasurement}(32) for _ in 1:num_vehicles]
    gt_channels = [Channel{CameraMeasurement}(32) for _ in 1:num_vehicles]
    
    target_channels = [Channel{Int}(1) for _ in 1:num_vehicles]
    # Fixed target segments for now.
    for id in 1:num_vehicles
        put!(target_channels[id], rand(target_segments))
    end

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
                    for (channel, meas_list) in zip(channels, meas_lists)
                        while isready(channel)
                            msg = take!(channel)
                            push!(meas_list, msg)
                        end
                    end
                    if isempty(meas_channels[id])
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
