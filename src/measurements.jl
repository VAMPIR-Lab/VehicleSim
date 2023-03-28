abstract type Measurement end

struct GPSMeasurement <: Measurement
    time::DateTime
    lat::Float64
    long::Float64
end

struct IMUMeasurement <: Measurement
    time::DateTime
    linear_accel::SVector{3, Float64}
    angular_vel::SVector{3, Float64}
end

struct CameraMeasurement <: Measurement
    time::DateTime
    camera_id::Int
    focal_length::Float64
    image_width::Int # pixels
    image_height::Int # pixels
    bounding_boxes::Vector{SVector{4, Int}}
end

struct GroundTruthMeasurement <: Measurement
    q::Vector{Float64}
    v::Vector{Float64}
end

struct MeasurementMessage
    measurements::Vector{Measurement}
end

function gps(vehicle, state_channel, meas_channel; meas_cov = Diagonal(1.0, 1.0))
    while true
        state = fetch(state_channel)
        # do something
    end
end

function measure_vehicles(vehicles, 
        state_channels, 
        meas_channels, 
        shutdown_channel;
        measure_gps = true,
        measure_imu = true,
        measure_cam = true,
        measure_gt = false
    )

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
                        msg = MeasurementMessage(vcat(meas_lists...))
                        put!(meas_channels[id], msg)
                        for meas_list in meas_lists
                            deleteat!(meas_list, 1:length(meas_list))
                        end
                    end
                end
            end
        end
    end
end
