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
