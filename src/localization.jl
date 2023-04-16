include("view_car.jl")
include("measurements.jl")
include("map.jl")

using Random
using Distributions

mutable struct CarParticle {
    config::CarConfig
    x_vel::Float64
    y_vel::Float64
    z_vel::Float64
    yaw_vel::Float64
    roll_vel::Float64
    pitch_vel::Float64
    weight::Float64
}


# Updated process model
function rigid_body_dynamics(particle::CarParticle, Δt::Float64)
    x_vel = particle.x_vel
    y_vel = particle.y_vel
    z_vel = particle.z_vel
    yaw_vel = particle.yaw_vel
    roll_vel = particle.roll_vel
    pitch_vel = particle.pitch_vel

    # Compute the rotation vector and its magnitude
    r = [yaw_vel, roll_vel, pitch_vel]
    mag = norm(r)

    # Compute the quaternion representing the rotation over the time step
    sᵣ = cos(mag*Δt / 2.0)
    vᵣ = sin(mag*Δt / 2.0) * r/mag

    # Compute the updated quaternion
    sₙ = particle.config.quaternion[1]
    vₙ = particle.config.quaternion[2:4]

    s = sₙ*sᵣ - vₙ'*vᵣ
    v = sₙ*vᵣ+sᵣ*vₙ+vₙ×vᵣ

    # Update the position, velocity, and angular velocity
    new_position = particle.config.position + Δt * [x_vel, y_vel, z_vel]
    new_quaternion = [s; v]
    new_velocity = [x_vel, y_vel, z_vel]
    new_angular_vel = [yaw_vel, roll_vel, pitch_vel]

    # Create a new CarParticle object with the updated state and weight
    new_particle = CarParticle(particle.config, x_vel, y_vel, z_vel, yaw_vel, roll_vel, pitch_vel, particle.weight)

    # Update the state of the new particle
    new_particle.config.position = new_position
    new_particle.config.quaternion = new_quaternion

    return new_particle
end

function measurement_model(particle::CarParticle)
    # Compute the GPS measurement
    gps_measurement = [particle.config.position[1], particle.config.position[2], particle.config.position[3]]

    # Compute the IMU measurement
    imu_measurement = [particle.yaw_vel, particle.roll_vel, particle.pitch_vel, particle.x_vel, particle.y_vel, particle.z_vel]

    # Combine the GPS and IMU measurements into a single vector
    measurement = vcat(gps_measurement, imu_measurement)

    return measurement
end

function resample(particles)
    weights = [p.weight for p in particles]
    resampled_particles = []
    for i in 1:length(particles)
        idx = sample(weights, Weights(weights))
        new_particle = deepcopy(particles[idx])
        new_particle.weight = 1.0 / length(particles)
        push!(resampled_particles, new_particle)
    end
    return resampled_particles
end


function localization(gps_channel, imu_channel, localization_state_channel)
    # Set up algorithm / initialize variables
    N = 1000 # number of particles
    particles = [CarParticle(config::CarConfig, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0/N) for i in 1:N]

    while true
        # Step 1: Prediction
        Δt = 1.0/30.0 # time step
        u = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # control input (zero for now)

        for i in 1:N
            # Apply the process model to generate a new particle
            new_particle = rigid_body_dynamics(particles[i], Δt)

            # Add some noise to the new particle's position
            new_particle.config.position += 0.5 * randn(3)

            # Save the new particle
            particles[i] = new_particle
        end

        # Step 2: Measurement Update
        fresh_gps_meas = []
        while isready(gps_channel)
            meas = take!(gps_channel)
            push!(fresh_gps_meas, meas)
        end

        for i in 1:N
            particles[i].weight = 1.0
            for j in 1:length(fresh_gps_meas)
                # Calculate the weight based on the measurement model
                expected_meas = measurement_model(particles[i])
                expected_gps_meas = expected_meas[1:3]
                expected_imu_meas = expected_meas[4:end]
                gps_meas = [fresh_gps_meas[j].x, fresh_gps_meas[j].y, fresh_gps_meas[j].z]
                imu_meas = [fresh_gps_meas[j].yaw_vel, fresh_gps_meas[j].roll_vel, fresh_gps_meas[j].pitch_vel, fresh_gps_meas[j].x_vel, fresh_gps_meas[j].y_vel, fresh_gps_meas[j].z_vel]
                gps_prob = pdf(MvNormal(expected_gps_meas, 0.5), gps_meas)
                imu_prob = pdf(MvNormal(expected_imu_meas, 0.1), imu_meas)
                particles[i].weight *= gps_prob * imu_prob
            end
        end

        # Step 3: Resampling
        particles = resample(particles)

        # Compute the estimated state as the weighted mean of the particles
        mu = CarParticle(config::CarConfig, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        for i in 1:N
            mu.config.position += particles[i].config.position * particles[i].weight
            mu.config.orientation += particles[i].config.orientation * particles[i].weight
            mu.config.velocity += particles[i].config.velocity * particles[i].weight
            mu.config.steering_angle += particles[i].config.steering_angle * particles[i].weight
            mu.config.angular_velocity += particles[i].config.angular_velocity * particles[i].weight
            # Return the estimated state
    return mu
end
