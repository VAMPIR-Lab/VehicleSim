include("view_car.jl")
include("measurements.jl")
include("map.jl")

using Random
using Distributions
using StatsBase

mutable struct Particle {
    Position::Vector{Float64}
    Quaternion::Quaternion
    Velocity::Vector{Float64}
    AngularVelocity::Vector{Float64}
    weight::Float64
}

function rigid_body_dynamics_particle(particle::Particle, Δt)
    r = particle.AngularVelocity
    mag = norm(r)

    sᵣ = cos(mag*Δt / 2.0)
    vᵣ = sin(mag*Δt / 2.0) * r/mag

    sₙ = particle.Quaternion[1]
    vₙ = particle.Quaternion[2:4]

    s = sₙ*sᵣ - vₙ'*vᵣ
    v = sₙ*vᵣ+sᵣ*vₙ+vₙ×vᵣ

    new_position = particle.Position + Δt * particle.Velocity
    new_quaternion = [s; v]
    new_velocity = particle.Velocity
    new_angular_vel = particle.AngularVelocity
    return [new_position; new_quaternion; new_velocity; new_angular_vel]
end

function gps_model(particle::Particle, state_channel, meas_channel; sqrt_meas_cov = Diagonal([1.0, 1.0]), max_rate=20.0)
    min_Δ = 1.0/max_rate
    t = time()
    T = get_gps_transform()
    gps_loc_body = T*[zeros(3); 1.0]
    while true
        sleep(0.0001)
        state = fetch(state_channel)
        tnow = time()
        if tnow - t > min_Δ
            t = tnow
            # Predict measurement based on particle state
            xyz_body = particle.position # position
            q_body = particle.quaternion # quaternion
            Tbody = get_body_transform(q_body, xyz_body)
            xyz_gps = Tbody * [gps_loc_body; 1]
            pred_meas = xyz_gps[1:2]

            # Generate actual measurement and update particle weight
            meas = pred_meas + sqrt_meas_cov*randn(2)
            likelihood = pdf(MvNormal(pred_meas, sqrt_meas_cov), meas)
            particle.weight *= likelihood
            gps_meas = GPSMeasurement(t, meas[1], meas[2])
            put!(meas_channel, gps_meas)
        end
    end
end

function imu_model(particle::Particle, state_channel, meas_channel; sqrt_meas_cov = Diagonal([0.01, 0.01, 0.01, 0.01, 0.01, 0.01]), max_rate=20.0)
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
            # Predict measurement based on particle state
            v_body = particle.Velocity # velocity
            ω_body = particle.AngularVelocity # angular_vel

            ω_imu = R * ω_body
            v_imu = R * v_body + p × ω_imu

            pred_meas = [v_imu; ω_imu]

            # Generate actual measurement and update particle weight
            meas = pred_meas + sqrt_meas_cov*randn(6)
            likelihood = pdf(MvNormal(pred_meas, sqrt_meas_cov), meas)
            particle.weight *= likelihood

            imu_meas = IMUMeasurement(t, meas[1:3], meas[4:6])
            put!(meas_channel, imu_meas)
        end
    end
end

function likelihood(predicted_meas, actual_meas, noise_cov)
    # Compute the likelihood of the actual measurement given the predicted measurement and the measurement noise covariance matrix
    likelihood = pdf(MvNormal(predicted_meas, noise_cov), actual_meas)
    return likelihood
end

function resample(particles)
    num_particles = length(particles)
    weights = [particle.weight for particle in particles]
    new_particles = Particle[]
    for i in 1:num_particles
        idx = sample(1:num_particles, weights, ProbabilityWeights(weights))
        push!(new_particles, deepcopy(particles[idx]))
        new_particles[i].weight = 1.0 / num_particles
    end
    return new_particles
end

function compute_state_estimate(particles::Vector{Particle})
    # Compute weighted average of particle states
    pos_mean = zeros(3)
    quat_mean = Quaternion(0.0, 0.0, 0.0, 0.0)
    vel_mean = zeros(3)
    angvel_mean = zeros(3)
    weight_sum = 0.0
    for particle in particles
        pos_mean += particle.weight * particle.position
        quat_mean += particle.weight * particle.quaternion
        vel_mean += particle.weight * particle.velocity
        angvel_mean += particle.weight * particle.angular_velocity
        weight_sum += particle.weight
    end
    pos_mean /= weight_sum
    quat_mean /= weight_sum
    vel_mean /= weight_sum
    angvel_mean /= weight_sum

    # Normalize quaternion mean
    quat_mean = normalize(quat_mean)

    # Create state estimate
    state_estimate = VehicleState(pos_mean, quat_mean, vel_mean, angvel_mean)
    return state_estimate
end


function localization(gps_channel, imu_channel, localization_state_channel)
### Step 1: Initialize particles with Gaussian distribution centered around first GPS Measurement
    num_particles = 100
    # Get initial position
    initial_pos = [0.0; 0.0; 0.0]
    initial_gps_meas = take!(gps_channel)
    initial_pos[0] = initial_gps_meas.latitude
    initial_pos[1] = initial_gps_meas.longitude

    # Initial Velocity to 0
    initial_lin_vel = [0.0; 0.0; 0.0]
    initial_ang_vel = [0.0; 0.0; 0.0]

    # Quaternion
    initial_quat = [1.0; 0.0; 1.0; 0.0]

    # Initialize particles with Gaussian distribution centered around first GPS Measurement
    num_particles = 100
    particles = []
    lat_dist = Normal(initial_pos[0], 1)
    long_dist = Normal(initial_pos[1], 1)

    particle_lat = rand(lat_dist, num_particles)
    particle_long = rand(long_dist, num_particles)
    for i in 1:num_particles
        particle_pos = [particle_lat[i]; particle_long[i]; 0.0]
        particle_quat = initial_quat
        particle_lin_vel = initial_lin_vel
        particle_ang_vel = initial_ang_vel
        particle_state = State(particle_pos, particle_quat, particle_lin_vel, particle_ang_vel)
        particle = Particle(particle_state, 1.0/num_particles)
        push!(particles, particle)
    end

    while true
        if isready(gps_channel) || isready(imu_channel)

### Step 2: Prediction Step - Propagate each particle forward using Rigid Body Dynamics
        # Get time difference
        fresh_gps_meas = take!(gps_channel)
        fresh_imu_meas = take!(imu_channel)
        Δt = fresh_gps_meas.time - initial_gps_meas.time
        # Propagate each particle forward using Rigid Body Dynamics
        for i in 1:num_particles
            particles[i] = rigid_body_dynamics_particle(particles[i], Δt)
        end

### Step 3: Update Step - Update particle weights based on GPS and IMU measurements
    # Generate predicted GPS and IMU measurements for each particle
        for i in 1:num_particles
            # Predict GPS measurement
            predicted_gps_meas = gps_model(particles[i], state_channel, gps_channel; sqrt_meas_cov = Diagonal([1.0, 1.0]), max_rate=20.0)

            # Predict IMU measurement
            predicted_imu_meas = imu_model(particles[i], state_channel, imu_channel; sqrt_meas_cov = Diagonal([0.01, 0.01, 0.01, 0.01, 0.01, 0.01]), max_rate=20.0)

            # Compute the likelihood of the actual GPS measurement given the predicted GPS measurement and the measurement noise covariance matrix
            gps_likelihood = likelihood(predicted_gps_meas, fresh_gps_meas, gps_noise_cov)

            # Compute the likelihood of the actual IMU measurement given the predicted IMU measurement and the measurement noise covariance matrix
            imu_likelihood = likelihood(predicted_imu_meas, fresh_imu_meas, imu_noise_cov)

            # Update particle weight
            particles[i].weight *= gps_likelihood * imu_likelihood
        end
### Step 4: Normalize particle weights
            # Normalize particle weights
            weights = [p.weight for p in particles]
            weights = weights ./ sum(weights)
            for i in 1:num_particles
                particles[i].weight = weights[i]
            end
        
### Step 5: Resample particles
            # Resample particles
            particles = resample(particles)
        end
        estimated_state = compute_state_estimate(particles)
        put!(localization_state_channel, estimated_state)
    end
end




