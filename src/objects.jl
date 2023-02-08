mutable struct Quaternion{T}
    w::T
    x::T
    y::T
    z::T
end
function Base.zero(::Type{Quaternion{T}}) where T
    Quaternion{T}(one(T), zero(T), zero(T), zero(T))
end

function Base.collect(q::Quaternion{T}) where T
    SVector{4, T}(q.w, q.x, q.y, q.z)
end

function quaternion_to_rotation_matrix(q::Quaternion)
    w = q.w
    x = q.x
    y = q.y
    z = q.z
    @SMatrix [1-2y^2-2z^2 2x*y-2w*z 2x*z+2w*y;
              2x*y+2w*z 1-2x^2-2z^2 2y*z-2w*x
              2x*z-2w*y 2y*z+2w*x 1-2x^2-2y^2]
end

function Base.:*(q1::Quaternion{T}, q2::Quaternion{T}) where T
    s1 = q1.w
    s2 = q2.w
    v1 = @SVector [q1.x, q1.y, q1.z]
    v2 = @SVector [q2.x, q2.y, q2.z]
    s = s1*s2 - v1'*v2
    v = s1*v2+s2*v1+v1×v2
    Quaternion{T}(s, v[1], v[2], v[3])
end

function normalize!(q::Quaternion)
    mag = sqrt(q.w^2 + q.x^2 + q.y^2 + q.z^2)
    q.w /= mag
    q.x /= mag
    q.y /= mag
    q.z /= mag
    nothing
end

function Base.:*(ω::SVector{3, T}, q::Quaternion{T}) where T
    Quaternion{T}(zero(T), ω[1], ω[2], ω[3]) * q
end
mutable struct RigidBody{T}
    mass::Float64
    I⁻¹::SMatrix{3,3,Float64}
    x::SVector{3, T}
    q::Quaternion{T}
    P::SVector{3, T}
    L::SVector{3, T}
end
function RigidBody{T}(mass, I⁻¹) where T
    RigidBody{T}(mass, I⁻¹, zero(SVector{3, T}), zero(Quaternion{T}), zero(SVector{3, T}), zero(SVector{3, T}))
end

struct RigidBodyTimeDerivative{T}
    ẋ::SVector{3, T}
    q̇::Quaternion{T}
    Ṗ::SVector{3, T}
    L̇::SVector{3, T}
end


function time_derivative(rigid_body, F, τ)
    v = rigid_body.P ./ rigid_body.mass
    R = quaternion_to_rotation_matrix(rigid_body.q)
    I⁻¹ = R' * rigid_body.I⁻¹ * R
    ω = I⁻¹ * rigid_body.L
    q̇ = 0.5 * ω * rigid_body.q
    RigidBodyTimeDerivative(v, q̇, F, τ)
end

function update_rigid_body!(rigid_body, time_derivative, Δ)
    rigid_body.x += Δ * time_derivative.ẋ
    foreach(field->setproperty!(rigid_body.q, field,  getproperty(rigid_body.q, field) + getproperty(time_derivative.q̇, field)), propertynames(rigid_body.q))
    #rigid_body.q += Δ * time_derivative.q̇
    rigid_body.P += Δ * time_derivative.Ṗ
    rigid_body.L += Δ * time_derivative.L̇
    normalize!(rigid_body.q)
    nothing
end

function test_simulate(vis, urdf_filename)
    urdf_path = joinpath(dirname(pathof(VehicleSim)), "assets", urdf_filename)
    mechanism = parse_urdf(urdf_path)
    mechanism_state = MechanismState(mechanism)
    J = first(joints(mechanism))
    inertia_obj = mechanism.graph.vertices[2] |> spatial_inertia
    rb = RigidBody{Float64}(inertia_obj.mass, inv(inertia_obj.moment))
    mvis = MechanismVisualizer(mechanism, URDFVisuals(urdf_path, package_path=[dirname(pathof(VehicleSim))]), vis)
    
    Δ = 0.001
    g = [0, 0, -9.81]


    set_configuration!(mechanism_state, J, [collect(rb.q); rb.x])
    set_configuration!(mvis, configuration(mechanism_state))

    F = @SVector [0, 0, 9.81*(inertia_obj.mass)]
    Fg = @SVector [0, 0, -9.81*(inertia_obj.mass)]
    r = @SVector [0, 0.001, 2.5]

    @infiltrate
    for t = 1:10000
        R = quaternion_to_rotation_matrix(rb.q)
        r_world = R*r
        τ = r_world × F
        rb_dot = time_derivative(rb, F+Fg, τ)
        update_rigid_body!(rb, rb_dot, Δ)
        set_configuration!(mechanism_state, J, [collect(rb.q); rb.x])
        set_configuration!(mvis, configuration(mechanism_state))
        println(rb.q)
        sleep(0.001)
    end
end
