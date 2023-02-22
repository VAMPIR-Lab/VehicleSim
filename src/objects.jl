mutable struct Quaternion{T}
    quaternion::SVector{4, T}
end
function Base.zero(::Type{Quaternion{T}}) where T
    Quaternion{T}([one(T), zero(T), zero(T), zero(T)])
end

function Base.getindex(q::Quaternion, i)
    q.quaternion[i]
end
function Base.setindex!(q::Quaternion{T}, v::T, i) where T
    q.quaternion[i] = v
end
function Base.firstindex(q::Quaternion)
    1
end
function Base.lastindex(q::Quaternion)
    4
end


function quaternion_to_rotation_matrix(q::Quaternion)
    w = q[1]
    x = q[2]
    y = q[3]
    z = q[4]
    @SMatrix [1-2y^2-2z^2 2x*y-2w*z 2x*z+2w*y;
              2x*y+2w*z 1-2x^2-2z^2 2y*z-2w*x
              2x*z-2w*y 2y*z+2w*x 1-2x^2-2y^2]
end

function quaternion_multiply!(q::Quaternion{T}, q1::Quaternion{T}, q2::Quaternion{T}) where T
    s1 = q1[1]
    s2 = q2[1]
    v1 = @SVector [q1.quaternion.x, q1.quaternion.y, q1.quaternion.z]
    v2 = @SVector [q2.quaternion.x, q2.quaternion.y, q2.quaternion.z]
    s = s1*s2 - v1'*v2
    v = s1*v2+s2*v1+v1×v2
    q[1] = s
    q[2:4] = v
end

function normalize!(q::Quaternion)
    qnorm = sqrt(q[1]^2 + q[2]^2 + q[3]^2 + q[4]^2)
    q[1:4] ./= qnorm
    nothing
end


function quaternion_multiply!(q, ω, q2::Quaternion{T}) where T
    s1 = zero(T) 
    s2 = q2[1]
    v1 = @SVector [ω[1], ω[2], ω[3]]
    v2 = @SVector [q2[2], q2[3], q2[4]]
    s = s1*s2 - v1'*v2
    v = s1*v2+s2*v1+v1×v2
    q[1] = s
    q[2:4] = v
end

struct RigidBodyState{T} <: AbstractVector{T}
    x::SVector{3, T}
    q::Quaternion{T}
    P::SVector{3, T}
    L::SVector{3, T}
end

function Base.zero(::Type{RigidBodyState{T}}) where T
    RigidBodyState{T}(zero(SVector{3,T}), zero(Quaternion{T}), zero(SVector{3,T}), zero(SVector{3,T}))
end
function Base.zero(::RigidBodyState{T}) where T
    RigidBodyState{T}(zero(SVector{3,T}), zero(Quaternion{T}), zero(SVector{3,T}), zero(SVector{3,T}))
end
function Base.getindex(rb::RigidBodyState{T}, i::Int) where T
    if i < 1 
        error("Out of bounds")
    elseif i < 4
        @inbounds rb.x[i]
    elseif i < 8
        @inbounds rb.q[i-3]
    elseif i < 11
        @inbounds rb.P[i-7]
    elseif i < 14
        @inbounds rb.L[i-10]
    else
        error("Out of bounds")
    end
end
function Base.setindex(rb::RigidBodyState{T}, v::T, i::Int) where T
    if i < 1 
        error("Out of bounds")
    elseif i < 4
        @inbounds rb.x[i] = v
    elseif i < 8
        @inbounds rb.q[i-3] = v
    elseif i < 11
        @inbounds rb.P[i-7] = v
    elseif i < 14
        @inbounds rb.L[i-10] = v
    else
        error("Out of bounds")
    end
end
function Base.firstindex(rb::RigidBodyState{T}) where T
    1
end
function Base.lastindex(rb::RigidBodyState{T}) where T
    13
end
function Base.length(rb::RigidBodyState{T}) where T
    13
end
function Base.eltype(::Type{RigidBodyState{T}}) where T
    T
end
function Base.iterate(rb::RigidBodyState{T}) where T
    rb[1], 2
end
function Base.iterate(rb::RigidBodyState{T}, state) where T
    return state > 13 ? nothing : (rb[state], state+1)
end

struct RigidBodyParameters
    mass::Float64
    I⁻¹::SMatrix{3,3,Float64}
end

function rigid_body!(drbs, rbs, p, t)
    (I⁻¹, mass, F, r, Fg) = p(t)
    q = Quaternion{Float64}(rbs[4:7])
    normalize!(q)
    R = quaternion_to_rotation_matrix(q)
    I⁻¹ = R' * I⁻¹ * R
    ω = I⁻¹ * rbs[11:13]
    τ = (R*r) × F - 100*ω
    drbs[1:3] = rbs[8:10] ./ mass
    quaternion_multiply!(@view(drbs[4:7]), ω, q)
    drbs[8:10] = F + Fg
    drbs[11:13] = τ
end



function test_simulate(vis, urdf_filename)
    urdf_path = joinpath(dirname(pathof(VehicleSim)), "assets", urdf_filename)
    mechanism = parse_urdf(urdf_path)
    mechanism_state = MechanismState(mechanism)
    J = first(joints(mechanism))

    mvis = MechanismVisualizer(mechanism, URDFVisuals(urdf_path, package_path=[dirname(pathof(VehicleSim))]), vis)
    
    rbs = zero(RigidBodyState{Float64})
    set_configuration!(mechanism_state, J, [rbs.q[1:4]; rbs.x])
    set_configuration!(mvis, configuration(mechanism_state))

    inertia_obj = mechanism.graph.vertices[2] |> spatial_inertia
    rb_params = RigidBodyParameters(inertia_obj.mass, inv(inertia_obj.moment))
    F = @SVector [0, 0, 9.81*(rb_params.mass)]
    Fg = @SVector [0, 0, -9.81*(rb_params.mass)]
    r = @SVector [5, 2.5, 2.5]

    p = t -> (rb_params.I⁻¹, rb_params.mass, F, r, Fg)

    tspan = (0.0, 100.0)
    rbs = zero(MVector{13, Float64})
    rbsdot = zero(MVector{13, Float64})
    rbs[4] = 1.0
    prob = ODEProblem(rigid_body!, rbs, tspan, p)
    sol = solve(prob)

    @infiltrate
    for t = 1:1000000 
        u = sol(t/10000.0)
        set_configuration!(mechanism_state, J, [u[4:7]; u[1:3]])
        set_configuration!(mvis, configuration(mechanism_state))
    end
end
