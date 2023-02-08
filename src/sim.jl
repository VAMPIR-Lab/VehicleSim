
function simulate(vis, urdf_filename)
    urdf_path = joinpath(dirname(pathof(VehicleSim)), "assets", urdf_filename)
    mechanism = parse_urdf(urdf_path)
    mech_joints = joints(mechanism)
    
    x1 = [0,0,0]
    x2 = [5.0, 5.0, 10.0]
    v1 = zeros(3)
    v2 = zeros(3)
    quat1 = QuatRotation(RotXYZ(0,1.57079632679,0))
    quat2 = QuatRotation(RotXYZ(0,1.57079632679,0))
    q1 = [quat1.w, quat1.x, quat1.y, quat1.z]
    q2 = [quat2.w, quat2.x, quat2.y, quat2.z]

    J1 = mech_joints[1]
    J2 = mech_joints[2]

    #parent = predecessor(joint, mechanism)
    #child = successor(joint, mechanism)
    #to_parent = joint_to_predecessor(joint)
    #xyz = translation(to_parent)
    #rpy = RotZYX(rotation(to_parent))

    q = [q1; x1; q2; x2]
    v = Vector{Float64}(undef, num_velocities(mechanism))
    s = Vector{Float64}(undef, num_additional_states(mechanism))
    mechanism_state = MechanismState(mechanism, q, v, s)

    #mechanism_state = MechanismState(mechanism)
    mvis = MechanismVisualizer(mechanism, URDFVisuals(urdf_path, package_path=[dirname(pathof(VehicleSim))]), vis)
    
    I1 = mechanism.graph.vertices[2] |> spatial_inertia
    I2 = mechanism.graph.vertices[3] |> spatial_inertia
    r1 = [-5.0, 2.5, 2.5]
    r2 = [5.0, -2.5, -2.5]
    m1 = I1.mass
    m2 = I2.mass
    I1 = I1.moment
    I2 = I2.moment
    Δ = 0.001
    g = [0, 0, -9.81]

    callbacks = generate_callbacks(I1, I2, m1, m2, r1, r2, Δ, g)
    
    set_configuration!(mechanism_state, J1, [q1; x1])
    set_configuration!(mechanism_state, J2, [q2; x2])
    set_configuration!(mvis, configuration(mechanism_state))

    ω1 = zeros(3)
    ω2 = zeros(3)
    F1 = [0, 0, 0.0]
    F2 = [0, 0, 9.81*(m1+m2)]
    f2mag = 9.81*(m1+m2)
    r01 = [-5.0, 2.5, 2.5]
    r02 = [-5.0, -2.5, -2.5]
    @infiltrate
    
    for t = 1:10000
        
        direction = [0,0,20] - x2
        F2 = direction ./ norm(direction) * f2mag * (norm(x2[1:2]) + 1)
        F2 = zeros(3)

        ret = solve(callbacks, x1, x2, v1, v2, q1, q2, ω1, ω2, F1, r01, F2, r02)
        x1 = ret.x1n
        x2 = ret.x2n
        v1 = ret.v1n
        v2 = ret.v2n
        ω1 = ret.ω1n
        ω2 = ret.ω2n
        #Q1 = Q(ω1)/2
        #Q2 = Q(ω2)/2
        #q1 = exp(Q1*Δ)*q1
        #q2 = exp(Q2*Δ)*q2

        q1 = ret.q1n
        q1 ./= norm(q1)
        q2 = ret.q2n
        q2 ./= norm(q2)

        set_configuration!(mechanism_state, J1, [q1; x1])
        set_configuration!(mechanism_state, J2, [q2; x2])
        set_configuration!(mvis, configuration(mechanism_state))
        println("diff:", x2-x1)
        println("cross:", quat_rotate(r2, q2) × quat_rotate(r1, q1))
    end

end

function solve(callbacks, x1, x2, v1, v2, q1, q2, ω1, ω2, F1, r01, F2, r02)
    wrapper_f = function(z)
        callbacks.full_cost_fn(z, [x1; v1; q1; ω1; x2; v2; q2; ω2; F1; r01; F2; r02])
    end
    wrapper_grad_f = function(z, grad)
        callbacks.full_cost_grad_fn(grad, z, [x1; v1; q1; ω1; x2; v2; q2; ω2; F1; r01; F2; r02])
    end
    wrapper_con = function(z, con)
        callbacks.full_constraint_fn(con, z, [x1; v1; q1; ω1; x2; v2; q2; ω2; F1; r01; F2; r02])
    end
    wrapper_con_jac = function(z, rows, cols, vals)
        if isnothing(vals)
            rows .= callbacks.full_constraint_jac_triplet.jac_rows
            cols .= callbacks.full_constraint_jac_triplet.jac_cols
        else
            callbacks.full_constraint_jac_triplet.full_constraint_jac_vals_fn(vals, z, [x1; v1; q1; ω1; x2; v2; q2; ω2; F1; r01; F2; r02])
        end
        nothing
    end
    wrapper_lag_hess = function(z, rows, cols, cost_scaling, μ, vals)
        if isnothing(vals)
            rows .= callbacks.full_lag_hess_triplet.hess_rows
            cols .= callbacks.full_lag_hess_triplet.hess_cols
        else
            callbacks.full_lag_hess_triplet.full_hess_vals_fn(vals, z, [x1; v1; q1; ω1; x2; v2; q2; ω2; F1; r01; F2; r02], μ, cost_scaling)
        end
        nothing
    end

    n = 41
    m = callbacks.num_constraints
    prob = Ipopt.CreateIpoptProblem(
        n,
        fill(-Inf, n),
        fill(Inf, n),
        m,
        fill(0.0, m),
        fill(0.0, m),
        length(callbacks.full_constraint_jac_triplet.jac_rows),
        length(callbacks.full_lag_hess_triplet.hess_rows),
        wrapper_f,
        wrapper_con,
        wrapper_grad_f,
        wrapper_con_jac,
        wrapper_lag_hess
    )

    zinit = [x1; v1; q1; ω1; x2; v2; q2; ω2; zeros(3); [0,0,9.81*1000,0,0,9.81*1000,0,0,0,0,0,0]]

    prob.x = zinit

    Ipopt.AddIpoptIntOption(prob, "print_level", 0)
    status = Ipopt.IpoptSolve(prob)

    if status != 0 && status != 1
        @warn "Problem not cleanly solved. IPOPT status is $(status)."
    end

    z = prob.x
    x1n = z[1:3]
    v1n = z[4:6]
    q1n = z[7:10]
    ω1n = z[11:13]
    x2n = z[14:16]
    v2n = z[17:19]
    q2n = z[20:23]
    ω2n = z[24:26]
    λ = z[27:29]
    λr = z[30:32]
    (; x1n, v1n, q1n, ω1n, x2n, v2n, q2n, ω2n, λ)
end

function generate_callbacks(I1, I2, m1, m2, r1, r2, Δ, g)
    params = Symbolics.scalarize(Symbolics.@variables(params[1:38])[1])
    z = Symbolics.scalarize(Symbolics.@variables(z[1:41])[1])

    x1 = params[1:3]
    v1 = params[4:6]
    q1 = params[7:10]
    ω1 = params[11:13]
    x2 = params[14:16]
    v2 = params[17:19]
    q2 = params[20:23]
    ω2 = params[24:26]
    F1 = params[27:29]
    r01 = params[30:32]
    F2 = params[33:35]
    r02 = params[36:38]
    
    x1n = z[1:3]
    v1n = z[4:6]
    q1n = z[7:10]
    ω1n = z[11:13]
    x2n = z[14:16]
    v2n = z[17:19]
    q2n = z[20:23]
    ω2n = z[24:26]
    λ = z[27:29]
    λ1 = z[30:32]
    λ2 = z[33:35]
    τ1 = z[36:38]
    τ2 = z[39:41]

    r0n1 = quat_rotate(r01, q1n)
    r0n2 = quat_rotate(r02, q2n)
    r1n = quat_rotate(r1, q1n)
    r2n = quat_rotate(r2, q2n)
    r1o = quat_rotate(r1, q1)
    r2o = quat_rotate(r2, q2)

    F1all = m1*g + F1 + λ1 
    T1all = r0n1 × F1 + r1n × λ1 + τ1
    F2all = [0,0,0]#m2*g + F2 + r2n × -λ1
    T2all = [0,0,0]#r0n2 × F2 - τ1

    R1 = quat_to_rotation(q1)
    R2 = quat_to_rotation(q2)

    I1t = R1*I1*R1'
    I2t = R2*I2*R2'

    a1 = F1all ./ m1
    a2 = F2all ./ m2
    α1 = I1t \ (T1all - ω1n × (I1 * ω1n))
    α2 = I2t \ (T2all - ω2n × (I2 * ω2n))
    
    q̇1 = 0.5 * Q(ω1n) * q1
    q̇2 = 0.5 * Q(ω2n) * q2

    cons = Vector{Symbolics.Num}()
    append!(cons, x1n - (x1+Δ*v1n))
    append!(cons, x2n - (x2+Δ*v2n))
    append!(cons, q1n - (q1+Δ*q̇1))
    append!(cons, q2n - (q2+Δ*q̇2))
    append!(cons, v1n - (v1+Δ*a1))
    append!(cons, v2n - (v2+Δ*a2))
    append!(cons, ω1n - (ω1+Δ*α1))
    append!(cons, ω2n - (ω2+Δ*α2))
    append!(cons, x1n+r1n - (x2n+r2n))

    #append!(cons, x2n-x1n - ([5.0,5,10]))
    m = length(cons)

    diff = (r2n × r1n)
    #diff = (x2n-x1n) - [5, 5, 10.0]
    
    constraints_jac = Symbolics.sparsejacobian(cons, z)
    (jac_rows, jac_cols, jac_vals) = findnz(constraints_jac)
    cost_scaling, μ = let
        Symbolics.@variables(cost_scaling, μ[1:m]) .|> Symbolics.scalarize
    end
    
    #cost_val = 0.5 * (λ'*λ + λ1'*λ1 + λ2'*λ2 + τ1'*τ1 + τ2'*τ2) + 100*diff'*diff
    cost_val = 0.5 * (λ1'*λ1) + 0.5*τ1'*τ1 + 100*ω1n'*ω1n
    cost_grad = Symbolics.gradient(cost_val, z)

    lag = cost_scaling * cost_val + sum(μ[i]*cons[i] for i = 1:m)
    lag_grad = Symbolics.gradient(lag, z)
    lag_hess = Symbolics.sparsejacobian(lag_grad, z)
    (hess_rows, hess_cols, hess_vals) = findnz(lag_hess)

    expression = Val{false}    

    full_cost_fn = let
        cost_fn = Symbolics.build_function(cost_val, [z; params]; expression)
        (z, params) -> cost_fn([z; params])
    end
    full_cost_grad_fn = let
        cost_grad_fn! = Symbolics.build_function(cost_grad, [z; params]; expression)[2]
        (grad, z, params) -> cost_grad_fn!(grad, [z; params])
    end
    full_constraint_fn = let
        constraint_fn! = Symbolics.build_function(cons, [z; params]; expression)[2]
        (cons, z, params) -> constraint_fn!(cons, [z; params])
    end
    full_constraint_jac_vals_fn = let
        constraint_jac_vals_fn! = Symbolics.build_function(jac_vals, [z; params]; expression)[2]
        (vals, z, params) -> constraint_jac_vals_fn!(vals, [z; params])
    end
    full_hess_vals_fn = let
        lag_hess_vals_fn! = Symbolics.build_function(hess_vals, [z; params; μ; cost_scaling]; expression)[2]
        (vals, z, params, μ, cost_scaling) -> lag_hess_vals_fn!(vals, [z; params; μ; cost_scaling])
    end

    full_constraint_jac_triplet = (; jac_rows, jac_cols, full_constraint_jac_vals_fn)
    full_lag_hess_triplet = (; hess_rows, hess_cols, full_hess_vals_fn)

    return (; full_cost_fn,
            full_cost_grad_fn,
            full_constraint_fn,
            full_constraint_jac_triplet,
            full_lag_hess_triplet, num_constraints=m)
end

function Q(q)
    [0 -q[1] -q[2] -q[3];
     q[1] 0 q[3] -q[2];
     q[2] -q[3] 0 q[1];
     q[3] q[2] -q[1] 0]
end

"""
    vprime = 2.0f * dot(u, v) * u
          + (s*s - dot(u, u)) * v
          + 2.0f * s * cross(u, v);
"""
function quat_rotate(v, q)
    u = q[2:4]
    s = q[1]
    2.0 * u'v * u + (s^2 - u'u) * v + 2.0*s*u×v
end

function quat_to_rotation(q)
    w = q[1]
    x = q[2]
    y = q[3]
    z = q[4]
    [1-2y^2-2z^2 2x*y-2w*z 2x*z+2w*y;
     2x*y+2w*z 1-2x^2-2z^2 2y*z-2w*x
     2x*z-2w*y 2y*z+2w*x 1-2x^2-2y^2]
end
