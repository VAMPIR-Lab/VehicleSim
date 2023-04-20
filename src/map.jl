@enum LaneTypes begin
    standard
    loading_zone
    intersection
    stop_sign
end

@enum Direction begin
    north
    east
    south
    west
end

function opposite(direction::Direction)
    direction == north && return south
    direction == east && return west
    direction == south && return north
    direction == west && return east
end



"""
Assume that pt_b is further along driving direction than pt_a
"""
struct LaneBoundary
    pt_a::SVector{2,Float64}
    pt_b::SVector{2,Float64}
    curvature::Float64
    hard_boundary::Bool
    visualized::Bool
end

function lane_boundary(pt_a, pt_b, hard, vis, left=true)
    dx = abs(pt_a[1] - pt_b[1])
    dy = abs(pt_a[2] - pt_b[2])
    if isapprox(dx - dy, 0; atol=1e-6)
        sign = left ? 1.0 : -1.0
        LaneBoundary(pt_a, pt_b, sign / dx, hard, vis)
    else
        LaneBoundary(pt_a, pt_b, 0.0, hard, vis)
    end
end

"""
lane_boundaries are in order from left to right
"""
mutable struct RoadSegment
    id::Int
    lane_boundaries::Vector{LaneBoundary}
    lane_types::Vector{LaneTypes}
    speed_limit::Float64
    children::Vector{Int}
end

function reached_target(pos, vel, seg)
    @assert length(seg.lane_types) > 1 && seg.lane_types[2] == loading_zone
    A = seg.lane_boundaries[2].pt_a
    B = seg.lane_boundaries[2].pt_b
    C = seg.lane_boundaries[3].pt_a
    D = seg.lane_boundaries[3].pt_b
    min_x = min(A[1], B[1], C[1], D[1])
    max_x = max(A[1], B[1], C[1], D[1])
    min_y = min(A[2], B[2], C[2], D[2])
    max_y = max(A[2], B[2], C[2], D[2])
    return min_x <= pos[1] <= max_x && min_y <= pos[2] <= max_y && abs(vel) < 0.1
end

"""
Assuming only 90° turns for now
"""
function generate_laneline_mesh(lb::LaneBoundary; res=1.0, width=0.3)
    pt_a = lb.pt_a
    pt_b = lb.pt_b
    curvature = lb.curvature
    if lb.visualized
        if lb.hard_boundary
            color = RGBA{Float32}(1,1,0,1)
        else
            color = RGBA{Float32}(1,1,0,0.5)
        end
    else
        return nothing
    end
    curved = !isapprox(curvature, 0.0; atol=1e-6)
    delta = pt_b-pt_a
    if curved
        rad = 1.0 / abs(curvature)
        dist = π*rad/2.0
    else
        dist = norm(pt_b-pt_a) - 0.02
    end
    N = (dist / res) |> ceil |> Int
    Δ = dist / N
    t = 0:Δ:dist .+ 0.01
    if curved
        left = curvature > 0
        if left
            if sign(delta[1]) == sign(delta[2])
                center = pt_a + [0, delta[2]]
            else
                center = pt_a + [delta[1], 0]
            end
        else
            if sign(delta[1]) == sign(delta[2])
                center = pt_a + [delta[1], 0]
            else
                center = pt_a + [0, delta[2]]
            end
        end
        pt_a_rel = pt_a - center
        pt_b_rel = pt_b - center
        θ0 = atan(pt_a_rel[2], pt_a_rel[1])
        θT = atan(pt_b_rel[2], pt_b_rel[1]) 
        Δθ = mod((θT - θ0) + π, 2π) - π
        dθ = sign(Δθ) * π/2.0 / N
        rad_1 = rad - width/2
        rad_2 = rad + width/2
        points = mapreduce(vcat, 0:N) do i
            θ = θ0 + dθ * i
            [GeometryBasics.Point{3,Float64}(center[1]+rad_1*cos(θ), center[2]+rad_1*sin(θ), 0),
             GeometryBasics.Point{3,Float64}(center[1]+rad_2*cos(θ), center[2]+rad_2*sin(θ), 0)]
        end
    else
        dir = delta / norm(delta)
        right = [dir[2], -dir[1]]
        points = mapreduce(vcat, t) do dist  
            pt_1 = pt_a+dir*dist-width/2*right
            pt_2 = pt_a+dir*dist+width/2*right
            [GeometryBasics.Point{3, Float64}(pt_1[1], pt_1[2], 0),
             GeometryBasics.Point{3, Float64}(pt_2[1], pt_2[2], 0)]
        end
    end
    K = length(points)/2 |> Int

    faces = mapreduce(vcat, 1:K-1) do k
        [GeometryBasics.TriangleFace((k-1)*2+1,k*2+1,k*2+2),
         GeometryBasics.TriangleFace((k-1)*2+1,k*2,k*2+2)]
    end
    mesh = GeometryBasics.Mesh(points, faces)
    MeshCat.Object(mesh, MeshPhongMaterial(color=color))
end

function generate_lane_mesh(lb1, lb2, lane_type; width=0.3, res=1.0)
    if lane_type == standard || lane_type == stop_sign || lane_type == intersection
        color=RGBA{Float32}(.35,.35,.35,1)
    elseif lane_type == loading_zone
        color=RGBA{Float32}(.5,.5,.5,1)
    end
    pt_a = lb1.pt_a
    pt_b = lb1.pt_b
    curvature = lb1.curvature
    curved = !isapprox(curvature, 0.0; atol=1e-6)
    delta = pt_b-pt_a
    if curved
        rad = 1.0 / abs(curvature)
        dist = π*rad/2.0
    else
        dist = norm(pt_b-pt_a)-0.002
    end
    N = (dist / res) |> ceil |> Int
    if curved
        left = curvature > 0
        if left
            if sign(delta[1]) == sign(delta[2])
                center = pt_a + [0, delta[2]]
            else
                center = pt_a + [delta[1], 0]
            end
        else
            if sign(delta[1]) == sign(delta[2])
                center = pt_a + [delta[1], 0]
            else
                center = pt_a + [0, delta[2]]
            end
        end
        pt_a_rel = pt_a - center
        pt_b_rel = pt_b - center
        θ0 = atan(pt_a_rel[2], pt_a_rel[1])
        θT = atan(pt_b_rel[2], pt_b_rel[1]) 
        Δθ = mod((θT - θ0) + π, 2π) - π
        dθ = sign(Δθ) * π/2.0 / N
        rad_1 = rad
        rad_2 = abs(lb2.pt_b[1]-lb2.pt_a[1])
        if rad_1 > rad_2
            rad_1 -= width/2
            rad_2 += width/2
        else
            rad_1 += width/2
            rad_2 -= width/2
        end
        points = mapreduce(vcat, 0:N) do i
            θ = θ0 + dθ * i
            [GeometryBasics.Point{3,Float64}(center[1]+rad_1*cos(θ), center[2]+rad_1*sin(θ), 0),
             GeometryBasics.Point{3,Float64}(center[1]+rad_2*cos(θ), center[2]+rad_2*sin(θ), 0)]
        end
    else
        dir1 = delta / norm(delta)
        right1 = [dir1[2], -dir1[1]]
        delta2 = lb2.pt_b - lb2.pt_a
        dir2 = delta2 / norm(delta2)
        right2 = [dir2[2], -dir2[1]]
        dist2 = norm(delta2)

        Δ = dist / N
        t = 0:Δ:dist .+ 0.001
        Δ2 = dist2 / N
        t2 = 0:Δ2:dist2 .+ 0.001

        points = mapreduce(vcat, zip(t, t2)) do (dist1, dist2)
            pt_1 = pt_a+dir1*dist1+width/2*right1
            pt_2 = lb2.pt_a + dir2*dist2-width/2*right2
            [GeometryBasics.Point{3,Float64}(pt_1[1], pt_1[2], 0),
             GeometryBasics.Point{3,Float64}(pt_2[1], pt_2[2], 0)]
        end
    end

    if lane_type == stop_sign
        points_end = points[end-3:end]
        points = points[1:end-2]
        faces_end = [GeometryBasics.TriangleFace(1,3,4),
                     GeometryBasics.TriangleFace(1,2,4)]
    else
        faces_end = nothing
    end

    K = length(points)/2 |> Int

    faces = mapreduce(vcat, 1:K-1) do k
        [GeometryBasics.TriangleFace((k-1)*2+1,k*2+1,k*2+2),
         GeometryBasics.TriangleFace((k-1)*2+1,k*2,k*2+2)]
    end
    mesh = GeometryBasics.Mesh(points, faces)
    obj = MeshCat.Object(mesh, MeshPhongMaterial(color=color))

    if lane_type == stop_sign
        mesh_end = GeometryBasics.Mesh(points_end, faces_end)
        white = RGBA{Float32}(1,1,1,1)
        obj_end = MeshCat.Object(mesh_end, MeshPhongMaterial(color=white))
    else
        obj_end = nothing
    end
    (; obj, obj_end)
end

function ground_mesh()
    green=RGBA{Float32}(0.34,0.49,0.27,1.0)
    points = GeometryBasics.Point{3,Float64}[]
    for x in [-200.0, 200.0]
        for y in [-200.0, 200.0]
            push!(points, GeometryBasics.Point{3,Float64}(x,y,-0.25))
        end
    end
    faces = [GeometryBasics.TriangleFace(1,2,3), GeometryBasics.TriangleFace(2,4,3)]
    mesh = GeometryBasics.Mesh(points, faces)
    obj = MeshCat.Object(mesh, MeshPhongMaterial(color=green))
end


function view_map(vis, all_segs)
    delete!(vis["Grid"])
    gm = ground_mesh()
    setobject!(vis["ground"], gm)
    for (id, seg) in all_segs
        meshes = generate_road_segment_mesh(seg)
        for (mesh_id, m) in meshes
            if isnothing(m)
                continue
            end
            setobject!(vis["map"]["$id"][mesh_id], m)
        end
    end
end

function generate_road_segment_mesh(seg; lane_width=0.3, poly_res=1.0)
    meshes = Dict{String, Any}()
    for (e,lb) in enumerate(seg.lane_boundaries)
        m = generate_laneline_mesh(lb; width=lane_width, res=poly_res)
        meshes["line_$e"] = m
        if e ≤ length(seg.lane_types)
            ms = generate_lane_mesh(lb, seg.lane_boundaries[e+1], seg.lane_types[e]; res=poly_res)
            meshes["lane_$e"] = ms.obj
            if !isnothing(ms.obj_end)
                meshes["stop_$e"] = ms.obj_end
            end
        end
    end
    meshes
end

function contains_lane_type(seg, types...)
    return any(lane_type ∈ types for lane_type in seg.lane_types)
end

function istaper(seg)
    A = seg.lane_boundaries[2].pt_a
    B = seg.lane_boundaries[2].pt_b
    C = seg.lane_boundaries[3].pt_a
    D = seg.lane_boundaries[3].pt_b
    norm(A-C) < 0.1 || norm(B-D) < 0.1
end

function identify_loading_segments(map)
    target_segments = []
    for (id, seg) in map
        if contains_lane_type(seg, loading_zone) && !istaper(seg)
            push!(target_segments, id)
        end
    end
    target_segments
end

function get_initialization_point(seg)
    lb_1 = seg.lane_boundaries[1]
    lb_2 = seg.lane_boundaries[2]

    pt_a = lb_1.pt_a
    pt_b = lb_1.pt_b
    pt_c = lb_2.pt_a
    pt_d = lb_2.pt_b

    curvature = lb_1.curvature
    curved = !isapprox(curvature, 0.0; atol=1e-6)
    delta = pt_b-pt_a
    if !curved
        pt = 0.25*(pt_a+pt_b+pt_c+pt_d)
        yaw = atan(delta[2], delta[1])
    else
        rad = 1.0 / abs(curvature)
        dist = π*rad/2.0
        left = curvature > 0
        if left
            if sign(delta[1]) == sign(delta[2])
                center = pt_a + [0, delta[2]]
            else
                center = pt_a + [delta[1], 0]
            end
        else
            if sign(delta[1]) == sign(delta[2])
                center = pt_a + [delta[1], 0]
            else
                center = pt_a + [0, delta[2]]
            end
        end
        pt_a_rel = pt_a - center
        pt_b_rel = pt_b - center

        θ0 = atan(pt_a_rel[2], pt_a_rel[1])
        θT = atan(pt_b_rel[2], pt_b_rel[1])

        θ = 0.5*(θ0 + θT)

        rad_1 = rad
        rad_2 = abs(pt_d[1]-pt_c[1])

        rad = 0.5*(rad_1+rad_2)

        pt = center + rad*[cos(θ), sin(θ)]
        yaw = left ? θ + π/2 : θ - π/2
    end
    CarConfig(SVector(pt[1], pt[2], 3.25), 0, 0, yaw, 0)
end

function training_map(; lane_width = 10.0,
                        speed_limit = 10.0,
                        pullout_length = 40.0,
                        pullout_taper = 10.0,
                        block_length = 80.0,
                        turn_curvature = 0.1,
                        intersection_curvature = 0.15)
    turn_r = 1.0/turn_curvature
    int_r = 1.0/intersection_curvature

    shortened_block_length = block_length - 2*(turn_r-int_r)
    single_shortened_block_length = block_length - (turn_r-int_r)

    all_segs = Dict{Int, RoadSegment}()

    segs_I = add_fourway_intersection!(all_segs, nothing, nothing; intersection_curvature, speed_limit, lane_width)
    segs = add_straight_segments!(all_segs, segs_I, west; length=block_length, speed_limit, stop_outbound=true, stop_inbound=true)
    segs_T = add_T_intersection!(all_segs, segs, west, east; intersection_curvature, lane_width, speed_limit)
    segs_S = add_pullout_segments!(all_segs, segs_T, south; length=block_length, pullout_length, pullout_taper, lane_width, speed_limit, pullout_inbound=false, pullout_outbound=true)
    segs_S = add_curved_segments!(all_segs, segs_S, south, true; turn_curvature, speed_limit, lane_width)
    segs_S = add_pullout_segments!(all_segs, segs_S, east; length=shortened_block_length, pullout_length, pullout_taper, lane_width, speed_limit, pullout_inbound=true, pullout_outbound=false)
    segs_S = add_curved_segments!(all_segs, segs_S, east, true; turn_curvature, speed_limit, lane_width)
    segs_S = add_straight_segments!(all_segs, segs_S, north; length=block_length, speed_limit, stop_outbound=true, stop_inbound=false)
    segs_I = add_segments!(all_segs, segs_S, north, segs_I)

    segs_N = add_pullout_segments!(all_segs, segs_T, north; length=single_shortened_block_length, pullout_length, pullout_taper, lane_width, speed_limit, pullout_inbound=true, pullout_outbound=false)
    segs_N = add_curved_segments!(all_segs, segs_N, north, false; turn_curvature, speed_limit, lane_width)
    segs_N = add_straight_segments!(all_segs, segs_N, east; length=single_shortened_block_length, speed_limit, stop_outbound=true, stop_inbound=false)
    segs_N = add_T_intersection!(all_segs, segs_N, east, south; intersection_curvature, lane_width, speed_limit)
    segs_N2 = add_pullout_segments!(all_segs, segs_N, south; length=block_length, pullout_length, pullout_taper, lane_width, speed_limit, pullout_inbound=true, pullout_outbound=true, stop_inbound=true, stop_outbound=true)
    segs_I = add_segments!(all_segs, segs_N2, south, segs_I)

    segs_E = add_straight_segments!(all_segs, segs_N, east; length=block_length, speed_limit, lane_width, stop_inbound=true, stop_outbound=false)
    segs_E = add_curved_segments!(all_segs, segs_E, east, false; turn_curvature, speed_limit, lane_width)
    segs_E = add_pullout_segments!(all_segs, segs_E, south; length=shortened_block_length, pullout_length, pullout_taper, lane_width, speed_limit, pullout_inbound=true, pullout_outbound=false)
    segs_E = add_curved_segments!(all_segs, segs_E, south, false; turn_curvature, speed_limit, lane_width)
    segs_E = add_straight_segments!(all_segs, segs_E, west; length=block_length, speed_limit, lane_width, stop_outbound=true, stop_inbound=false)
    segs_I = add_segments!(all_segs, segs_E, west, segs_I)

    all_segs
end

function add_segments!(all_segs, base, direction, segs)
    foreach(id->all_segs[id].children=segs.origins[opposite(direction)], base.sinks[direction])
    foreach(id->all_segs[id].children=base.origins[direction], segs.sinks[opposite(direction)])
    segs
end

function add_curved_segments!(all_segs, base, direction, turn_left; turn_curvature=0.1, speed_limit=7.5, lane_width=5.0)
    
    if direction == west
        lane_dir = SVector(-1.0, 0)
    elseif direction == north
        lane_dir = SVector(0.0, 1.0)
    elseif direction == east
        lane_dir = SVector(1.0, 0.0)
    elseif direction == south
        lane_dir = SVector(0.0, -1.0)
    end
    right_dir = SVector(lane_dir[2], -lane_dir[1])

    if isnothing(base)
        pt_a = SVector(0.0, 0)
        pt_b = pt_a + right_dir * lane_width
        pt_c = pt_b + right_dir * lane_width
    else
        dir_sinks = base.sinks[direction]
        dir_origins = base.origins[direction]
        pt_a = (all_segs[first(dir_origins)].lane_boundaries[2]).pt_a
        pt_b = (all_segs[first(dir_origins)].lane_boundaries[1]).pt_a
        pt_c = (all_segs[first(dir_sinks)].lane_boundaries[2]).pt_b
    end
    inside_rad = 1.0 / turn_curvature
    middle_rad = inside_rad + lane_width
    outside_rad = middle_rad + lane_width
    if turn_left
        end_direction = mod(Int(direction) - 1, 4) |> Direction
        pt_d = pt_a + lane_dir * inside_rad - right_dir * inside_rad
        pt_e = pt_b + lane_dir * middle_rad - right_dir * middle_rad
        pt_f = pt_c + lane_dir * outside_rad - right_dir * outside_rad
    else
        end_direction = mod(Int(direction) + 1, 4) |> Direction
        pt_d = pt_a + lane_dir * outside_rad + right_dir * outside_rad
        pt_e = pt_b + lane_dir * middle_rad + right_dir * middle_rad
        pt_f = pt_c + lane_dir * inside_rad + right_dir * inside_rad
    end

    seg_id = maximum(keys(all_segs); init=0)

    b1 = lane_boundary(pt_b, pt_e, true, true, turn_left)
    b2 = lane_boundary(pt_c, pt_f, true, true, turn_left)
    seg_1 = RoadSegment(seg_id+=1, [b1, b2], [standard,], speed_limit, [])

    b1 = lane_boundary(pt_e, pt_b, true, true, !turn_left)
    b2 = lane_boundary(pt_d, pt_a, true, true, !turn_left)
    seg_2 = RoadSegment(seg_id+=1, [b1, b2], [standard,], speed_limit, [])

    if !isnothing(base)
        seg_2.children = base.origins[direction]
        foreach(id->all_segs[id].children = [seg_1.id,], base.sinks[direction])
    end
    all_segs[seg_1.id] = seg_1
    all_segs[seg_2.id] = seg_2
    
    sinks = Dict{Direction, Vector{Int}}()
    origins = Dict{Direction, Vector{Int}}()
    origins[end_direction] = [seg_2.id,]
    sinks[end_direction] = [seg_1.id,]

    (; sinks, origins)
end

function add_T_intersection!(all_segs, base, direction, T_direction; intersection_curvature=0.25, lane_width=5.0, speed_limit=7.5)
    segs = add_fourway_intersection!(all_segs, base, direction; intersection_curvature, lane_width, speed_limit)
    for id in segs.origins[opposite(T_direction)]
        delete!(all_segs, id)
        for (dir, seg_ids) in segs.sinks
            filter!(sid->sid!=id, seg_ids)
        end
    end
    for id in segs.sinks[opposite(T_direction)]
        delete!(all_segs, id)
        for (dir, seg_ids) in segs.origins
            filter!(sid->sid!=id, seg_ids)
        end
    end
    delete!(segs.origins, opposite(T_direction))
    delete!(segs.sinks, opposite(T_direction))
    segs
end

function add_pullout_segments!(all_segs, base, direction; length=40.0, pullout_length=20.0, pullout_taper=5.0, lane_width = 5.0, speed_limit=7.5, pullout_inbound=false, pullout_outbound=false, stop_outbound=false, stop_inbound=false)
    end_lengths = (length - pullout_length - 2*pullout_taper) / 2.0
    base = add_straight_segments!(all_segs, base, direction; length=end_lengths, speed_limit, lane_width, stop_outbound=false, stop_inbound)
    
    base = add_double_segments!(all_segs, base, direction; taper=1, length=pullout_taper, speed_limit, lane_width, pullout_inbound, pullout_outbound)
    base = add_double_segments!(all_segs, base, direction; taper=0, length=pullout_length, speed_limit, lane_width, pullout_inbound, pullout_outbound)
    base = add_double_segments!(all_segs, base, direction; taper=-1, length=pullout_taper, speed_limit, lane_width, pullout_inbound, pullout_outbound)
    
    base = add_straight_segments!(all_segs, base, direction; length=end_lengths, speed_limit, lane_width, stop_outbound, stop_inbound=false)
end

function add_double_segments!(all_segs, base, direction; taper=1, length=5.0, speed_limit=7.5, lane_width=5.0, pullout_inbound=false, pullout_outbound=false)
    if isnothing(base)
        @error "Unsupported"
    end

    if direction == west
        lane_dir = SVector(-1.0, 0)
    elseif direction == north
        lane_dir = SVector(0.0, 1.0)
    elseif direction == east
        lane_dir = SVector(1.0, 0.0)
    elseif direction == south
        lane_dir = SVector(0.0, -1.0)
    end
    right_dir = SVector(lane_dir[2], -lane_dir[1])

    if isnothing(base)
        pt_a = SVector(0.0, 0)
        pt_b = pt_a + right_dir * lane_width
        pt_c = pt_b + right_dir * lane_width
    else
        dir_sinks = base.sinks[direction]
        dir_origins = base.origins[direction]
        pt_a = (all_segs[first(dir_origins)].lane_boundaries[2]).pt_a
        pt_b = (all_segs[first(dir_origins)].lane_boundaries[1]).pt_a
        pt_c = (all_segs[first(dir_sinks)].lane_boundaries[2]).pt_b
    end

    pt_d = pt_a + lane_dir * length
    pt_e = pt_b + lane_dir * length
    pt_f = pt_c + lane_dir * length
    
    pt_a_l = pt_a - right_dir * lane_width
    pt_c_r = pt_c + right_dir * lane_width
    pt_d_l = pt_d - right_dir * lane_width
    pt_f_r = pt_f + right_dir * lane_width

    seg_id = maximum(keys(all_segs); init=0)

    b1 = LaneBoundary(pt_b, pt_e, 0.0, true, true)
    if pullout_outbound
        b2 = LaneBoundary(pt_c, pt_f, 0.0, false, true)
        if taper == 1
            b3 = LaneBoundary(pt_c, pt_f_r, 0.0, true, true)
        elseif taper == 0
            b3 = LaneBoundary(pt_c_r, pt_f_r, 0.0, true, true)
        elseif taper == -1
            b3 = LaneBoundary(pt_c_r, pt_f, 0.0, true, true)
        end
        outbound_types = [standard, loading_zone]
        seg_1 = RoadSegment(seg_id+=1, [b1,b2,b3], outbound_types, speed_limit, [])
    else
        b2 = LaneBoundary(pt_c, pt_f, 0.0, true, true)
        seg_1 = RoadSegment(seg_id+=1, [b1, b2], [standard,], speed_limit, [])
    end

    b1 = LaneBoundary(pt_e, pt_b, 0.0, true, true)
    if pullout_inbound
        b2 = LaneBoundary(pt_d, pt_a, 0.0, false, true)
        if taper == 1
            b3 = LaneBoundary(pt_d_l, pt_a, 0.0, true, true)
        elseif taper == 0
            b3 = LaneBoundary(pt_d_l, pt_a_l, 0.0, true, true)
        elseif taper == -1
            b3 = LaneBoundary(pt_d, pt_a_l, 0.0, true, true)
        end
        inbound_types = [standard, loading_zone]
        seg_2 = RoadSegment(seg_id+=1, [b1, b2, b3], inbound_types, speed_limit, [])
    else
        b2 = LaneBoundary(pt_d, pt_a, 0.0, true, true)
        seg_2 = RoadSegment(seg_id+=1, [b1, b2], [standard,], speed_limit, [])
    end
    
    if !isnothing(base)
        seg_2.children = base.origins[direction]
        foreach(id->all_segs[id].children = [seg_1.id,], base.sinks[direction])
    end
    all_segs[seg_1.id] = seg_1
    all_segs[seg_2.id] = seg_2
    
    sinks = Dict{Direction, Vector{Int}}()
    origins = Dict{Direction, Vector{Int}}()
    origins[direction] = [seg_2.id,]
    sinks[direction] = [seg_1.id,]

    (; sinks, origins)
end

function add_straight_segments!(all_segs, base, direction; length=40.0, speed_limit = 7.5, lane_width=5.0, stop_outbound=false, stop_inbound=false)
    if direction == west
        lane_dir = SVector(-1.0, 0)
    elseif direction == north
        lane_dir = SVector(0.0, 1.0)
    elseif direction == east
        lane_dir = SVector(1.0, 0.0)
    elseif direction == south
        lane_dir = SVector(0.0, -1.0)
    end
    right_dir = SVector(lane_dir[2], -lane_dir[1])

    if isnothing(base)
        pt_a = SVector(0.0, 0)
        pt_b = pt_a + right_dir * lane_width
        pt_c = pt_b + right_dir * lane_width
    else
        dir_sinks = base.sinks[direction]
        dir_origins = base.origins[direction]
        pt_a = (all_segs[first(dir_origins)].lane_boundaries[2]).pt_a
        pt_b = (all_segs[first(dir_origins)].lane_boundaries[1]).pt_a
        pt_c = (all_segs[first(dir_sinks)].lane_boundaries[2]).pt_b
    end

    pt_d = pt_a + lane_dir * length
    pt_e = pt_b + lane_dir * length
    pt_f = pt_c + lane_dir * length
    
    seg_id = maximum(keys(all_segs); init=0)

    b1 = LaneBoundary(pt_b, pt_e, 0.0, true, true)
    b2 = LaneBoundary(pt_c, pt_f, 0.0, true, true)
    outbound_type = stop_outbound ? stop_sign : standard
    seg_1 = RoadSegment(seg_id+=1, [b1, b2], [outbound_type,], speed_limit, [])

    b1 = LaneBoundary(pt_e, pt_b, 0.0, true, true)
    b2 = LaneBoundary(pt_d, pt_a, 0.0, true, true)
    inbound_type = stop_inbound ? stop_sign : standard
    seg_2 = RoadSegment(seg_id+=1, [b1, b2], [inbound_type], speed_limit, [])

    if !isnothing(base)
        seg_2.children = base.origins[direction]
        foreach(id->all_segs[id].children = [seg_1.id,], base.sinks[direction])
    end
    all_segs[seg_1.id] = seg_1
    all_segs[seg_2.id] = seg_2
    
    sinks = Dict{Direction, Vector{Int}}()
    origins = Dict{Direction, Vector{Int}}()
    origins[direction] = [seg_2.id,]
    sinks[direction] = [seg_1.id,]
    (; sinks, origins)
end

function add_fourway_intersection!(all_segs, base, direction; intersection_curvature = 0.25, lane_width = 5.0, speed_limit=7.5)
    lw = lane_width
    r = 1.0 / intersection_curvature
    if isnothing(base)
        offset = SVector(0.0, 0)
    else
        prev_sinks = base.sinks[direction]
        prev_origins = base.origins[direction]

        if direction == north
            offset = all_segs[first(prev_sinks)].lane_boundaries[1].pt_b + SVector(-r-lw, 0.0)
        elseif direction == east
            offset = all_segs[first(prev_sinks)].lane_boundaries[2].pt_b + SVector(0.0, -r)
        elseif direction == south
            offset = all_segs[first(prev_sinks)].lane_boundaries[2].pt_b + SVector(-r, -2*lw -2r)
        elseif direction == west
            offset = all_segs[first(prev_sinks)].lane_boundaries[1].pt_b + SVector(-2*lw - 2*r, -r-lw)
        end
    end
    pt_a = offset+SVector(r, 0.0)
    pt_b = offset+SVector(r+lw, 0.0)
    pt_c = offset+SVector(r+2*lw, 0.0)

    pt_d = offset+SVector(2*lw+2r, r)
    pt_e = offset+SVector(2*lw+2r, lw+r)
    pt_f = offset+SVector(2*lw+2r, 2*lw+r)

    pt_g = offset+SVector(r+2*lw, 2*lw+2r)
    pt_h = offset+SVector(r+lw, 2*lw+2r)
    pt_i = offset+SVector(r, 2*lw+2r)

    pt_j = offset+SVector(0.0, 2*lw+r)
    pt_k = offset+SVector(0.0, lw+r)
    pt_l = offset+SVector(0.0, r)

    seg_id = maximum(keys(all_segs); init=0)

    # South origins
    b1 = lane_boundary(pt_b, pt_k, true, false, true)
    b2 = lane_boundary(pt_c, pt_j, true, false, true)
    seg_1 = RoadSegment(seg_id+=1, [b1, b2], [intersection,], speed_limit, [])

    b1 = lane_boundary(pt_b, pt_h, true, false) 
    b2 = lane_boundary(pt_c, pt_g, true, false) 
    seg_2 = RoadSegment(seg_id+=1, [b1, b2], [intersection,], speed_limit, [])

    b1 = lane_boundary(pt_b, pt_e, true, false, false) 
    b2 = lane_boundary(pt_c, pt_d, true, true, false) 
    seg_3 = RoadSegment(seg_id+=1, [b1, b2], [intersection,], speed_limit, [])
   
    # East origins
    b1 = lane_boundary(pt_e, pt_b, true, false, true)
    b2 = lane_boundary(pt_f, pt_a, true, false, true)
    seg_4 = RoadSegment(seg_id+=1, [b1, b2], [intersection,], speed_limit, [])

    b1 = lane_boundary(pt_e, pt_k, true, false) 
    b2 = lane_boundary(pt_f, pt_j, true, false) 
    seg_5 = RoadSegment(seg_id+=1, [b1, b2], [intersection,], speed_limit, [])

    b1 = lane_boundary(pt_e, pt_h, true, false, false) 
    b2 = lane_boundary(pt_f, pt_g, true, true, false) 
    seg_6 = RoadSegment(seg_id+=1, [b1, b2], [intersection,], speed_limit, [])
    
    # North origins
    b1 = lane_boundary(pt_h, pt_e, true, false, true)
    b2 = lane_boundary(pt_i, pt_d, true, false, true)
    seg_7 = RoadSegment(seg_id+=1, [b1, b2], [intersection,], speed_limit, [])

    b1 = lane_boundary(pt_h, pt_b, true, false) 
    b2 = lane_boundary(pt_i, pt_a, true, false) 
    seg_8 = RoadSegment(seg_id+=1, [b1, b2], [intersection,], speed_limit, [])

    b1 = lane_boundary(pt_h, pt_k, true, false, false) 
    b2 = lane_boundary(pt_i, pt_j, true, true, false) 
    seg_9 = RoadSegment(seg_id+=1, [b1, b2], [intersection,], speed_limit, [])
    
    # West origins
    b1 = lane_boundary(pt_k, pt_h, true, false, true)
    b2 = lane_boundary(pt_l, pt_g, true, false, true)
    seg_10 = RoadSegment(seg_id+=1, [b1, b2], [intersection,], speed_limit, [])

    b1 = lane_boundary(pt_k, pt_e, true, false) 
    b2 = lane_boundary(pt_l, pt_d, true, false) 
    seg_11 = RoadSegment(seg_id+=1, [b1, b2], [intersection,], speed_limit, [])

    b1 = lane_boundary(pt_k, pt_b, true, false, false) 
    b2 = lane_boundary(pt_l, pt_a, true, true, false) 
    seg_12 = RoadSegment(seg_id+=1, [b1, b2], [intersection,], speed_limit, [])
    
    foreach(seg->all_segs[seg.id]=seg, [seg_1, seg_2, seg_3, seg_4, seg_5, seg_6, seg_7, seg_8, seg_9, seg_10, seg_11, seg_12])

    sinks = Dict{Direction, Vector{Int}}()
    origins = Dict{Direction, Vector{Int}}()
    origins[south] = [seg_1.id, seg_2.id, seg_3.id]
    sinks[south] = [seg_4.id, seg_8.id, seg_12.id]
    origins[east] = [seg_4.id, seg_5.id, seg_6.id]
    sinks[east] = [seg_3.id, seg_7.id, seg_11.id]
    origins[north] = [seg_7.id, seg_8.id, seg_9.id]
    sinks[north] = [seg_2.id, seg_6.id, seg_10.id]
    origins[west] = [seg_10.id, seg_11.id, seg_12.id]
    sinks[west] = [seg_1.id, seg_5.id, seg_9.id]


    if !isnothing(base)
        foreach(id->all_segs[id].children = origins[opposite(direction)], base.sinks[direction])
        foreach(id->all_segs[id].children = base.origins[direction], sinks[opposite(direction)])
    end

    (; sinks, origins)
end
