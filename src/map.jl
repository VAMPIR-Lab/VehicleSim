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

function lane_boundary(pt_a, pt_b, hard, vis)
    dx = abs(pt_a[1] - pt_b[1])
    dy = abs(pt_a[2] - pt_b[2])
    if isapprox(dx - dy, 0; atol=1e-6)
        LaneBoundary(pt_a, pt_b, 1.0 / dx, hard, vis)
    else
        LaneBoundary(pt_a, pt_b, 0.0, hard, vis)
    end
end

"""
lane_boundaries are in order from left to right
"""
mutable struct RoadSegment
    lane_boundaries::Vector{LaneBoundary}
    lane_types::Vector{LaneTypes}
    speed_limit::Float64
    children::Vector{RoadSegment}
end

function training_map(; lane_width = 5.0,
                        speed_limit = 7.5,
                        pullout_length = 20.0,
                        pullout_taper = 5.0,
                        block_length = 40.0,
                        turn_curvature = 0.2,
                        intersection_curvature = 0.25)
    turn_r = 1.0/turn_curvature
    int_r = 1.0/intersection_curvature

    shortened_block_length = block_length - 2*(turn_r-int_r)
    single_shortened_block_length = block_length - (turn_r-int_r)

    segs_I = add_fourway_intersection!(nothing, nothing; intersection_curvature, speed_limit, lane_width)
    segs = add_straight_segments!(segs_I, west; length=block_length, speed_limit, stop_outbound=true, stop_inbound=true)
    @infiltrate
    segs_T = add_T_intersection!(segs, west, east; intersection_curvature, lane_width, speed_limit)
    segs_S = add_pullout_segments!(segs_T, south; length=block_length, pullout_length, pullout_taper, lane_width, speed_limit, pullout_inbound=false, pullout_outbound=true)
    segs_S = add_curved_segments!(segs_S, south, true; turn_curvature, speed_limit, lane_width)
    segs_S = add_pullout_segments!(segs_S, east; length=shortened_block_length, pullout_length, pullout_taper, lane_width, speed_limit, pullout_inbound=true, pullout_outbound=false)
    segs_S = add_curved_segments!(segs_S, east, true; turn_curvature, speed_limit, lane_width)
    segs_S = add_straight_segments!(segs_S, north; length=block_length, speed_limit, stop_outbound=true, stop_inbound=false)
    segs_I = add_segments!(segs_S, north, segs_I)

    segs_N = add_pullout_segments!(segs_T, north; length=block_length, pullout_length, pullout_taper, lane_width, speed_limit, pullout_inbound=true, pullout_outbound=false)
    segs_N = add_curved_segments!(segs_N, north, false; turn_curvature, speed_limit, lane_width)
    segs_N = add_straight_segments!(segs_N, east; length=single_shortened_block_length, speed_limit, stop_outbound=true, stop_inbound=false)
    segs_N = add_T_intersection!(segs_N, east, south; intersection_curvature, lane_width, speed_limit)
    segs_N2 = add_pullout_segments!(segs_N, south; length=block_length, pullout_length, pullout_taper, lane_width, speed_limit, pullout_inbound=true, pullout_outbound=true, stop_inbound=true, stop_outbound=true)
    segs_I = add_segments!(segs_N2, south, segs_I)

    segs_E = add_straight_segments!(segs_N, east; length=block_length, speed_limit, lane_width, stop_inbound=true, stop_outbound=false)
    segs_E = add_curved_segments!(segs_E, east, false; turn_curvature, speed_limit, lane_width)
    segs_E = add_pullout_segments!(segs_E, south; length=shortened_block_length, pullout_length, pullout_taper, lane_width, speed_limit, pullout_inbound=true, pullout_outbound=false)
    segs_E = add_curved_segments!(segs_E, south, false; turn_curvature, speed_limit, lane_width)
    segs_E = add_straight_segments!(segs_E, west; length=block_length, speed_limit, lane_width, stop_outbound=true, stop_inbound=false)
    segs_I = add_segments!(segs_E, west, segs_I)
end

function add_segments!(base, direction, segs)
    foreach(s->s.children=segs.origins[opposite(direction)], base.sinks[direction])
    foreach(s->s.children=base.origins[direction], segs.sinks[opposite(direction)])
    segs
end

function add_curved_segments!(base, direction, turn_left; turn_curvature=0.1, speed_limit=7.5, lane_width=5.0)
    
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
        pt_a = ((dir_origins |> first).lane_boundaries[2]).pt_a
        pt_b = ((dir_origins |> first).lane_boundaries[1]).pt_a
        pt_c = ((dir_sinks |> first).lane_boundaries[2]).pt_b
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

    b1 = lane_boundary(pt_b, pt_e, true, true)
    b2 = lane_boundary(pt_c, pt_f, true, true)
    seg_1 = RoadSegment([b1, b2], [standard,], speed_limit, Vector{RoadSegment}())

    b1 = lane_boundary(pt_e, pt_b, true, true)
    b2 = lane_boundary(pt_d, pt_a, true, true)
    seg_2 = RoadSegment([b1, b2], [standard,], speed_limit, Vector{RoadSegment}())

    if !isnothing(base)
        seg_2.children = base.origins[direction]
    end
    
    sinks = Dict{Direction, Vector{RoadSegment}}()
    origins = Dict{Direction, Vector{RoadSegment}}()

    origins[end_direction] = [seg_2,]
    sinks[end_direction] = [seg_1,]
    (; sinks, origins)
end

function add_T_intersection!(base, direction, T_direction; intersection_curvature=0.25, lane_width=5.0, speed_limit=7.5)
    segs = add_fourway_intersection!(base, direction; intersection_curvature, lane_width, speed_limit)
    for seg in segs.origins[opposite(T_direction)]
        for (dir, ssegs) in segs.sinks
            dir == opposite(T_direction) && continue
            filter!(s->s!=seg, ssegs)
        end
    end
    for seg in segs.sinks[opposite(T_direction)]
        for (dir, osegs) in segs.origins
            dir == opposite(T_direction) && continue
            filter!(s->s!=seg, osegs)
        end
    end
    delete!(segs.origins, opposite(T_direction))
    delete!(segs.sinks, opposite(T_direction))
    segs
end

function add_pullout_segments!(base, direction; length=40.0, pullout_length=20.0, pullout_taper=5.0, lane_width = 5.0, speed_limit=7.5, pullout_inbound=false, pullout_outbound=false, stop_outbound=false, stop_inbound=false)
    end_lengths = (length - pullout_length - 2*pullout_taper) / 2.0
    base = add_straight_segments!(base, direction; length=end_lengths, speed_limit, lane_width, stop_outbound=false, stop_inbound)
    
    base = add_double_segments!(base, direction; taper=1, length=pullout_taper, speed_limit, lane_width, pullout_inbound, pullout_outbound)
    base = add_double_segments!(base, direction; taper=0, length=pullout_length, speed_limit, lane_width, pullout_inbound, pullout_outbound)
    base = add_double_segments!(base, direction; taper=-1, length=pullout_taper, speed_limit, lane_width, pullout_inbound, pullout_outbound)
    
    base = add_straight_segments!(base, direction; length=end_lengths, speed_limit, lane_width, stop_outbound, stop_inbound=false)
end

function add_double_segments!(base, direction; taper=1, length=5.0, speed_limit=7.5, lane_width=5.0, pullout_inbound=false, pullout_outbound=false)
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
        pt_a = ((dir_origins |> first).lane_boundaries[2]).pt_a
        pt_b = ((dir_origins |> first).lane_boundaries[1]).pt_a
        pt_c = ((dir_sinks |> first).lane_boundaries[2]).pt_b
    end

    pt_d = pt_a + lane_dir * length
    pt_e = pt_b + lane_dir * length
    pt_f = pt_c + lane_dir * length
    
    pt_a_l = pt_a - right_dir * lane_width
    pt_c_r = pt_c + right_dir * lane_width
    pt_d_l = pt_d - right_dir * lane_width
    pt_f_r = pt_f + right_dir * lane_width

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
        seg_1 = RoadSegment([b1,b2,b3], outbound_types, speed_limit, Vector{RoadSegment}())
    else
        b2 = LaneBoundary(pt_c, pt_f, 0.0, true, true)
        seg_1 = RoadSegment([b1, b2], [standard,], speed_limit, Vector{RoadSegment}())
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
        seg_2 = RoadSegment([b1, b2, b3], inbound_types, speed_limit, Vector{RoadSegment}())
    else
        b2 = LaneBoundary(pt_d, pt_a, 0.0, true, true)
        seg_2 = RoadSegment([b1, b2, b3], [standard,], speed_limit, Vector{RoadSegment}())
    end
    
    if !isnothing(base)
        seg_2.children = base.origins[direction]
    end
    
    sinks = Dict{Direction, Vector{RoadSegment}}()
    origins = Dict{Direction, Vector{RoadSegment}}()
    origins[direction] = [seg_2,]
    sinks[direction] = [seg_1,]
    (; sinks, origins)
end

function add_straight_segments!(base, direction; length=40.0, speed_limit = 7.5, lane_width=5.0, stop_outbound=false, stop_inbound=false)
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
        pt_a = ((dir_origins |> first).lane_boundaries[2]).pt_a
        pt_b = ((dir_origins |> first).lane_boundaries[1]).pt_a
        pt_c = ((dir_sinks |> first).lane_boundaries[2]).pt_b
    end

    pt_d = pt_a + lane_dir * length
    pt_e = pt_b + lane_dir * length
    pt_f = pt_c + lane_dir * length

    b1 = LaneBoundary(pt_b, pt_e, 0.0, true, true)
    b2 = LaneBoundary(pt_c, pt_f, 0.0, true, true)
    outbound_type = stop_outbound ? stop_sign : standard
    seg_1 = RoadSegment([b1, b2], [outbound_type,], speed_limit, Vector{RoadSegment}())

    b1 = LaneBoundary(pt_e, pt_b, 0.0, true, true)
    b2 = LaneBoundary(pt_d, pt_a, 0.0, true, true)
    inbound_type = stop_inbound ? stop_sign : standard
    seg_2 = RoadSegment([b1, b2], [inbound_type], speed_limit, Vector{RoadSegment}())

    if !isnothing(base)
        seg_2.children = base.origins[direction]
    end
    
    sinks = Dict{Direction, Vector{RoadSegment}}()
    origins = Dict{Direction, Vector{RoadSegment}}()
    origins[direction] = [seg_2,]
    sinks[direction] = [seg_1,]
    (; sinks, origins)
end

function add_fourway_intersection!(base, direction; intersection_curvature = 0.25, lane_width = 5.0, speed_limit=7.5)
    lw = lane_width
    r = 1.0 / intersection_curvature
    if isnothing(base)
        offset = SVector(0.0, 0)
    else
        origins = base.sinks[direction]
        sinks = base.origins[direction]

        if direction == north
            offset = (sinks |> first).lane_boundaries[2].pt_b + SVector(-r, 0.0)
        elseif direction == east
            offset = (origins |> first).lane_boundaries[2].pt_a + SVector(0.0, -r)
        elseif direction == south
            offset = (origins |> first).lane_boundaries[2].pt_a + SVector(-r, -2*lw -2r)
        elseif direction == west
            offset = (sinks |> first).lane_boundaries[2].pt_a + SVector(-2*lw - 2*r, -r)
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

    # South origins
    b1 = lane_boundary(pt_b, pt_k, true, false)
    b2 = lane_boundary(pt_c, pt_j, true, false)
    seg_1 = RoadSegment([b1, b2], [intersection,], speed_limit, Vector{RoadSegment}())

    b1 = lane_boundary(pt_b, pt_h, true, false) 
    b2 = lane_boundary(pt_c, pt_g, true, false) 
    seg_2 = RoadSegment([b1, b2], [intersection,], speed_limit, Vector{RoadSegment}())

    b1 = lane_boundary(pt_b, pt_e, true, false) 
    b2 = lane_boundary(pt_c, pt_d, true, true) 
    seg_3 = RoadSegment([b1, b2], [intersection,], speed_limit, Vector{RoadSegment}())
   
    # East origins
    b1 = lane_boundary(pt_e, pt_b, true, false)
    b2 = lane_boundary(pt_f, pt_a, true, false)
    seg_4 = RoadSegment([b1, b2], [intersection,], speed_limit, Vector{RoadSegment}())

    b1 = lane_boundary(pt_e, pt_k, true, false) 
    b2 = lane_boundary(pt_f, pt_j, true, false) 
    seg_5 = RoadSegment([b1, b2], [intersection,], speed_limit, Vector{RoadSegment}())

    b1 = lane_boundary(pt_e, pt_h, true, false) 
    b2 = lane_boundary(pt_f, pt_g, true, true) 
    seg_6 = RoadSegment([b1, b2], [intersection,], speed_limit, Vector{RoadSegment}())
    
    # North origins
    b1 = lane_boundary(pt_h, pt_d, true, false)
    b2 = lane_boundary(pt_i, pt_e, true, false)
    seg_7 = RoadSegment([b1, b2], [intersection,], speed_limit, Vector{RoadSegment}())

    b1 = lane_boundary(pt_h, pt_b, true, false) 
    b2 = lane_boundary(pt_i, pt_a, true, false) 
    seg_8 = RoadSegment([b1, b2], [intersection,], speed_limit, Vector{RoadSegment}())

    b1 = lane_boundary(pt_h, pt_k, true, false) 
    b2 = lane_boundary(pt_i, pt_j, true, true) 
    seg_9 = RoadSegment([b1, b2], [intersection,], speed_limit, Vector{RoadSegment}())
    
    # West origins
    b1 = lane_boundary(pt_k, pt_h, true, false)
    b2 = lane_boundary(pt_l, pt_g, true, false)
    seg_10 = RoadSegment([b1, b2], [intersection,], speed_limit, Vector{RoadSegment}())

    b1 = lane_boundary(pt_k, pt_e, true, false) 
    b2 = lane_boundary(pt_l, pt_d, true, false) 
    seg_11 = RoadSegment([b1, b2], [intersection,], speed_limit, Vector{RoadSegment}())

    b1 = lane_boundary(pt_k, pt_b, true, false) 
    b2 = lane_boundary(pt_l, pt_a, true, true) 
    seg_12 = RoadSegment([b1, b2], [intersection,], speed_limit, Vector{RoadSegment}())
    
    sinks = Dict{Direction, Vector{RoadSegment}}()
    origins = Dict{Direction, Vector{RoadSegment}}()
    origins[south] = [seg_1, seg_2, seg_3]
    sinks[south] = [seg_4, seg_8, seg_12]
    origins[east] = [seg_4, seg_5, seg_6]
    sinks[east] = [seg_3, seg_7, seg_11]
    origins[north] = [seg_7, seg_8, seg_9]
    sinks[north] = [seg_2, seg_6, seg_10]
    origins[west] = [seg_10, seg_11, seg_12]
    sinks[west] = [seg_1, seg_5, seg_9]

    if !isnothing(base)
        foreach(s->s.children = origins[opposite(direction)], base.sinks[direction])
        foreach(s->s.children = base.origins[direction], sinks[opposite(direction)])
    end

    (; sinks, origins)
end
