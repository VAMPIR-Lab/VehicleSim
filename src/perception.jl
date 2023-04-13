# The entire perception.jl file is working on ONE bounding box (i.e. only four corners of z is given).
# So, if there are two bounding boxes coming in per vehicle, the ekf function must be called twcie in
# example_project.jl's perception() function.

function perception_f(x, delta_t)
    # update xk = [p1 p2 theta vel l w h]
    # - updated-p1 = p1 + delta_time *cos(theta)*v
    # - updated-p2 = p2 + delta_time*sin(theta)*v
    theta_k = x[3]
    vel_k = x[4]
    x + delta_t * [vel_k * cos(theta_k), vel_k * sin(theta_k), 0, 0, 0, 0, 0]
end

function perception_jac_fx(x, delta_t)
    theta = x[3]
    vel = x[4]

    [1 0 (-sin(theta)*vel*delta_t) (delta_t*cos(theta)) 0 0 0
        0 1 (cos(theta)*vel*delta_t) (delta_t*sin(theta)) 0 0 0
        0 0 1 0 0 0 0
        0 0 0 1 0 0 0
        0 0 0 0 1 0 0
        0 0 0 0 0 1 0
        0 0 0 0 0 0 1
    ]

end

function perception_get_3d_bbox_corners(x, box_size)
    theta = x[3]
    # referenced L20 pg.4 as well
    R = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1]
    xyz = [x[1] x[2] box_size[3] / 2]
    T = [R xyz]
    corners = []

    for dx in [-box_size[1] / 2, box_size[1] / 2]
        for dy in [-box_size[2] / 2, box_size[2] / 2]
            for dz in [-box_size[3] / 2, box_size[3] / 2]
                push!(corners, T * [dx, dy, dz, 1])
            end
        end
    end
    corners
end

function convert_to_pixel_unrounded(num_pixels, pixel_len, px)
    min_val = -pixel_len * num_pixels / 2
    pix_id = (px - min_val) / pixel_len
    return pix_id
end

"""
    Usage:
        - should be called per camera

    Parameters:
        - x_other: state of a recognized car (in [p1 p2 theta vel l w h])
        - x_ego: state of ego car (in a format that localization gives)
"""
function perception_h(x_other, x_ego, cam_id)
    # constant variables
    vehicle_size = SVector(13.2, 5.7, 5.3)
    focal_len = 0.01
    pixel_len = 0.001
    image_width = 640
    image_height = 480
    # num_vehicles = length(x_other)

    corners_body = [perception_get_3d_bbox_corners(x_other, vehicle_size)]
    @debug corners_body

    # Section 1: Get transformation matrices
    T_body_cam1 = get_cam_transform(1)
    T_body_cam2 = get_cam_transform(2)
    T_cam_camrot = get_rotated_camera_transform()

    T_body_camrot1 = multiply_transforms(T_body_cam1, T_cam_camrot)
    T_body_camrot2 = multiply_transforms(T_body_cam2, T_cam_camrot)

    # make sure the ego state you get from localization team follows the same format
    T_world_body = get_body_transform(x_ego.q[1:4], x_ego.q[5:7]) # get_body_transform(quat, loc)
    T_world_camrot1 = multiply_transforms(T_world_body, T_body_camrot1)
    T_world_camrot2 = multiply_transforms(T_world_body, T_body_camrot2)

    transform = invert_transform(T_world_camrot1) # defulat to camera 1
    if cam_id == 2
        transform = invert_transform(T_world_camrot2)
    end

    # Section 2: Calculate the bounding boxes
    bbox = []
    # bbox_unrounded are used to calculate the jacobian of h in float to be more precise
    bbox_unrounded = []
    corners = []
    # NOTE: deal with having only 1 or 2 boxes here
    # similar to x_carrot = R * [q1 q2 q3] + t, turn points rotated cam frame
    corners_of_other_vehicle = [transform * [pt; 1] for pt in corners_body[j]]

    left = image_width / 2
    right = -image_width / 2
    top = image_height / 2
    bot = -image_height / 2

    # we are basically getting through each corner values in camera frame and 
    # keep updating the left, top, bottom, right values!
    for corner in corners_of_other_vehicle
        # every point of corner in camera frame now
        if corner[3] < focal_len
            break
        end
        px = focal_len * corner[1] / corner[3]
        py = focal_len * corner[2] / corner[3]
        left = min(left, px)
        right = max(right, px)
        top = min(top, py)
        bot = max(bot, py)
    end

    if top ≈ bot || left ≈ right || top > bot || left > right
        # out of frame - return empty bbox
        return bbox
    else
        # update corners
        push!(corners, top)
        push!(corners, bot)
        push!(corners, left)
        push!(corners, right)

        # update bbox
        top = convert_to_pixel(image_height, pixel_len, top)
        bot = convert_to_pixel(image_height, pixel_len, bot)
        left = convert_to_pixel(image_width, pixel_len, left)
        right = convert_to_pixel(image_width, pixel_len, right)
        push!(bbox, SVector(top, left, bot, right))

        # update bbox_unrounded
        top_ur = convert_to_pixel_unrounded(image_height, pixel_len, top)
        bot_ur = convert_to_pixel_unrounded(image_height, pixel_len, bot)
        left_ur = convert_to_pixel_unrounded(image_height, pixel_len, left)
        right_ur = convert_to_pixel_unrounded(image_height, pixel_len, right)
        push!(bbox_unrounded, SVector(top_ur, left_ur, bot_ur, right_ur))
    end
    return bbox, bbox_unrounded, corners
end

function perception_jac_hx(zk, x_other, x_ego, cam_id)
    top = zk[1]
    left = zk[2]
    bot = zk[3]
    right = zk[4]

    # Calculate J1


    # Calculate J2 -- confirmed it's correct
    T_body_cam1 = get_cam_transform(1)
    T_body_cam2 = get_cam_transform(2)
    T_cam_camrot = get_rotated_camera_transform()

    T_body_camrot1 = multiply_transforms(T_body_cam1, T_cam_camrot)
    T_body_camrot2 = multiply_transforms(T_body_cam2, T_cam_camrot)

    # make sure the ego state you get from localization team follows the same format
    T_world_body = get_body_transform(x_ego.q[1:4], x_ego.q[5:7]) # get_body_transform(quat, loc)
    T_world_camrot1 = multiply_transforms(T_world_body, T_body_camrot1)
    T_world_camrot2 = multiply_transforms(T_world_body, T_body_camrot2)

    J2 = invert_transform(T_world_camrot1) # defulat to camera 1
    if cam_id == 2
        J2 = invert_transform(T_world_camrot2)
    end

    # Calculate J3


    # Calculate J4 -- confirmed it's correct
    pixel_len = 0.001
    s = 1 / pixel_len
    J4 = [s 0; 0 s]
end


"""
    Usage:
        - should be called per camera -- per vehicle is addressed in example_project.jl file
    Variables:
        - x = state of the other car = [p1 p2 theta vel l w h] (7 x 1)
        - P = covariance of the state
        - z = measurement = [y1a y2a y1b y2b] (4 x 1)
        - Q = process noise
        - R = measurement noise
"""
function perception_ekf(xego, delta_t, cam_id)
    # constant noise variables
    covariance_p = Diagonal([1^2, 1^2, 0.2^2, 0.4^2, 0.005^2, 0.003^2, 0.001^2])  # covariance for process model
    covariance_z = Diagonal([1^2, 1^2, 1^2, 1^2]) # covariance for measurement model
    num_steps = 25

    # initial states -- *change based on your state attributes
    x0 = [xego[1] + 2, xego[2] + 2, 0, xego[4], 8, 5, 5] # [p1 p2 theta vel l w h]
    # mean value of xk state of the OTHER car
    mu = zeros(7)
    sigma = Diagonal([1^2, 1^2, 0.2^2, 0.4^2, 0.005^2, 0.003^2, 0.001^2])

    # variables to keep updating
    # timesteps = []
    mus = [mu,] # the means
    sigmas = Matrix{Float64}[sigma,] # list of sigma_k's
    zs = Vector{Float64}[] # measurements of other car(s)

    x_prev = x0
    for k = 1:num_steps # for k = 1:something
        xk = perception_f(x_prev, delta_t)
        x_prev = xk
        zk, bbox_unrounded, corners = perception_h(xk, xego, cam_id)

        # *All of the equations below are referenced from L17 pg.3 and HW4
        # Process model: P(xk | xk-1, bbxk) = N(A*x-1, sig_carrot))
        # - A = perception_jac_fx(x_prev, delta_t)
        # - sig_carrot = convariance_p + A * sigmas[k] * A'
        # - mu_carrot = perception_f(u[k], delta_t)
        A = perception_jac_fx(mus[k], delta_t)
        sig_carrot = covariance_p + A * sigmas[k] * A'
        mu_carrot = perception_f(mus[k], delta_t)

        # Measurement model
        C = perception_jac_hx(zk, mu_carrot, xego, cam_id)
        sigma_k = inv(inv(sig_carrot) + C' * inv(covariance_z) * C)
        mu_k = sigma_k * (inv(sigma_carrot) * mu_carrot + C' * inv(covariance_z) * zk)

        # update the variables
        push!(mus, mu_k)
        push!(sigmas, sigma_k)
        push!(zs, zk)
        # push!(gt_states, xk)
        # push!(timesteps, delta_t)
    end
end
