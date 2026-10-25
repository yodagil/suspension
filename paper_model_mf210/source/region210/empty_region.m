function R = empty_region()

    R = struct( 'limit_roll_points',zeros(0,3), 'outer_limit_points',zeros(0,3), 'inner_limit_points',zeros(0,3), 'outer_vertices',zeros(0,3), 'inner_vertices',zeros(0,3), 'outer_volume',NaN, 'inner_volume',NaN, 'unstable_inside_outer',NaN, 'unstable_inside_inner',NaN, 'inner_depth',NaN, 'is_valid_inner',false, 'vc_split_center',NaN, 'used_stable_mask_fallback',false, 'used_support_fallback',false);

end

