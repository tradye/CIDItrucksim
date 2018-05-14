function steer_angle_feedforwardterm = computeFeedForward(ref_curvature)
    kv =  lr_ * mass_ / 2 / cf_ / wheelbase_ - lf_ * mass_ / 2 / cr_ / wheelbase_;

    steer_angle_feedforwardterm =
      (wheelbase_ * ref_curvature + kv * v * v * ref_curvature -
       matrix_k_(0, 2) *
           (lr_ * ref_curvature -
            lf_ * mass_ * v * v * ref_curvature / 2 / cr_ / wheelbase_)) *
      180 / M_PI * steer_transmission_ratio_ /
      steer_single_direction_max_degree_ * 100;



end