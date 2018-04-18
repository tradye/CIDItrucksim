function WriteAndPlot(truck_state)
% truck_state is a 1*15 matrix, which including all the information necessary of  truck and trailer
    persistent time_sequence;
    persistent path_index;
    persistent path_x;
    persistent path_y;
    persistent path_heading;
    persistent steer_angle_;
    persistent truck_x_;
    persistent truck_y_;
    persistent truck_heading_;
    persistent truck_lateral_err;
    persistent truck_heading_err;
    persistent trailer_x_;
    persistent trailer_y_;
    persistent trailer_heading_;
    persistent trailer_heading_rate;
    global num_i;
    
    time_sequence(num_i,1) = truck_state(1);
    path_index(num_i,1) = truck_state(2);
    path_x(num_i,1) = truck_state(3);
    path_y(num_i,1) = truck_state(4);
    path_heading(num_i,1) = truck_state(5);
    steer_angle_(num_i,1) = truck_state(6);
    truck_x_(num_i,1) = truck_state(7);
    truck_y_(num_i,1) = truck_state(8);
    truck_heading_(num_i,1) = truck_state(9);
    truck_lateral_err(num_i,1) = truck_state(10);
    truck_heading_err(num_i,1) = truck_state(11);
    trailer_x_(num_i,1) = truck_state(12);
    trailer_y_(num_i,1) = truck_state(13);
    trailer_heading_(num_i,1) = truck_state(14);
    trailer_heading_rate(num_i,1) = truck_state(15);
    
    plot_plot(truck_x_,truck_y_,path_x,path_y,truck_lateral_err,truck_heading_err);%plot_plot(truck_x,truck_y,ref_x,ref_y,lateral_err,heading_err)
    
    T = table(time_sequence,path_index, path_x, path_y, path_heading, steer_angle_,...
                   truck_x_, truck_y_, truck_heading_, truck_lateral_err, truck_heading_err ,...
                   trailer_x_ ,trailer_y_ , trailer_heading_ ,  trailer_heading_rate);
        writetable(T,'truck_state_.csv');

end