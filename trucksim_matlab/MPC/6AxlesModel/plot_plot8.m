function plot_plot8(truck_x, ...
                    truck_y, ...
                    truck_ref_x, ...
                    truck_ref_y, ...
                    truck_lateral_err, ...
                    truck_heading_err, ...
                    trailer_x, ...
                    trailer_y, ...
                    trailer_ref_x, ...
                    trailer_ref_y, ...
                    trailer_lateral_err, ...
                    trailer_heading_err, ...
                    steer_angle,...
                    num ...
                    )
%     global t_sim;
    
    T = [];
%     for i = 1:1:(t_sim *100 + 1)
    for i = 1:1: num
        T(i) = 0.01 *i;
    end        
    
    figure(1)
    plot(truck_ref_x,truck_ref_y,'r');
    hold on
    plot(truck_x,truck_y,'b');
    xlabel('Truck X(m)');
    ylabel('Truck Y(m)');
    hold on
    
    figure(2)
    subplot(2,1,1);    
    plot(T,truck_lateral_err,'g');
    xlabel('T(s)');
    ylabel('Truck lateral error (m)');
    hold on
    subplot(2,1,2);    
    plot(T,truck_heading_err,'r');
    xlabel('T(s)');
    ylabel('Truck heading error (rad)');
    hold on
    
    figure(3)
    plot(trailer_ref_x,trailer_ref_y,'r');
    hold on
    plot(trailer_x,trailer_y,'b');
    xlabel('Trailer X(m)');
    ylabel('Trailer Y(m)');
    hold on
    
    figure(4)
    subplot(2,1,1);    
    plot(T,trailer_lateral_err,'g');
    xlabel('T(s)');
    ylabel('Trailer lateral error(m)');
    hold on
    subplot(2,1,2);    
    plot(T,trailer_heading_err,'r');
    xlabel('T(s)');
    ylabel('Trailer heading error(rad)');
    hold on
    figure(5)
    plot(T,steer_angle,'r');
    xlabel('T(s)');
    ylabel('Steering angle(deg)');
    hold on


end