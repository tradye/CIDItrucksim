function plot_plot(truck_x,truck_y,ref_x,ref_y,lateral_err,heading_err)
    global t_sim;
    figure(1)
    plot(ref_x,ref_y,'r');
    hold on
    plot(truck_x,truck_y,'b');
    xlabel('X(m)');
    ylabel('Y(m)');
    hold on
    
    T = [];
    for i = 1:1:(t_sim *100 + 1)
        T(i) = 0.01 *i;
    end
    figure(2)
    subplot(2,1,1);
    % for i = 1:1:3000
    %     plot(T,Lateral_ERR);
    % end
    plot(T,lateral_err,'g');
    xlabel('采样时间 T(s)');
    ylabel('lateral_error(m)');

    subplot(2,1,2);
    % for i = 1:1:3000
    %     plot(T,Heading_ERR);
    % end
    plot(T,heading_err,'r');
    xlabel('采样时间 T(s)');
    ylabel('heading_error(rad)');




end