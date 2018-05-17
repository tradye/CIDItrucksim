function [x_com,y_com]=ComputeCOMPosition(rear_to_com_distance,has_orientation,x_,y_,z_)
    v = zeros(1,3);
    pos_vec = zeros(1,3);
    v(1,2) = rear_to_com_distance;
    pos_vec(1,1) = x_;
    pos_vec(1,2) = y_;
    pos_vec(1,3) = z_;
    
    if (has_orientation)
        %% 如果此处has_orientation 为真，则把orientation用四元数
        % 转换成欧拉角信息，具体见apollo math::Vec2d VehicleStateProvider::ComputeCOMPosition
        x_com = pos_vec(1,1) + v(1,1);
        y_com = pos_vec(1,2) + v(1,2);
    else
        x_com = pos_vec(1,1) + v(1,1);
        y_com = pos_vec(1,2) +v(1,2); %%  trucksim中不是质心为原点，而是以前轮或者后轮轴中心为原点
                                                 %     但是apollo是以后轮轴中心为坐标原点的，但是计算误差时是以质心位置为参考算的
    end



end