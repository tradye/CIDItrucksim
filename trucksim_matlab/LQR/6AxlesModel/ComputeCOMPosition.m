function [x_com,y_com]=ComputeCOMPosition(rear_to_com_distance,has_orientation,x_,y_,z_)
    v = zeros(1,3);
    pos_vec = zeros(1,3);
    v(1,2) = rear_to_com_distance;
    pos_vec(1,1) = x_;
    pos_vec(1,2) = y_;
    pos_vec(1,3) = z_;
    
    if (has_orientation)
        %% ����˴�has_orientation Ϊ�棬���orientation����Ԫ��
        % ת����ŷ������Ϣ�������apollo math::Vec2d VehicleStateProvider::ComputeCOMPosition
        x_com = pos_vec(1,1) + v(1,1);
        y_com = pos_vec(1,2) + v(1,2);
    else
        x_com = pos_vec(1,1) + v(1,1);
        y_com = pos_vec(1,2) +v(1,2); %%  trucksim�в�������Ϊԭ�㣬������ǰ�ֻ��ߺ���������Ϊԭ��
                                                 %     ����apollo���Ժ���������Ϊ����ԭ��ģ����Ǽ������ʱ��������λ��Ϊ�ο����
    end



end