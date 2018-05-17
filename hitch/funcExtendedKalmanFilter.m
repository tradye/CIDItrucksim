function [mu1, Sigma1] = funcExtendedKalmanFilter(Sigma0, z1, G1, R1, Q1, H1, g)    
    if g > 1
       disp("Got here")
    end
    p_mu1 = g;
    p_Sigma1 = G1*Sigma0*G1' + R1;
    K1 = p_Sigma1*H1'*inv(H1*p_Sigma1*H1' + Q1);
    mu1 = p_mu1 + K1*(z1 - p_mu1);
    if mu1 > 1
       disp("Got here") 
    end
    I = eye(size(K1*H1));
    Sigma1 = (I-K1*H1)*p_Sigma1;    
end %funcEKF