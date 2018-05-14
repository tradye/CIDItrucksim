function k =SolveLQRProblem(A,B,Q,R,tolerance,max_num_iteration)
    AT = A';BT = B';
   
    persistent  num_iteration ;
    persistent  diff ;
    persistent  P_next ;
%     persistent  P ;
    diff = 100.0;
    num_iteration = 0;
    P_next = zeros(4,4);P = zeros(4,4);
    P = Q;
    while ((num_iteration < max_num_iteration) && (diff > tolerance)  )
       
         P_next = AT * P * A - AT * P * B * inv(R + BT * P * B)* BT * P * A + Q;
        P_tmp = (P_next - P);
        diff = abs(max(P_tmp(:)));
        P = P_next;            
        
        num_iteration = num_iteration + 1;
%         fprintf('num_iteration = %f \n',num_iteration);
    end
    
%     fprintf('num_iteration = %f \n',num_iteration);
    
    if(num_iteration >= max_num_iteration)
        fprintf('LQR solver cannot converge to a solution,');
        fprintf('last consecutive result diff. is: %f \n',diff);
    else
        fprintf('LQR solver converged at iteration: %f',num_iteration);
        fprintf(' max consecutive result diff.: %f \n',diff);
    end
        
    k = inv(R + BT * P *B) * BT * P * A;

end