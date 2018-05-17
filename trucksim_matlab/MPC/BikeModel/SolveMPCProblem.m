function [control_out] = SolveMPCProblem(matrix_a,matrix_b,matrix_c,matrix_q,matrix_r,...
                                                              matrix_lower,matrix_upper,reference,matrix_init_state,control)
                                                          
% 
%  @brief Solver for discrete-time model predictive control problem.
%  @param matrix_a The system dynamic matrix
%  @param matrix_b The control matrix
%  @param matrix_c The disturbance matrix
%  @param matrix_q The cost matrix for control state
%  @param matrix_lower The lower bound control constrain matrix
%  @param matrix_upper The upper bound control constrain matrix
%  @param matrix_initial_state The initial state matrix
%  @param reference The control reference vector with respect to time
%  @param tolerance The control convergence tolerance
%  @param max_num_iteration The maximum iterations for solving ARE
%  @param control The feedback control matrix (pointer)
%  

%     [horizon , ~] = size(reference);
%     [rows , ~] = size(matrix_b);
    horizon = size(reference,1);
    rows = size(matrix_b,1);
    cols =  size(matrix_b,2);
    matrix_t = zeros(rows * horizon , 1);
    for i = 1:1:horizon
         matrix_t( ((i -1) * size(reference{1},1)  + 1): (size(reference{1},1) + (i - 1) * size(reference{1},1)) , 1 : 1) = reference{i,1};
    end
%     control = cell(horizon,1); 
    matrix_v = zeros(size(control{1} , 1) * horizon , 1);
    for j = 1:1:horizon
        matrix_v( ((j -1) * size(control{1},1)  + 1): (size(control{1},1) + (j - 1) * size(control{1},1)) ,  1 : 1 ) = control{j};
    end
    
    a_power = cell(horizon,1);
    a_power{1} = matrix_a;
    
    for i = 2:1:horizon
         a_power{i} = matrix_a * a_power{i - 1};
    end
    
    matrix_k = zeros(rows * horizon , size(matrix_b,2) * horizon);
    for i = 1:1:horizon
        for j = 1:1: i
             if(j == i)
                matrix_k( ((i - 1)* rows + 1) :  (rows + (i - 1) * rows) ,  ((j - 1) * cols + 1 ) : (cols +  (j - 1) * cols) ) =  matrix_b;
            else
                matrix_k( ((i - 1)* rows + 1) :  (rows + (i - 1) * rows) ,  ((j - 1) * cols + 1 ) : (cols +  (j - 1) * cols) ) = a_power{i - j} * matrix_b;
             end
        end
    end
    % Initialize matrix_k, matrix_m, matrix_t and matrix_v, matrix_qq, matrix_rr,
    % vector of matrix A power   
    matrix_m = zeros(rows *horizon , 1);
    matrix_qq = zeros(size(matrix_k , 1)  , size(matrix_k , 1) );
    matrix_rr = zeros(size(matrix_k , 2)  , size(matrix_k , 2) );
    matrix_ll = zeros(size(matrix_lower , 1) *horizon , 1);
    matrix_uu = zeros(size(matrix_upper , 1) *horizon , 1);
    % Compute matrix_m
    matrix_m (1 : size(matrix_a,1) , 1 : 1) = matrix_a * matrix_init_state + matrix_c; 
    for i =2:1:horizon
          matrix_m(   ((i - 1) * size(matrix_a,1) + 1)  :  (size(matrix_a,1) + (i - 1) * size(matrix_a,1) ) , 1 : 1   ) = ...
            matrix_a * matrix_m( ((i - 2) * size(matrix_a,1) + 1 )  :  (size(matrix_a,1) + (i - 2) * size(matrix_a,1) ) , 1 : 1 ) + matrix_c;
    end
    % Compute matrix_ll, matrix_uu, matrix_qq, matrix_rr
%     P.block(i, j, rows, cols)          // P(i+1 : i+rows, j+1 : j+cols)
%     P.block<rows, cols>(i, j)          // P(i+1 : i+rows, j+1 : j+cols) 
%     for (unsigned int i = 0; i < horizon; ++i) {
%         matrix_ll.block(i * (*control)[0].rows(), 0, (*control)[0].rows(), 1) =matrix_lower;
%         matrix_uu.block(i * (*control)[0].rows(), 0, (*control)[0].rows(), 1) = matrix_upper;
%         matrix_qq.block(i * matrix_q.rows(), i * matrix_q.rows(), matrix_q.rows(), matrix_q.rows()) = matrix_q;
%         matrix_rr.block(i * matrix_r.rows(), i * matrix_r.rows(), matrix_r.cols(),matrix_r.cols()) = matrix_r;
%     }
    for i = 1 : 1: horizon
        matrix_ll( ((i - 1) * size(control{1} , 1)  + 1 )   :  (size(control{1} ,1)  + (i - 1) * size(control{1} , 1)) , 1 : 1  ) = matrix_lower;
        matrix_uu(  ((i - 1) * size(control{1} , 1)  + 1 )   :  (size(control{1} ,1)  + (i - 1) * size(control{1} , 1)) , 1 : 1   ) = matrix_upper;
        matrix_qq(  ((i - 1) * size(matrix_q ,1) +  1) :  (size(matrix_q ,1) + (i - 1) * size(matrix_q ,1)) ,...
          ( ( i - 1)* size(matrix_q ,1) + 1) : (size(matrix_q ,1) + ( i - 1) * size(matrix_q ,1)) ) = matrix_q;
        matrix_rr( ((i - 1) * size(matrix_r ,1) +  1) :  (size(matrix_r ,1) + (i - 1) * size(matrix_r ,1)) ,...
          ( ( i - 1)* size(matrix_r ,1) + 1) : (size(matrix_r ,1) + ( i - 1) * size(matrix_r ,1))  ) = matrix_r;
    end
    % Update matrix_m1, matrix_m2, convert MPC problem to QP problem done
    matrix_m1 = matrix_k' * matrix_qq * matrix_k + matrix_rr;
    matrix_m2 = matrix_k' * matrix_qq * (matrix_m - matrix_t);
    
    matrix_inequality_constrain_ll = eye(size(matrix_ll , 1) , size(matrix_ll , 1));
    matrix_inequality_constrain_uu = eye(size(matrix_uu , 1) , size(matrix_uu , 1));
    
    matrix_inequality_constrain = zeros((size(matrix_ll , 1) + size(matrix_uu , 1)) , size(matrix_ll , 1));
    for i = 1:1:(size(matrix_ll , 1) + size(matrix_uu , 1))
        for j = 1:1:size(matrix_ll , 1)
            if i <= size(matrix_ll , 1)
                matrix_inequality_constrain(i,j) = matrix_inequality_constrain_ll(i,j) ;
            else
                matrix_inequality_constrain(i,j) = matrix_inequality_constrain_uu(i - size(matrix_ll , 1), j )*(-1);
            end
        end
    end
    
    matrix_inequality_boundary = zeros((size(matrix_ll , 1) + size(matrix_uu , 1)) , size(matrix_ll , 2));
     for i = 1:1:(size(matrix_ll , 1) + size(matrix_uu , 1))
        for j = 1:1:size(matrix_ll , 2)
            if i <= size(matrix_ll , 1)
                matrix_inequality_boundary(i,j) = matrix_ll(i,j);
            else
                matrix_inequality_boundary(i,j) = matrix_uu(i - size(matrix_ll , 1), j )*(-1);
            end
        end
     end
    
    matrix_equality_constrain = zeros((size(matrix_ll , 1) + size(matrix_uu , 1)) , size(matrix_ll , 1));   
    matrix_equality_boundary = zeros((size(matrix_ll , 1) + size(matrix_uu , 1)) , size(matrix_ll , 2));

    [L, ~] = chol(matrix_m1,'lower');
    Linv = inv(L);
    opt = mpcqpsolverOptions;
    matrix_v = mpcqpsolver(Linv,matrix_m2,matrix_inequality_constrain,...
         matrix_inequality_boundary,[],zeros(0,1),false(size(matrix_inequality_boundary)),opt);
     control_out = cell(horizon,1); 
     for i = 1:1:horizon
         control_out{i,1} = matrix_v( ((i -1) * size(control{1},1)  + 1): (size(control{1},1) + (i - 1) * size(control{1},1)) ,  1 : 1 );
     end










end