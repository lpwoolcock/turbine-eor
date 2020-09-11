function [x_star, u_star, A, B, C, H, K] = eq_region3(tp, vx0, T)
    [x_star, u_star, A_c, B_c, H_c] = eq_region3_cont(tp, vx0);

    A = expm(A_c*T);
    B = A_c\(A-eye(size(A)))*B_c;
    C = [0 0 0 1 0];
    H = A_c\(A-eye(size(A)))*H_c;
    
    % needs to be using discretized versions
    coder.extrinsic('lqr');
    [K,~,~] = lqr(A_c,B_c,C.'*C,30);
end

