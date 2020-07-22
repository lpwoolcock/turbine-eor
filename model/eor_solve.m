function [Pi, Gamma] = eor_solve(A,B,Ew,Ce,Deu,Dew,S)
    % for square systems
    n = size(A, 1); % no. system states
    m = size(B, 2); % no. system inputs
    q = size(Ce, 1); % no. system outputs
    s = size(S, 1); % no. exosystem states
    
    A1 = [A B; Ce Deu];
    A1_dagger = pinv(A1);
    A2 = [eye(n) zeros(n,m); zeros(q,n+m)];
    
    coder.extrinsic('dlyap');
    X = dlyap(A1_dagger*A2, S, -A1_dagger*[Ew;Dew]);
    Pi = X(1:n,1:s);
    Gamma = X(n+1:n+m,1:s);
end