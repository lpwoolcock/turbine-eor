function [d] = fdiff(f, x, dx)
    if nargin ~= 3
        dx = 1e-3;
    end
    
    d = (f(x+dx) - f(x)) / dx;
end