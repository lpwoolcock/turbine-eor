function sum = get_average_moment(c,m,t)
    sum = 0;
    secs_in_20yrs = 6.307*10^8;
    nLCref = 2*10^6;
    for i=1:size(c,1)
        sum = sum + (c(i,1)*c(i,2)^m);
    end
    %weigting assume life time of 20years 
    sum = ((secs_in_20yrs*sum)/(nLCref*t))^(1/m);

end