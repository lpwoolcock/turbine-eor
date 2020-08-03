
%load the data




% 
% MyT = NRELBaseline.TwrBsMyt;
% MxT = NRELBaseline.TwrBsMxt;
% MyB = NRELBaseline.RootMyb1;
% MxB = NRELBaseline.RootMxb1;
% T = NRELBaseline.Time;
% LSS = NRELBaseline.RotTorq;
% 
% moments = [MyT MxT MyB MxB LSS];
% wohler = [4 4 10 10 4];
% 
% %linear
% size(moments,2);
% avg = zeros(size(moments,2),1);
% for i=1:size(moments,2)
%     avg(i,1) = get_average_moment(rainflow(moments(1:end,i)),wohler(i),T(end)-T(1));
% end
% 
% avg
% 
% 
% %average moments
% function sum = get_average_moment(c,m,t)
% 
% sum = 0;
% 
% for i=1:size(c,1)
%     sum = sum + c(i,1)*c(i,2)^m/t;
% end
% sum = sum^(1/m);
% 
% end
