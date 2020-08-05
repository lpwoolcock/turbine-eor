names = importdata('simulink_outfile_list.csv');
names = names(2:end);

windspeeds = [2 4 6 8];


 
 for j = 1:size(windspeeds,2)
     
     
     Data = load(sprintf('simulink_outfile_%d.mat',windspeeds(j))).data;
     data = Data.data;
     time = Data.time;

     
     for i=1:9
        figure()
        plot(time,data(:,i));
        title(sprintf('%s windspeed %d',char(names(i)),windspeeds(j)))
     end
     
     
 end
 
