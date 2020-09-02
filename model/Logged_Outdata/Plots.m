names = importdata('simulink_outfile_list.csv');
names = names(2:end);
dels = importdata('DELs.csv')

windspeeds = [2 4];
load_names = {'MyT','MxT','MyB','MxB','LSS'};

 
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
 
 for i = 1:size(windspeeds,2)
    figure()
    bar(windspeeds,dels(:,i)) 
    title(sprintf('DELs for %s',char(load_names(i))))
    xlabel windspeeds
    ylabel DEL
     
 end
     
     
 
