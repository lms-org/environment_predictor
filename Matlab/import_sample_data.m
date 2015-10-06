
linePoints = struct;
delimiter = ',';
formatSpec = '%f%f%[^\n\r]';

for i=1:1431
    filename = ['C:\Users\Tobias\Desktop\logging_data\_0_', num2str(i),'.csv'];

    fileID = fopen(filename,'r');
    dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter,  'ReturnOnError', false);
    fclose(fileID);
    linePoints.(['frame_' num2str(i)]).xl = dataArray{:, 1};
    linePoints.(['frame_' num2str(i)]).yl = dataArray{:, 2}; 
    
    filename = ['C:\Users\Tobias\Desktop\logging_data\_2_', num2str(i),'.csv'];
    fileID = fopen(filename,'r');
    dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter,  'ReturnOnError', false);
    fclose(fileID);
    linePoints.(['frame_' num2str(i)]).xr = dataArray{:, 1};
    linePoints.(['frame_' num2str(i)]).yr = dataArray{:, 2};     
    
end

clearvars filename delimiter formatSpec fileID dataArray ans;

