function [pic,xl,yl,xhl,yhl,xr,yr,xhr,yhr] = UADgetData(number)
%read data from the datalog and calculate the transformed points
%   Natürlich muss der Pfad geändert werden, wenn man auf einer anderen
%   Maschine ist



if number < 1 || number > 1431
    error('diese zeit gibt es nicht');
end



%% Code to import points (MATLAB auto generated)

%% %% %%%%%%%% linke Punkte


filename = ['C:\Users\Tobias\Desktop\logging_data\_0_', num2str(number),'.csv'];
delimiter = ',';

%% Format string for each line of text:
%   column1: double (%f)
%	column2: double (%f)
% For more information, see the TEXTSCAN documentation.
formatSpec = '%f%f%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to format string.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter,  'ReturnOnError', false);

%% Close the text file.
fclose(fileID);

%% Post processing for unimportable data.
% No unimportable data rules were applied during the import, so no post
% processing code is included. To generate code which works for
% unimportable data, select unimportable cells in a file and regenerate the
% script.

%% Allocate imported array to column variable names
xl = dataArray{:, 1};
yl = dataArray{:, 2};

%näher als 0 meter und weiter als 2.5 m,eter muss weg
indWrongl = (xl<= 0) | (xl > 2.5);

xl = xl(~indWrongl);
yl = yl(~indWrongl);


%% Clear temporary variables
clearvars filename delimiter formatSpec fileID dataArray ans;










%% %% %%%%%%%% rechte Punkte
filename = ['C:\Users\Tobias\Desktop\logging_data\_2_', num2str(number),'.csv'];
delimiter = ',';

%% Format string for each line of text:
%   column1: double (%f)
%	column2: double (%f)
% For more information, see the TEXTSCAN documentation.
formatSpec = '%f%f%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to format string.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter,  'ReturnOnError', false);

%% Close the text file.
fclose(fileID);

%% Post processing for unimportable data.
% No unimportable data rules were applied during the import, so no post
% processing code is included. To generate code which works for
% unimportable data, select unimportable cells in a file and regenerate the
% script.

%% Allocate imported array to column variable names
xr = dataArray{:, 1};
yr = dataArray{:, 2};

%näher als 0 meter und weiter als 2.5 m,eter muss weg
indWrongr = (xr<= 0) | (xr >= 1.8);


xr = xr(~indWrongr);
yr = yr(~indWrongr);


%% Clear temporary variables
clearvars filename delimiter formatSpec fileID dataArray ans;







%% Koordinaten Transformieren

[xhl,yhl] = UDAmapToPicture(xl,yl);
[xhr,yhr] = UDAmapToPicture(xr,yr);


%% Bild imprtieren

pfadBild = ['C:\Users\Tobias\Desktop\logging_data\', sprintf('%04i.pgm',number-1)];
% pfadBild = ['C:\Users\Tobias\Desktop\logging_data\_0_', num2str(number),'.csv'];
pic = imread(pfadBild);


end

