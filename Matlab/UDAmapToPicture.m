function [X,Y] = UDAmapToPicture(x,y)
%Die Funktion bildet von dem Koordinatensystem auf dem Auto in Pixel des
%Bildes ab
%   Detailed explanation goes here

%convert from Meter to pixel
x = x/5*512;
y = y/5*512;

%umdrehen
X = x;
Y = -y+256;

end

