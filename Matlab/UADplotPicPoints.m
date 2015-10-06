function UADplotPicPoints( number )
%Diese Funktion plottet ein Bild un die dazugehörigen Punkte
%   Detailed explanation goes here

[pic,xl,yl,xhl,yhl,xr,yr,xhr,yhr] = UADgetData(number);
imshow(pic)
hold on
plot(xhl,yhl,'r+');
plot(xhr,yhr,'g+');
pause(0.001);
end

