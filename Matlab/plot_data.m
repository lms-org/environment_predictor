clc;
clear F;

r = zeros(6,1);
r_r = r;
r_l = r;

delta = 0.3; %Länge der Segmente
Pk = eye(numel(r)).*1; %Anfanskovarianz (Verlässlichkeit des Ausgangszustands)
Pk_r = Pk;
Pk_l = Pk;

A = eye(numel(r));

Q = eye(numel(r)).*1; %Kovarianz des Zustandsübergangs
for i=1:numel(r)
    for k=1:numel(r)
        Q(i, k) = 15*(1 - 0.2^(1/abs(k-i)));
    end
end


mov_frame = 1;
for i=300:1400
    
    clc;
    clf;

    disp(['frame #', num2str(i)])

    [pic,xl,yl,xhl,yhl,xr,yr,xhr,yhr] = UADgetData(i);
    imshow(pic)
    hold on    
    
    [r, Pk]  = kalman_filter_lr(r, A, Pk, Q, 10, delta, xl, yl, flipud(xr), flipud(yr));


    disp(['Krümmung mittlere Spur:  ', sprintf('%.1f', mean(r(3:5)))])    
      
    %Plots   
    [xpl, ypl] = projectPoints(r, delta, 0.38);
    [X,Y] = UDAmapToPicture(xpl, ypl);
    plot(X, Y, '-x', 'color', 'white', 'linewidth', 2)
    [xpl, ypl] = projectPoints(r, delta, -0.38);
    [X,Y] = UDAmapToPicture(xpl, ypl);
    plot(X, Y, '-x', 'color', 'white', 'linewidth', 2)    
   
    
    [x, y] = getPointsFromState(r, delta);  
    [X,Y] = UDAmapToPicture(x, y);
    plot(X, Y, '-+', 'color', 'yellow','linewidth', 2)
    
%     axis([0 600 0 600]);
    
    
    % Messpunkte
    plot(xhl,yhl,'r+');
    plot(xhr,yhr,'g+');


    drawnow
    
%     pause(0.1)
%     waitforbuttonpress

%     F(mov_frame) = getframe(gcf);
    mov_frame = mov_frame + 1;

end

% movie(F, 2, 30)
