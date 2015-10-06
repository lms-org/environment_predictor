function [r, Pk] = kalman_filter_middle (r, A, Pk, Q, R_fakt, delta, xl, yl, xr, yr)


%% linke Seitenlinie
[xp, yp] = projectPoints(r, delta, 0.38); %Punkte von Mittellinie nach links projizieren
P = [xp, yp, zeros(numel(xp), 1)];

D = 10000*ones(numel(xl), 3);
for s=1:numel(r)
   for m=1:numel(xl)            
        dist_point = sqrt((P(s, 1)-(xl(m)))^2 + (P(s, 2)-(yl(m)))^2);
        if dist_point < D(m, 3)
            D(m, 1) = s;
            D(m, 2) = 0;
            D(m, 3) = dist_point;            
        end            
        if s > 1
            [dist_line, lambda, ~] = d_line_point([P(s-1, 1), P(s-1, 2)], [P(s, 1), P(s, 2)], [xl(m), yl(m)]);
            if dist_line < D(m, 3) && lambda > 0 && lambda < 1
                D(m, 1) = s;
                D(m, 2) = lambda;
                D(m, 3) = dist_line;              
            end
        end         
    end 
end

% Messpunkte ausfiltern, die vor dem letzten prädizierten Punkt liegen
ind = ((D(:, 1) == numel(r) & D(:, 2) == 0) | D(:, 3) > 0.5);
xl(ind) = [];
yl(ind) = [];
D(ind, :) = [];

% Messmatrix, Mess- und Erwartungsvektor bauen
Hl = messmatrix_new(P, r, delta, D(:, 1), D(:, 2));
zl =  zeros(2*numel(xl), 1);
zml = zeros(2*numel(xl), 1);
count = 1;
for m=1:numel(xl)
        zl(count) = xl(m);
        zl(count + 1) = yl(m);
        if D(m, 2) > 0 
            zml(count)     = P(D(m, 1)-1, 1) + D(m, 2) * (P(D(m, 1), 1) - P(D(m, 1)-1, 1));
            zml(count + 1) = P(D(m, 1)-1, 2) + D(m, 2) * (P(D(m, 1), 2) - P(D(m, 1)-1, 2));
        else
            zml(count) = P(D(m, 1), 1);
            zml(count + 1) = P(D(m, 1), 2);  
        end         
        count = count + 2;
end



%% rechte Seitenlinie
[xp, yp] = projectPoints(r, delta, -0.38); %Punkte von Mittellinie nach rechts projizieren
P = [xp, yp, zeros(numel(xp), 1)];

D = 10000*ones(numel(xr), 3);
for s=1:numel(r)
   for m=1:numel(xr)            
        dist_point = sqrt((P(s, 1)-(xr(m)))^2 + (P(s, 2)-(yr(m)))^2);
        if dist_point < D(m, 3)
            D(m, 1) = s;
            D(m, 2) = 0;
            D(m, 3) = dist_point;            
        end            
        if s > 1
            [dist_line, lambda, ~] = d_line_point([P(s-1, 1), P(s-1, 2)], [P(s, 1), P(s, 2)], [xr(m), yr(m)]);
            if dist_line < D(m, 3) && lambda > 0 && lambda < 1
                D(m, 1) = s;
                D(m, 2) = lambda;
                D(m, 3) = dist_line;              
            end
        end         
    end 
end

% Messpunkte ausfiltern, die vor dem letzten prädizierten Punkt liegen
ind = ((D(:, 1) == numel(r) & D(:, 2) == 0) | D(:, 3) > 0.5);
xr(ind) = [];
yr(ind) = [];
D(ind, :) = [];

% Messmatrix, Mess- und Erwartungsvektor bauen
Hr = messmatrix_new(P, r, delta, D(:, 1), D(:, 2));

zr =  zeros(2*numel(xr), 1);
zmr = zeros(2*numel(xr), 1);
count = 1;
last_val = struct();
for m=1:numel(xr)
        zr(count) = xr(m);
        zr(count + 1) = yr(m);
        if D(m, 2) > 0 
            zmr(count)     = P(D(m, 1)-1, 1) + D(m, 2) * (P(D(m, 1), 1) - P(D(m, 1)-1, 1));
            zmr(count + 1) = P(D(m, 1)-1, 2) + D(m, 2) * (P(D(m, 1), 2) - P(D(m, 1)-1, 2));
        else
            zmr(count) = P(D(m, 1), 1);
            zmr(count + 1) = P(D(m, 1), 2);  
        end            
        count = count + 2;
end


%% beide Linien kombinieren

H = [Hl; Hr];
z = [zl; zr];
zm = [zml; zmr];

R = eye(2*(numel(xl)+numel(xr))).*R_fakt;

% H = Hl;
% z = zl;
% zm =zml;

% H = Hr;
% z = zr;
% zm =zmr;


%% KALMAN FILTER    
% r = A*r + B*u

Pk = A*Pk*A' + Q;
y_tilde = z - zm;
K = Pk*H'/(H*Pk*H' + R);
r = r + (K*y_tilde)';
Pk = (eye(numel(r)) - K*H)*Pk;


%% Zustandsbegrenzunen
% Krümmung
kr = 0.72; 
for i=3:numel(r)
    r(i) = max(-kr, r(i));
    r(i) = min(kr, r(i));
end

% y-Wert des ersten Punktes
% r(1) =  max(-0.3, r(1));
% r(1) =  min(0.3, r(1));

% Startwinkel 
% r(2) =  max(-1, r(2));
% r(2) =  min(1, r(2));







