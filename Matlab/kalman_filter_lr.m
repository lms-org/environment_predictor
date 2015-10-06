function [r, Pk] = kalman_filter_lr (r, A, Pk, Q, R_fakt, delta, xl, yl, xr, yr, xm, ym) %#codegen

% "kalman_filter_lr" 
% 
% ----Nomenklatur----
%  * r: Zustandsvektor = [y0, phi0, kappa_1, kappa_2, ... , kappa_(n-1)] 
% 	--> y0: y-Wert des ersten Punktes (x-Wert ist immer 0)
% 	--> phi0: Anfangssteigung zwischem ersten und zweiten Punkt
% 	--> kappa_i: Krümmungen an den entsprechenden Punkten (Krümmungsdefinition: 1/R, 
% 	wobei R der Radius des Kreises ist, auf dem der betrachtete Punkt, sein Vorgänger 
% 	und sein Nachfolger liegen)
%  * Pk: Kovarianzmatrix des Zustands dim[n x n], mit n = Dimension des Zustandsvektors
%  * A: State-Transition-Matrix dim[n x n] (ist zurzeit eine Einheitsmatrix, da noch keine 
% 	 Messwerte fuer die Eigenbewegung verfügbar sind)
%  * Q: Kovarianz des Zustandsübergangs (Prozessrauschen) dim[n x n] (symmetrische Matrix, mit weg von der 
% 	 Diagonalen abnehmenden Eintraegen)
%  * R_fakt: Unsicherheit der Messwerte (Messrauschen)
%  * delta: Abstand der Punkte (delta*n ergibt die Länge des praedizierten Fahrstreifens)
%  * xl, yl: Vektoren mit den Messwerten für die linke Spur
%  * xr, yr: Vektoren mit den Messwerten für die rechte Spur
% 
%  ----Grober Ablauf des Algorithmus----
%  1. Projektion der Punkte von der Mittellinie nach links bzw. rechts
%  2. Fuer jeden Messpunkt: Berechnung des kleinsten Abstands zum aktuell praedizierten Strassenverlauf
%  3. Assemblierung der Jakobimatrix fuer die Projektion aus dem Zustandsraum r auf x-y-Koordinaten
%  4. Kalman-Filter: Praediktion -> Messwerte einbeziehen -> Update
%  5. Zustandsbegrenzungen einbringen



%% linke Seitenlinie
[xp, yp, phi] = projectPoints(r, delta, 0.38); %Punkte von Mittellinie nach links projizieren
P = [xp, yp, phi];
D = 10000*ones(numel(xl), 3);

% Fuer jeden Messpunkt den nähesten Punkt der aktuellen Praediktion finden 
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
                D(m, 1) = s-1;
                D(m, 2) = lambda;
                D(m, 3) = dist_line;              
            end
        end         
    end 
end

% Messpunkte ausfiltern, die vor dem letzten prädizierten Punkt oder zu weit entfernt liegen
ind = (D(:, 1) == numel(r) | D(:, 3) > 0.5);
xl(ind) = [];
yl(ind) = [];
D(ind, :) = [];


% Messmatrix, Mess- und Erwartungsvektor bauen
Hl = messmatrix(P, r, delta, D(:, 1), D(:, 2));
zl =  zeros(2*numel(xl), 1);
zml = zeros(2*numel(xl), 1);
count = 1;
for m=1:numel(xl)
        zl(count) = xl(m);
        zl(count + 1) = yl(m);
        zml(count)     = P(D(m, 1), 1) + D(m, 2) * (P(D(m, 1)+1, 1) - P(D(m, 1), 1));
        zml(count + 1) = P(D(m, 1), 2) + D(m, 2) * (P(D(m, 1)+1, 2) - P(D(m, 1), 2));
        count = count + 2;
end



%% rechte Seitenlinie
[xp, yp, phi] = projectPoints(r, delta, -0.38); %Punkte von Mittellinie nach rechts projizieren
P = [xp, yp, phi];
D = 10000*ones(numel(xr), 3); 

% Fuer jeden Messpunkt den nähesten Punkt der aktuellen Praediktion finden
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
                D(m, 1) = s-1;
                D(m, 2) = lambda;
                D(m, 3) = dist_line;              
            end
        end         
    end 
end

% Messpunkte ausfiltern, die vor dem letzten prädizierten Punkt oder zu weit entfernt liegen
% ind = ((D(:, 1) == numel(r) & D(:, 2) == 0) | D(:, 3) > 0.5 );
ind = (D(:, 1) == numel(r) | D(:, 3) > 0.5);
xr(ind) = [];
yr(ind) = [];
D(ind, :) = [];

% Messmatrix, Mess- und Erwartungsvektor bauen
Hr = messmatrix(P, r, delta, D(:, 1), D(:, 2));

zr =  zeros(2*numel(xr), 1);
zmr = zeros(2*numel(xr), 1);
count = 1;
for m=1:numel(xr)
        zr(count) = xr(m);
        zr(count + 1) = yr(m);    
        zmr(count)     = P(D(m, 1), 1) + D(m, 2) * (P(D(m, 1)+1, 1) - P(D(m, 1), 1));
        zmr(count + 1) = P(D(m, 1), 2) + D(m, 2) * (P(D(m, 1)+1, 2) - P(D(m, 1), 2));
        count = count + 2;
end


%% Mittellinie
[xp, yp, phi] = getPointsFromState(r, delta); 
P = [xp, yp, phi];
D = 10000*ones(numel(xm), 3);

% Fuer jeden Messpunkt den nähesten Punkt der aktuellen Praediktion finden 
for s=1:numel(r)
   for m=1:numel(xm)            
        dist_point = sqrt((P(s, 1)-(xm(m)))^2 + (P(s, 2)-(ym(m)))^2);
        if dist_point < D(m, 3)
            D(m, 1) = s;
            D(m, 2) = 0;
            D(m, 3) = dist_point;            
        end            
        if s > 1
            [dist_line, lambda, ~] = d_line_point([P(s-1, 1), P(s-1, 2)], [P(s, 1), P(s, 2)], [xm(m), ym(m)]);
            if dist_line < D(m, 3) && lambda > 0 && lambda < 1
                D(m, 1) = s-1;
                D(m, 2) = lambda;
                D(m, 3) = dist_line;              
            end
        end         
    end 
end

% Messpunkte ausfiltern, die vor dem letzten prädizierten Punkt oder zu weit entfernt liegen
ind = (D(:, 1) == numel(r) | D(:, 3) > 0.5);
xm(ind) = [];
ym(ind) = [];
D(ind, :) = [];


% Messmatrix, Mess- und Erwartungsvektor bauen
Hm = messmatrix(P, r, delta, D(:, 1), D(:, 2));
z_m =  zeros(2*numel(xm), 1);
zmm = zeros(2*numel(xm), 1);
count = 1;
for m=1:numel(xm)
        z_m(count) = xm(m);
        z_m(count + 1) = ym(m);
        zmm(count)     = P(D(m, 1), 1) + D(m, 2) * (P(D(m, 1)+1, 1) - P(D(m, 1), 1));
        zmm(count + 1) = P(D(m, 1), 2) + D(m, 2) * (P(D(m, 1)+1, 2) - P(D(m, 1), 2));
        count = count + 2;
end


%% Linke und rechte Linie kombinieren

H = [Hl; Hr; Hm]; %Messmatrix
z = [zl; zr; z_m]; %Messwerte
zm = [zml; zmr; zmm]; %erwartete Werte


%% KALMAN FILTER    
dd = 0.013;
A = zeros(numel(r));
A(1,1) = 1;
A(2,2) = 1;
A(2,3) = dd/sqrt(1-delta^2*r(3)^2/4);
for i=3:numel(r)-1
    A(i,i) = 1-dd/delta;
    A(i,i+1) = dd/delta;
end
A(numel(r),numel(r)) = 1;


r = A*r;
R = eye(2*(numel(xl)+numel(xr)+numel(xm))).*R_fakt; 

Pk = A*Pk*A' + Q;
y_tilde = z - zm;
K = Pk*H'/(H*Pk*H' + R);
r = r + K*y_tilde;
Pk = (eye(numel(r)) - K*H)*Pk;


%% Zustandsbegrenzungen
% Krümmung
kr = 0.75; % bezogen auf Mittellinie, 0.72 entspricht einem minimalen Innenradius von 1m
for i=3:numel(r)
    r(i) = max(-kr, r(i));
    r(i) = min(kr, r(i));
end

% y-Wert des ersten Punktes
y_max = 0.5;
r(1) =  max(-y_max, min(y_max, r(1)));

% Startwinkel 
phi_max = 0.78;
r(2) =  max(-phi_max, min(phi_max, r(2))); % 45°










