function [r, Pk] = kalman_filter (r, A, Pk, Q, R_fakt, delta, xm, ym)


% Punktkoordinaten aus r berechnen
P = zeros(numel(r), 3); %x, y, phi
phi = r(2);
D = 10000*ones(numel(xm), 3);

for s=1:numel(r)
    if s==1
        P(s, 1) = 0;
        P(s, 2) = r(1);
        P(s, 3) = phi;
    elseif s==2
        P(s, 1) = P(s-1, 1) + delta*cos(r(2));
        P(s, 2) = P(s-1, 2) + delta*sin(r(2));  
        P(s, 3) = phi;
    else        
        dw = 2*acos(delta*r(s)/2);
        phi = phi - dw - pi; % -dw wegen VZ-Definition der Krümmung
        P(s, 1) = P(s-1, 1) + delta*cos(phi);
        P(s, 2) = P(s-1, 2) + delta*sin(phi);
        P(s, 3) = phi;             
    end  

    
    %geringster Abstand von Messpunkten und Strecke
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
                D(m, 1) = s;
                D(m, 2) = lambda;
                D(m, 3) = dist_line;
            end
        end         
    end  
end

% Messpunkte ausfiltern, die vor dem letzten prädizierten Punkt liegen
% for k=1:numel(xm)
%     if numel(xm) < k
%         if (D(k, 1) == numel(r)-1 && D(k, 2) ~= 0 && D(k, 3) > delta/2) || D(k, 3) > 0.5
%             xm(k) = [];
%             ym(k) = [];
%             D(k, :) = [];
%             R(k, :) = [];
%             R(:, k) = [];
%             R(k+1, :) = [];
%             R(:, k+1) = [];
%         end
%     end    
% end


% Messmatrix, Mess- und Erwartungsvektor bauen
H = messmatrix_new(P, r, delta, D(:, 1), D(:, 2));
z =  zeros(2*numel(xm), 1);
zm = zeros(2*numel(xm), 1);
count = 1;

for m=1:numel(xm)
%         H(count:count+1, 1:end) = Messmatrix(P, r, delta, D(m, 1), D(m, 2));
        z(count) = xm(m);
        z(count + 1) = ym(m);
        if D(m, 2) > 0 
            zm(count)     = P(D(m, 1)-1, 1) + D(m, 2) * (P(D(m, 1), 1) - P(D(m, 1)-1, 1));
            zm(count + 1) = P(D(m, 1)-1, 2) + D(m, 2) * (P(D(m, 1), 2) - P(D(m, 1)-1, 2));
        else
            zm(count) = P(D(m, 1), 1);
            zm(count + 1) = P(D(m, 1), 2);  
        end        
        count = count + 2;
end


%KALMAN FILTER    
% r = f(r) %state transition, abhängig von der Eigenbewegung
% A = transition_matrix(r, d_x, d_y, delta);
R = eye(2*(numel(xm))).*R_fakt; 

Pk = A*Pk*A' + Q;
y_tilde = z - zm;
K = Pk*H'/(H*Pk*H' + R);
r = r + (K*y_tilde)';
Pk = (eye(numel(r)) - K*H)*Pk;

% x0 = x0 + d_i*cos(r(2));
% y0 = y0 + d_i*sin(r(2));

% Zustandsbegrenzung
fakt = 0.9; %maximale Krümmung
for i=3:numel(r)
    r(i) = max(-fakt, r(i));
    r(i) = min(fakt, r(i));
end

% y-Wert des ersten Punktes
r(1) =  max(-0.5, r(1));
r(1) =  min(0.5, r(1));

% Startwinkel 
r(2) =  max(-0.78, r(2)); % 45°
r(2) =  min(0.78, r(2));








