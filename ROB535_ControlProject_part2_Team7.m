function [U, FLAG_terminate] = ROB535_ControlProject_part2_Team7(TestTrack, Xobs_seen, inputz)
%P control
% close all
% clear
% clc

cline = TestTrack.cline;
cx = cline(1,:);
cy = cline(2,:);


theta = TestTrack.theta;

bl = TestTrack.bl;
bl_x = bl(1,:);
bl_y = bl(2,:);

br = TestTrack.br;
br_x = br(1,:);
br_y = br(2,:);

% leftcenter = (cline + 0.99*bl) ./2;
% rightcenter = (cline + 1.009*br) ./2;
leftcenter = ((cline + bl) ./2);
rightcenter = ((cline + br) ./2);
lc_x = leftcenter(1,:);
lc_y = leftcenter(2,:);
rc_x = rightcenter(1,:);
rc_y = rightcenter(2,:);

theta_l = [0,atan2(diff(lc_y),diff(lc_x))];
theta_r = [0,atan2(diff(rc_y),diff(rc_x))];

% initial value
%         x   u  y  v psi r
z(1,:) = inputz;

F_x = 0;

cxx = cx;
cyy = cy;
ctheta = theta;
for i = 1:51 
    %% added part
    % justify side of obsticle and change tracing line

    if ~isempty(Xobs_seen)
           
        % find the nearest obsticle
        center = obs_center(Xobs_seen);
        obs_x = -1;
        obs_y = -1;
        for j = 1:size(center,1)
            if center(j, 1) > z(i, 1)
                obs_x = center(j,1);
                obs_y = center(j,2);
                break;
            end
        end

        if obs_x ~= -1
        
            idxobs = knnsearch([cx; cy]',[obs_x obs_y]);
%         side_obs = sign( cos(theta(idxobs)) * (cx(idxobs) - obs_x) );
%             side_obs = sign(cos(theta(idxobs))* (cx(idxobs) - obs_x) );
            
            
%             if obs_y < -5&& obs_y >-70
            if obs_y < 36 && obs_y >-119
                side_obs = -sign(cos(theta(idxobs))*(obs_y-cy(idxobs)) - sin(theta(idxobs))*(obs_x-cx(idxobs))) ;
            else
                side_obs = sign(cos(theta(idxobs))*(obs_y-cy(idxobs)) - sin(theta(idxobs))*(obs_x-cx(idxobs)));
            end


            if side_obs == 1
                cxx = rc_x;
                cyy = rc_y;
                ctheta = theta_r;
            elseif side_obs == -1
                cxx = lc_x;
                cyy = lc_y;
                ctheta = theta_l;
            end
        end
    else
        cxx = cx;
        cyy = cy;
        ctheta = theta;
    end

    %% added part end


    [idx, D] = knnsearch([cxx; cyy]',[z(i,1) z(i,3)]);

    % distance beween car and "cline"
%     cur_angle = atan2(cyy(idx)- z(i,3), cxx(idx) - z(i,1)); % arctan(y/x)
%     distance = abs(D * sin(theta(idx) - cur_angle));
    distance = sqrt( (cxx(idx) - z(i,1)).^2 + (cyy(idx) - z(i,3)).^2  );

%     r = sqrt( (cx(idx) - z(i,1)).^2 + (cy(idx) - z(i,3)).^2  );

    
    % justify side of cline and determine input based on it
%     side = sign( cos(theta(idx)) * (cxx(idx) - z(i,1)) );
    side = sign(cos(ctheta(idx))*(z(i,3)-cyy(idx)) - sin(ctheta(idx))*(z(i,1)-cxx(idx)));
%     side = sign(cos(ctheta(idx))* (cxx(idx) - z(i,1)) );
%     u = [-0.03*side*distance - 2*(z(i,5)-ctheta(idx+1)), F_x];  % Input
    u = [-0.02*side*distance - 1*(z(i,5)-ctheta(idx+1)), F_x];  % Input

    if z(i,1) > 1469
        FLAG_terminate = true;
    else
        FLAG_terminate = false;
    end

    
    % Speed Control
    if z(i,2) > 10
        F_x = 0;
    elseif z(i,2) > 5
        F_x = 40;
    else
        F_x = 990;
    end
    
%     if z(i,1) - cx(idx) > 5
%         u(1) = u(1) + 0.02;
%     elseif cx(idx) - z(i,1) >5
%         u(1) = u(1) - 0.02;
%     end

    % steering angel control
    if u(1) > 0.5
        u(1) = 0.5;
    elseif u(1) < -0.5
        u(1) = -0.5;
    end

    % save cur model
    U(i,:) = u;

    % Step forward
    xnew = stepin(z(i,:),u);
    z(i+1,:) = xnew(end,:);

   
end
% plot(z(:,1),z(:,3))

plot(br_x,br_y,'k')
hold on
plot(bl_x,bl_y,'k')
%plot(sol1(:,1),sol1(:,3))
plot(z(:,1),z(:,3))
for i = 1:length(Xobs_seen)
    plot(Xobs_seen{1,i}(:,1),Xobs_seen{1,i}(:,2),'m')
end
% 
% ROB535_ControlProject_part1_input = U;



%% helper functions
function [Y] = stepin(z,U)

% generate time vector
T = [0 0.01];

% constants
delta_f = U(1);
m = 1400;
Nw = 2;
f = 0.01;
Iz = 2667;
a = 1.35;
b = 1.45;
By = 0.27;
Cy  = 1.2;
Dy = 0.7;
Ey = -1.6;
Shy = 0;
Svy = 0;
g = 9.806;

% front and rear lateral slip angles
alpha_f = @(t,x) rad2deg(delta_f-atan2(x(4)+a*x(6),x(2))) ;
alpha_r = @(t,x) rad2deg(-atan2((x(4)-b*x(6)),x(2))) ;

% lateral forces
phi_yf = @(t,x) (1-Ey)*(alpha_f(t,x)+Shy)+(Ey/By)*atan(By*(alpha_f(t,x)+Shy)) ;
phi_yr = @(t,x) (1-Ey)*(alpha_r(t,x)+Shy)+(Ey/By)*atan(By*(alpha_r(t,x)+Shy)) ;


F_zf = b/(a+b) * m * g;
F_zr = a/(a+b) * m * g;

F_yf = @(t,x) F_zf * Dy * sin( Cy*atan(By*phi_yf(t,x)) ) + Svy ;
F_yr = @(t,x) F_zr * Dy * sin( Cy*atan(By*phi_yr(t,x)) ) + Svy ;

% F_tot = sqrt((Nw*Fx)^2+(F_yr^2));
% F_max = 0.7*m*g;
% 
% if F_tot > F_max
%     Fx = F_max/F_tot*Fx;
%     F_yr = F_max/F_tot*F_yr;
% end

% output states
dZ = @(t,x) [x(2)*cos(x(5))-x(4)*sin(x(5)) ;...
            1/m * (-f*m*g+Nw*U(2)-F_yf(t,x)*sin(delta_f)) + x(4) * x(6) ;...
            x(2)*sin(x(5))+x(4)*cos(x(5)) ;...
            1/m * (F_yf(t,x)*cos(delta_f)+F_yr(t,x)) - x(2)*x(6) ;...
            x(6) ;...
            (F_yf(t,x)*a*cos(delta_f)-F_yr(t,x)*b)/Iz] ;
      
% Solve for trajectory
[~,Y] = ode45(dZ,T,z) ;

end

function center = obs_center(Xobs_seen)
    center = zeros(size(Xobs_seen,2),2);
    for l = 1:size(Xobs_seen,2)
        obs = Xobs_seen{l};
        center(l,1) = mean(obs(:,1));
        center(l,2) = mean(obs(:,2));
    end
    
end

end
