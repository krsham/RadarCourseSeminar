clc; clear; close all
%% 
PRF = 1e3;
Tstep = 1/PRF;
Nsteps = 1000;
g = 9.8; % m/s
init_angle    = 0;
spin_center    = [1000,1000,0]';
spin_radius = 10; % meters
%% Kinematics
target_acceleration = 10 * g; % m/s
angular_speed = target_acceleration / (spin_radius^2);
init_velocity = spinningVelocity(angular_speed,spin_radius,init_angle);
init_acceleration = spinningAccel(angular_speed,spin_radius,init_angle);
init_pos      = spin_center + spin_radius*[cos(init_angle),sin(init_angle),0]';
%% 

comjet = phased.Platform('InitialPosition', init_pos, ...
    'MotionModel','Velocity', ...
    'InitialVelocity', init_velocity, ...
    'VelocitySource','Input port' ...
    );

accel = init_acceleration;
pos   = init_pos;
vel   = init_velocity;
ang   = init_angle;


figure(1)
hold on;
for i = 1:Nsteps
    plot(pos(1),pos(2),'b*');
    [pos] = comjet.step(Tstep,vel);
    ang = spinningAngle(spin_center,pos);
    vel = spinningVelocity(angular_speed,spin_radius,ang);

end

function velocity = spinningVelocity(angular_speed,spin_radius,spin_angle)
    % 2D
    % angular_speed : positive for clockwise, negative for c.clockwise
    % spin_angle: azimuth....
    velocity  = angular_speed * spin_radius * [-sin(spin_angle) cos(spin_angle) 0]';
end

function accel = spinningAccel(angular_speed,spin_radius,spin_angle)
    accel  = angular_speed * spin_radius^2 * [-cos(spin_angle) -sin(spin_angle) 0]';
end
function ang = spinningAngle(spin_center,current_pos)
    dir_vec = -(spin_center - current_pos);
    ang  = atan2(dir_vec(2),dir_vec(1));
end
