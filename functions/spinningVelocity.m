function velocity = spinningVelocity(angular_speed,spin_radius,spin_angle)
    % 2D
    % angular_speed : positive for clockwise, negative for c.clockwise
    % spin_angle: azimuth....
    velocity  = angular_speed * spin_radius * [-sin(spin_angle) cos(spin_angle) 0]';
end