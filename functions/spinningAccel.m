function accel = spinningAccel(angular_speed,spin_radius,spin_angle)
    accel  = angular_speed * spin_radius^2 * [-cos(spin_angle) -sin(spin_angle) 0]';
end