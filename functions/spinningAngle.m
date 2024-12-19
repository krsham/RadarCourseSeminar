function ang = spinningAngle(spin_center,current_pos)
    dir_vec = -(spin_center - current_pos);
    ang  = atan2(dir_vec(2),dir_vec(1));
end
