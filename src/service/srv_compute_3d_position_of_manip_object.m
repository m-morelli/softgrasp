function [ti] = srv_compute_3d_position_of_manip_object(u, reducedProb)
    % 
    xyz = [u(1:3-1*(reducedProb.isSet));0];
    ti = xyz(1:3);
