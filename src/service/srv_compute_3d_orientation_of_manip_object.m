function [ri] = srv_compute_3d_orientation_of_manip_object(u, reducedProb)
    % 
    ea = [u(end-2*(~reducedProb.isSet):end);0; 0]';
    ri = eul2r(ea(1:3));
