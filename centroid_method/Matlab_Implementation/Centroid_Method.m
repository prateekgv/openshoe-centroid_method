function [pos_pseudo_measurement_r pos_pseudo_measurement_l]=Centroid_Method(p_r,p_l,p_0)

% Global struct holding the simulation settings
global simdata

% Distance between the position estimates of right and left foot INS
d_h = norm(p_r-p_l);

% Estimates after applying range constraints
pos_pseudo_measurement_r = (simdata.range_constraint*p_r - simdata.range_constraint*p_l + 2*d_h*p_0)/(2*d_h);
pos_pseudo_measurement_l = (simdata.range_constraint*p_l - simdata.range_constraint*p_r + 2*d_h*p_0)/(2*d_h);

end
