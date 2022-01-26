%
% Perspective projection function (world to camera)
%
%---------------------- Projection function ------------------------
% Author:       Paghalaya
%
% Date 
% and time:     06/11/2014 @ 8:32 PM
%
% Syntax:       [u, v]  = EXTRINSICMATRIX (camera, vehicle)
%
% Description:  Gets you all extrinsic matrix of the camera.
%               M_extrinsic is a 3 x 4 matrix. This matrix
%               represents world to camera transformation
%               
% Inputs:       struct(camera): camera.rX - rotation in x
%                               camera.rY - rotation in y
%                               camera.rZ - rotation in z
%                               camera.tX - translation in x
%                               camera.tY - translation in y
%                               camera.tZ - translation in z
%
%               struct(vehicle): vehicle.rX - rotation in x
%                                vehicle.rY - rotation in y
%                                vehicle.rZ - rotation in z
%                                vehicle.tX - translation in x
%                                vehicle.tY - translation in y
%                                vehicle.tZ - translation in z
%               
%
% Outputs:      M_extrinsic - extrinsic matrix
%
% Note:        No change required!
%
% SEE ALSO:
% ROTX, ROTY, ROTZ, INV
%-------------------------------------------------------------------
function  [u, v]  = perspectiveprojection (tranvec, worldpoint, f_x, f_y, c_x, c_y)
    
    % tranvec.alpha bar variable
    alpha_bar = worldpoint.X_w * (cosd(tranvec.alpha) * cosd(tranvec.beta)) - worldpoint.Y_w * (cosd(tranvec.beta) * sind(tranvec.alpha)) + ...
                        worldpoint.Z_w * sind(tranvec.beta) + tranvec.t_x;
    
    % tranvec.beta bar variable
    beta_bar  = worldpoint.X_w * (cosd(tranvec.gamma) * sind(tranvec.alpha)  + cosd(tranvec.alpha) * sind(tranvec.beta) * sind(tranvec.gamma)) + ...
                        worldpoint.Y_w * (cosd(tranvec.alpha) * cosd(tranvec.gamma)  - sind(tranvec.alpha) * sind(tranvec.beta) * sind(tranvec.gamma)) - ...
                        worldpoint.Z_w * (cosd(tranvec.beta)  * sind(tranvec.gamma)) + tranvec.t_y;
            
    % tranvec.gamma bar variable
    gamma_bar = worldpoint.X_w * (sind(tranvec.alpha) * sind(tranvec.gamma)  - cosd(tranvec.gamma) * cosd(tranvec.gamma) * sind(tranvec.beta)) + ...
                        worldpoint.Y_w * (cosd(tranvec.gamma) * sind(tranvec.gamma)  + cosd(tranvec.gamma) * sind(tranvec.alpha) * sind(tranvec.beta)) + ...
                        worldpoint.Z_w * (cosd(tranvec.beta)  * cosd(tranvec.gamma)) + tranvec.t_z;
                    
    % Function --> f(alpha, beta, gamma, t_x, t_y, t_z)
    u = f_x * (alpha_bar / gamma_bar) + c_x;
    
    v = f_y * (beta_bar / gamma_bar)  + c_y;
end