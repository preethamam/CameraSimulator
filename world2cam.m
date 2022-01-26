%
% Coordinates transformation (world to camera)
%
%---------------------- Transformation ------------------------
% Author:       Paghalaya
%
% Date 
% and time:     06/02/2014 @ 7:05 PM
%
% Syntax:       M_extrinsic  = CAM2WORLD (camera, vehicle)
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
%--------------------------------------------------------------
function  M_ext  = world2cam (camera, vehicle)

    % Vehicle to world transformation
    R_V_W    = rotx(vehicle.rX) * roty(vehicle.rY) * rotz(vehicle.rZ);

    t_Vorg_W = [vehicle.tX; vehicle.tY; vehicle.tZ];

    Tr_V_W   = [R_V_W t_Vorg_W;
                0 0 0 1];

    %  Camera to Vehicle transformation
    R_C_V    = rotx(camera.rX) * roty(camera.rY) * rotz(camera.rZ);

    t_Corg_V = [camera.tX; camera.tY; camera.tZ];

    Tr_C_V   = [R_C_V t_Corg_V;
                0 0 0 1];

    % Camera to world transformation
    Tr_C_W   = Tr_V_W * Tr_C_V;

    % World to camera transformation
    Tr_W_C   = inv(Tr_C_W);

    % Extract the releveant rows of the world to camera transformation
    M_ext    = Tr_W_C(1:3, :);
    
end