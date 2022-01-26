%
% Image buffer coordinates finder
%
%-------------------- Image coordinates -----------------------
% Author:       Paghalaya
%
% Date 
% and time:     05/02/2014 @ 7:32 PM
%
% Syntax:       [u,v] = IMAGEBUFFERCOORDS (M_ext, P_w, focal_x,...
%                                          focal_y, c_x, c_y)
%
% Description:  Projects world coordinates to the camera's image
%               plane. Using the intrinsic matrix data, world 
%               coordinates are projected onto the image buffer.
%               That is u,v coordinates of the image.
%               
% Inputs:       M_ext    - extrinsic matrix
%
%               P_w      - Homogeneous world coordinates
%
%               focal_x  - Focal length (scaled in X direction)
%                          in pixels
%
%               focal_y  - Focal length (scaled in Y direction)
%                          in pixels
%
%               c_x      - center in X direction of image 
%                          buffer coordinates
%
%               c_y      - center in Y direction of image 
%                          buffer coordinates
%
% Outputs:      u - pixel location in X direction (row index) 
%
%               v - pixel location in Y direction (column index) 
%
% Note:         No hange required!
%
% SEE ALSO:
% CAM2WORLD
%--------------------------------------------------------------
function [u,v] = imagebuffercoords (M_ext, P_w, focal_x, focal_y, c_x, c_y)

% Bar parameters
alpha_bar = M_ext(1,:) * P_w';
beta_bar  = M_ext(2,:) * P_w';
gamma_bar = M_ext(3,:) * P_w';

% Image buffer coordinates
u = round((focal_x * alpha_bar ./ gamma_bar) + c_x);
v = round((focal_y * beta_bar  ./ gamma_bar) + c_y);

end