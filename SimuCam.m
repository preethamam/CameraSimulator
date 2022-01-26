%//%************************************************************************%
%//%*                              Ph.D                                    *%
%//%*                        Camera Simulator						       *%
%//%*                                                                      *%
%//%*             Name: Preetham Aghalaya Manjunatha    		           *%
%//%*             USC ID Number: 7356627445		                           *%
%//%*             USC Email: aghalaya@usc.edu                              *%
%//%*             Submission Date: --/--/2012                              *%
%//%************************************************************************%
%//%*             Viterbi School of Engineering,                           *%
%//%*             Sonny Astani Dept. of Civil Engineering,                 *%
%//%*             University of Southern california,                       *%
%//%*             Los Angeles, California.                                 *%
%//%************************************************************************%

%% Start
% Start clock and clean workspace
tic;
clear all; close all; clc;

%% Inputs
% Focal lengths
focal_x = 600; %604.755931512397; 
focal_y = 600; %602.949183850974;

% Image size
image_x = 1280;
image_y = 720;

% Center of image buffer
c_x = image_x / 2; % SoftKinetic - 310.145564199017
c_y = image_y / 2; % SoftKinetic - 247.712239470423

%% World points/objects
% Cross mark
xline = 7:0.01:8;
yline = 5:0.01:6;

crossX = [xline', 10 * ones(size(xline')), ...
                0 * ones(size(xline')), ...
                ones(size(xline'))];
            
crossY = [2 * ones(size(xline')), yline' ...
                0 * ones(size(xline')), ...
                ones(size(xline'))];
            
% Curve
t = 0:0.01:2*pi;
a = 1; b = 1;
xcircle = a * cos(t);
ycircle = b * sin(t);

circcoord = [xcircle', ycircle', zeros(size(ycircle')), ones(size(ycircle'))];

% World points
P_w = [crossX; crossY; circcoord];

%% Noise for location
delta = 0:0.1:2.5;

%% Extrinsic matrix
uvmatold = zeros(size(P_w,1), 2);

for itr = 1:numel(delta)
    % Camera struct
    camera.rX = -120;
    camera.rY = 0;
    camera.rZ = 0;
    camera.tX = 0;
    camera.tY = 1;
    camera.tZ = 2;

    % Vehicle struct
    vehicle.rX = 0;
    vehicle.rY = 0;
    vehicle.rZ = 0 + delta(itr);
    vehicle.tX = 2 ;
    vehicle.tY = -5;
    vehicle.tZ = 2;

    % Get extrinsic matrix
    M_ext  = world2cam (camera, vehicle);

    %% Image buffer coordinates
    [u,v] = imagebuffercoords (M_ext, P_w, focal_x, focal_y, c_x, c_y);
    if (itr == 1)
        uvmatone = [u', v'];
    end
    error(itr) = sqrt (mse (uvmatone - [u', v']));

    %% Plot
    I = zeros (image_y, image_x);
    for i = 1:numel(u)
        if ~( (v(i) <= 0 || u(i) <= 0) ||  (v(i) > image_y || u(i) > image_x) )
            I(v(i), u(i)) = 1;
        end
    end

    % Display image
    imshow(I); impixelinfo
    
    % Motion figure
    drawnow;
    
    % Pause figure
    pause(0.5)
end

%% Error plot
figure;
plot (delta(1:end), error(1:end), '*-')
grid on;
xlabel ('\delta (world units)')
ylabel ('Mean Square Error (pixels)')

%% End
% Get runtime
Runtime = toc;


