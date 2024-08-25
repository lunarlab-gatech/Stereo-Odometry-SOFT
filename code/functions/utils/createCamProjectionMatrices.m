function [P0, P1, P2, P3] = createCamProjectionMatrices(cam_params)
%CREATECAMPROJECTIONMATRICES Creates projection matrices for a stereo
%camera. It is assumed that the parameters are for rectified images.
%
% Input:
%   - cam_params: structure comprising of fx, fy, cx, cy, baseline 
%
% Output:
%   - P0(3, 4): projection matrix for the left camera
%   - P1(3, 4): projection matrix for the right camera
%   - P2(3, 4): projection matrix for the left camera
%   - P3(3, 4): projection matrix for the right camera


% for left camera
P0 = [cam_params.fx, 0, cam_params.cx, 0; ...
      0, cam_params.fy, cam_params.cy, 0; ...
      0, 0, 1, 0];

% for right camera
P1 = [cam_params.fx, 0, cam_params.cx, - cam_params.base; ...
      0, cam_params.fy, cam_params.cy, 0; ...
      0, 0, 1, 0];

% for left camera (P2)
P2 = [cam_params.fx, 0, cam_params.cx, cam_params.P2_ftx; ...
      0, cam_params.fy, cam_params.cy, cam_params.P2_fty; ...
      0, 0, 1, 0];

% for right camera (P3)
P3 = [cam_params.fx, 0, cam_params.cx, cam_params.P3_ftx; ...
      0, cam_params.fy, cam_params.cy, cam_params.P3_fty; ...
      0, 0, 1, 0];
end

