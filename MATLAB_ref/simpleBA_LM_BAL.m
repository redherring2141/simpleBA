% ba_example.m
% Simple example of bundle adjustment.
% 11/09/2014
% Refer to "A tutorial on SE(3) transformation parameterizations
% and on-manifold optimization" - J.L. Blanco, for explanation of Jacobians

close all;
clear all;
clc;
home;

%%%%% Parameter setting start %%%%%
RANDOM_SAVE = 'LOAD'; % 'SAVE' for random variable gen & save, 'LOAD' for random variable fixe & load
MODE = 'LM' % 'LM' for Levenberg-Marquardt method, 'GN' for Gauss-Newton method
%NPOSES = 4; % fix this for now
%NPTS = 50;
NUM_ITERATIONS = 30;


% generate noisy initial guess poses
ROTATION_NOISE_STD = 0.7/180 * pi;
POSITION_NOISE_STD = 0.5;

% generate noisy initial guess points
POINT_STD = [0.01; 0.01; 0.01];

% std deviation on image noise
FOCAL_LENGTH = 500;
IMAGE_NOISE_STD = 0.3 / FOCAL_LENGTH;

OUTLIER_PROB = 0.1;    % probability of a _bad_ outlier
OUTLIER_IMAGE_NOISE_STD = 30 / FOCAL_LENGTH;
%%%%% Parameter setting end %%%%%


%%%%%%%%%% Data generation start %%%%%%%%%%
%BALdata_fid = fopen('../data/problem-16-22106-pre.txt', 'rt');
%BALdata_fid = fopen('../data/problem-49-7776-pre.txt', 'rt');
BALdata_fid = fopen('../data/toy_ex_test_data.txt', 'rt');
BALdata_header = fscanf(BALdata_fid, '%d %d %d', [3 1]);
NPOSES = BALdata_header(1);
NPTS = BALdata_header(2);
NOBS = BALdata_header(3);

BALdata_cell = textscan(BALdata_fid, '%f%f%f%f', 'HeaderLines', 1, 'Delimiter', '\t');
BALdata_raw_col1 = BALdata_cell{1,1};
BALdata_raw_col2 = BALdata_cell{1,2};
BALdata_raw_col3 = BALdata_cell{1,3};
BALdata_raw_col4 = BALdata_cell{1,4};

BALdata_obs = zeros(NOBS, 4);
BALdata_poses = zeros(3, 3, NPOSES);
BALdata_pts = zeros(NPTS, 3);

BALdata_obs(:,1) = BALdata_raw_col1(1:NOBS);
BALdata_obs(:,2) = BALdata_raw_col2(1:NOBS);
BALdata_obs(:,3) = BALdata_raw_col3(1:NOBS);
BALdata_obs(:,4) = BALdata_raw_col4(1:NOBS);

BALdata_poses = reshape(BALdata_raw_col1(NOBS+1:NOBS+(NPOSES*9)),3,3,NPOSES);
BALdata_pts = reshape(BALdata_raw_col1(NOBS+(NPOSES*9)+1:end),3,NPTS);
cams_ang_noisy = BALdata_poses(:,1,:);
cams_pos_noisy = BALdata_poses(:,2,:);

fclose(BALdata_fid);
%%%%%%%%%% Data generation end %%%%%%%%%%

START_POSE = 1;
NPOSES_OPT = (NPOSES - START_POSE + 1);

wRb_cams_noisy = zeros(3,3,NPOSES);
p_cams_noisy = zeros(3,1,NPOSES);
points_world_noisy = zeros(3,NPTS);

for idx_cam=1:NPOSES
    wRb_cams_noisy(:,:,idx_cam) = eul2rotm(cams_ang_noisy(:,idx_cam)');
    p_cams_noisy(:,1,idx_cam) = cams_pos_noisy(:,idx_cam);
end

points_world_noisy = reshape(BALdata_pts, 3, NPTS);
points_image_noisy = zeros(3, NPTS, NPOSES);
points_image_noisy(1:2,:,:) = reshape(BALdata_obs(:,3:4)', 2, NPTS, NPOSES);
points_image_noisy(3,:,:) = 1;



% plot point cloud
f3d = figure;
hold on;
scatter3(points_world_noisy(1,:), points_world_noisy(2,:), points_world_noisy(3,:), 'b');
title('Simulated 3D point cloud');  
grid on;
axis equal;
axis vis3d;




%%%%% Initial guess by triangulation start %%%%%
t_start = cputime;
% estimated poses
wRb_cams_estimate = wRb_cams_noisy;
p_cams_estimate = p_cams_noisy;

% triangulate initial guesses on all features w/ least squares
points_world_estimate = zeros(3,1,NPTS);
for i=1:NPTS
    A = zeros(3,3);
    b = zeros(3,1);
    
    % all observations of this feature, normalized
    u = squeeze( points_image_noisy(:,i,:) );
    u = bsxfun(@rdivide, u, sqrt(sum(u.^2,1)));
    
    for j=1:NPOSES
        % rotate into world
        v = wRb_cams_estimate(:,:,j) * u(:,j);
        B = eye(3,3) - v*v';
        A = A + B;
        p_cams_tmp = p_cams_estimate(:,:,j);
        tmp=B*p_cams_estimate(:,:,j);
        b = b + B*p_cams_estimate(:,:,j);
    end
    
    % solve
    points_world_estimate(:,:,i) = A\b;
end
% plot estimated 3D points
figure(f3d);
scatter3(points_world_estimate(1,:), points_world_estimate(2,:), points_world_estimate(3,:), 'r');

% plot estimated camera SRTs
for j=1:NPOSES
    wRb = wRb_cams_estimate(:,:,j);
    cPo = p_cams_estimate(:,:,j);
    
    zcam = wRb * [0;0;1];
    xcam = wRb * [1;0;0];
    ycam = wRb * [0;1;0];
end

% find best point 
point_deltas = points_world_noisy - squeeze(points_world_estimate);
point_deltas = sqrt(sum(point_deltas.^2,1));
[~,best_point_idx] = min(point_deltas);

% convert poses to SE3
cam_pose_estimate = zeros(4,4,NPOSES);
for j=1:NPOSES
    wRb = wRb_cams_estimate(:,:,j);
    p = p_cams_estimate(:,:,j);
    tmp_wRbp = -wRb'*p;
    cam_pose_estimate(:,:,j) = [wRb' -wRb'*p; 0 0 0 1];
end
t_elapsed=cputime-t_start
%%%%% Initial guess by triangulation end %%%%%






%%%%%%%%%% Bundle adjustment start %%%%%%%%%%
% run bundle adjustment
%%%%% Initial value assignment start %%%%%
r = zeros(NPTS*NPOSES*2, 1);
numJRows = 2*NPTS*NPOSES;
numJCols = NPTS*3+NPOSES_OPT*6;
J = zeros(NPTS*NPOSES*2, NPTS*3 + NPOSES_OPT*6);

for i=1:NPTS
    p_world = points_world_estimate(:,:,i);
    for j=1:NPOSES
        % camera pose
        H_cam = cam_pose_estimate(:,:,j);
        
        % transform to camera
        p_cam = H_cam * [p_world; 1];
        p_cam = p_cam(1:3); % truncate to remove 1
        
        xc = p_cam(1);  % camera coordinate
        yc = p_cam(2);
        zc = p_cam(3);
        
        % projection jacobian (2x3)
        Jproj = [1/zc 0 -xc/(zc*zc); 
                0 1/zc -yc/(zc*zc)]; 
        
        % project to image coordinates and calculate residual
        h_est = p_cam / zc;      
        row = (j-1)*NPTS*2 + (i-1)*2 + 1;
        r(row:row+1,1) = points_image_noisy(1:2,i,j) - h_est(1:2);
        
        % pose jacobian (3x6)
        Jpose = [eye(3,3) -skew3(p_cam)];
        
        % point jacobian (3x3)
        Jpoint = H_cam(1:3,1:3);
        
        % insert jacobians
        if (j >= START_POSE)
            % optimizing pose also
            cols_pose = NPTS*3 + (j-START_POSE)*6 + 1;
            cols_pose = cols_pose:(cols_pose+5);
            J(row:row+1, cols_pose) = Jproj * Jpose;
        else
            % optimizing only point
        end
        
        cols_point = (i-1)*3 + 1;
        cols_point = cols_point:(cols_point+2);
        J(row:row+1, cols_point) = Jproj * Jpoint;
    end
end
% calculate cauchy weights
r2 = r.*r;
sigsqrd = mean(r2);
W = 1 ./ (1 + r2/sigsqrd);
W = diag(W);

% calculate update (slow and simple method)
H = J'*W*J;
damper = diag(diag(H));
g = J'*W*r;
lambda = 0.00001; % lambda = tau*max(damper)

% we will optimize only the poses from START_POSE to NPOSES (inclusive)
r_temp = zeros(NPTS*NPOSES*2, 1);
err = 10000;
err_prev = 10000;
v = 2;
cam_pose_estimate_temp = cam_pose_estimate;
points_world_estimate_temp = points_world_estimate;
%%%%% Initial value assignment end %%%%%

%%%%% Nonlinear least square optimization start %%%%%
t_start=cputime;
for iter=1:NUM_ITERATIONS
    fprintf('Iter %i, magnitude %f\n', iter, norm(r));

    if (strcmp(MODE, 'LM'))
        H_lm = H + lambda*damper;
        dx = H_lm\g;
    else
        dx = H\g;
    end

    norm_dx = norm(dx);

    % update points
    dx_points = dx(1:(NPTS*3),:);
    dx_points = reshape(dx_points,3,1,size(dx_points,1)/3);
    points_world_estimate_temp = points_world_estimate + dx_points;
    
    % update poses
    dx_poses = dx((NPTS*3 + 1):end,:);
    dx_poses = reshape(dx_poses,6,size(dx_poses,1)/6);
    for j=START_POSE:NPOSES
        twist = dx_poses(:,j - START_POSE + 1);
        % approximate the exponential map
        S = skew3(twist(4:6));
        V = eye(3,3) + (1/2)*S + (1/6)*S*S;
        update = [rodrigues(twist(4:6)) V*twist(1:3); 0 0 0 1];
        cam_pose_estimate_temp(:,:,j) = update * cam_pose_estimate(:,:,j);
    end

    for i=1:NPTS
        p_world = points_world_estimate_temp(:,:,i);
        for j=1:NPOSES
            H_cam = cam_pose_estimate_temp(:,:,j);
            p_cam = H_cam * [p_world; 1];
            p_cam = p_cam(1:3);
            zc = p_cam(3);
            h_est = p_cam/zc;
            row = (j-1)*NPTS*2 + (i-1)*2 + 1;
            %r_temp(row:row+1,1) = points_image_noisy(1:2,i,j)-h_est(1:2);
            r(row:row+1,1) = points_image_noisy(1:2,i,j)-h_est(1:2);
        end
    end

    err = r'*r;
    rho = err_prev-err;
    err_prev = err;

    if (strcmp(MODE, 'LM'))
        if (rho > 0)
            points_world_estimate = points_world_estimate_temp;
            cam_pose_estimate = cam_pose_estimate_temp;           
            %lambda = lambda/2;
            lambda = lambda*max(1/3, 1-(2*rho-1)^3); % mu:=mu*max(1/3,1-(2rho-1)^3)
            v = 2;          
        else
            %lambda = lambda*2;
            lambda = lambda*v;
            v = 2*v;
        end
    else
        points_world_estimate = points_world_estimate_temp;
        cam_pose_estimate = cam_pose_estimate_temp;
    end

    for i=1:NPTS
        p_world = points_world_estimate(:,:,i);
        for j=1:NPOSES
            % camera pose
            H_cam = cam_pose_estimate(:,:,j);
            
            % transform to camera
            p_cam = H_cam * [p_world; 1];
            p_cam = p_cam(1:3); % truncate to remove 1
            
            xc = p_cam(1);  % camera coordinate
            yc = p_cam(2);
            zc = p_cam(3);
            
            % projection jacobian (2x3)
            Jproj = [1/zc 0 -xc/(zc*zc); 
                    0 1/zc -yc/(zc*zc)]; 
            
            % project to image coordinates and calculate residual
            h_est = p_cam / zc;      
            row = (j-1)*NPTS*2 + (i-1)*2 + 1;
            r(row:row+1,1) = points_image_noisy(1:2,i,j) - h_est(1:2);
            
            % pose jacobian (3x6)
            Jpose = [eye(3,3) -skew3(p_cam)];
            
            % point jacobian (3x3)
            Jpoint = H_cam(1:3,1:3);
            
            % insert jacobians
            if (j >= START_POSE)
                % optimizing pose also
                cols_pose = NPTS*3 + (j-START_POSE)*6 + 1;
                cols_pose = cols_pose:(cols_pose+5);
                J(row:row+1, cols_pose) = Jproj * Jpose;
            else
                % optimizing only point
            end
            
            cols_point = (i-1)*3 + 1;
            cols_point = cols_point:(cols_point+2);
            J(row:row+1, cols_point) = Jproj * Jpoint;
        end
    end    
    % calculate cauchy weights
    r2 = r.*r;
    sigsqrd = mean(r2);
    W = 1 ./ (1 + r2/sigsqrd);
    W = diag(W);

    % calculate update (slow and simple method)
    H = J'*W*J;
    damper = diag(diag(H));
    g = J'*W*r;    
end
%%%%% Nonlinear least square optimization end %%%%%

t_elapsed=cputime-t_start
Avg_time = t_elapsed/NUM_ITERATIONS
%%%%%%%%%% Bundle adjustment end %%%%%%%%%%


% convert poses back to R,p form
for j=1:NPOSES
    H = cam_pose_estimate(:,:,j);
    wRb = H(1:3,1:3)';
    p = -wRb * H(1:3,4);    
    wRb_cams_estimate(:,:,j) = wRb;
    p_cams_estimate(:,:,j) = p;
end

% plot adjusted points
figure(f3d);
scatter3(points_world_estimate(1,:), points_world_estimate(2,:), points_world_estimate(3,:), 'g');


% plot noisy camera SRTs
for j=1:NPOSES
    wRb_new = wRb_cams_estimate(:,:,j);
    cPo_new = p_cams_estimate(:,:,j);
    
    zcam_new = wRb_new * [0;0;1];
    xcam_new = wRb_new * [1;0;0];
    ycam_new = wRb_new * [0;1;0];

    % camera vector
    h = quiver3(cPo_new(1),cPo_new(2),cPo_new(3),zcam_new(1)*0.5,zcam_new(2)*0.5,zcam_new(3)*0.5,'g');
    set(h,'linewidth',2);
    h = quiver3(cPo_new(1),cPo_new(2),cPo_new(3),xcam_new(1)*0.5,xcam_new(2)*0.5,xcam_new(3)*0.5,'g');
    set(h,'linewidth',2);
    h = quiver3(cPo_new(1),cPo_new(2),cPo_new(3),ycam_new(1)*0.5,ycam_new(2)*0.5,ycam_new(3)*0.5,'g');
    set(h,'linewidth',2);
end




% output positions of cameras after adjustment
disp(squeeze(p_cams));
disp(squeeze(p_cams_noisy));
disp(squeeze(p_cams_estimate));



%%%%% BAL format data export start %%%%%
cams_ang_noisy = zeros(3,NPOSES);
cams_pos_noisy = zeros(3,NPOSES);
fileID_tmp = fopen('../data/toy_ex_test_data.txt', 'w');
fprintf(fileID_tmp, '%d %d %d\n', NPOSES, NPTS, NPOSES*NPTS);
for idx_cam=1:NPOSES
    for idx_pts=1:NPTS
        fprintf(fileID_tmp, '%d %d\t%7.6e %7.6e\n', idx_cam-1, idx_pts-1, points_image_noisy(1, idx_pts, idx_cam), points_image_noisy(2, idx_pts, idx_cam));
    end
end

for idx_cam=1:NPOSES
    cams_ang_noisy(:,idx_cam) = rotm2eul(wRb_cams_noisy(:,:,idx_cam));
    cams_pos_noisy(:,idx_cam) = p_cams_noisy(:,1,idx_cam);
end

for idx_cam=1:NPOSES   
    fprintf(fileID_tmp, '%7.6e\n%7.6e\n', cams_ang_noisy(1,idx_cam), cams_ang_noisy(2,idx_cam), cams_ang_noisy(3,idx_cam));
    fprintf(fileID_tmp, '%7.6e\n%7.6e\n', cams_pos_noisy(1,idx_cam), cams_pos_noisy(2,idx_cam), cams_pos_noisy(3,idx_cam));
    
    fprintf(fileID_tmp, '%7.6e\n%7.6e\n', 30/500, 0, 0);
end

for idx_pts=1:NPTS
    fprintf(fileID_tmp, '%7.6e\n%7.6e\n%7.6e\n', points_world_noisy(1,idx_pts), points_world_noisy(2,idx_pts), points_world_noisy(3,idx_pts));
end

fclose(fileID_tmp);
%%%%% BAL format data export end %%%%%