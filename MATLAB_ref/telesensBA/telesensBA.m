clear all;
close all;
clc;

load('../randn_angs_NPOSES.mat');
load('../randn_pos_NPOSES.mat');
load('../randn_pts_world_NPTS.mat');

load('../randn_pts_img_noisy_NPTS_NPOSES.mat');
load('../outlier_idx_NPOSES_NPTS.mat');
load('../randn_nnz_outliers_NPOSES.mat');

objPts = reshape(randn_pts_world_NPTS,3,50);
poses_ang = reshape(randn_angs_NPOSES,3,4);
poses_pos = reshape(randn_pos_NPOSES,3,4);
%fu0v0 = [1 1 1 1; 0 0 0 0; 0 0 0 0];
%poses = [poses_ang; poses_pos; fu0v0];
poses = [poses_ang; poses_pos];
imgPts = reshape(randn_pts_img_noisy_NPTS_NPOSES,

%[poses_new, camParams]BAImple_No_ObjPoints(objPts, poses, imgPts)