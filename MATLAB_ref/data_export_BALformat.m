%BAL format data export
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
    
    fprintf(fileID_tmp, '%7.6e\n%7.6e\n', 1, 0, 0);
end

for idx_pts=1:NPTS
    fprintf(fileID_tmp, '%7.6e\n%7.6e\n%7.6e\n', points_world(1,idx_pts), points_world(2,idx_pts), points_world(3,idx_pts));
end

fclose(fileID_tmp);