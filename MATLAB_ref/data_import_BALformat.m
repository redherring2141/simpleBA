BALdata_fid = fopen('../data/problem-16-22106-pre.txt', 'rt');
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
BALdata_poses = zeros(NPOSES, 9);
BALdata_pts = zeros(NPTS, 3);

BALdata_obs(:,1) = BALdata_raw_col1(1:NOBS);
BALdata_obs(:,2) = BALdata_raw_col2(1:NOBS);
BALdata_obs(:,3) = BALdata_raw_col3(1:NOBS);
BALdata_obs(:,4) = BALdata_raw_col4(1:NOBS);

BALdata_poses = reshape(BALdata_raw_col1(NOBS+1:NOBS+(NPOSES*9)),3,3,NPOSES);
BALdata_pts = reshape(BALdata_raw_col1(NOBS+(NPOSES*9)+1:end),3,NPTS)';

fclose(BALdata_fid);

%BALdata_obs = dlmread('../data/problem-16-22106-pre.txt', ' ', 1, 0);

%BALdata_raw = textscan(BALdata_fid, '%2c%2c%15c%8c', 4, 'Delimiter', '\n');
