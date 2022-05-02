fileID = fopen('./H_MATLAB.txt', 'w');
fprintf(fileID, '%5.4f\n', H);
fclose(fileID);