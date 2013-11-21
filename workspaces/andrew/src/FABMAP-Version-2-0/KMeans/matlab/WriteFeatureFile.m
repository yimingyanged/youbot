function WriteFeatureFile(filename,points)

%Points is a vector of size (dim,n)
%for n points

sFilename = strcat('/path/to/working/directory/',filename,'.points');
fid = fopen(sFilename,'wt');

dim = size(points,1);
num_points = size(points,2);

fprintf(fid,'%d\n',dim);
fprintf(fid,'%d\n',num_points);

%for k = 1:size(points,2)
%    fprintf(fid,'%s\n',num2str(points(:,k)','%0.5g '));
%end

fclose(fid);

%Now write the matrix
dlmwrite(sFilename,points','-append','delimiter',' ','newline','pc');

