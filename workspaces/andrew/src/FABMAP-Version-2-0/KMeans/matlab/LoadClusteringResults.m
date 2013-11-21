function[CX,assignments] = LoadClusteringResults(filename,points)

%Points is a vector of size (dim,n)
%for n points

show_plot = true;
show_assignments = true;

sAssignFile = strcat('/path/to/working/directory/',filename,'.points_assignments');
assignments = dlmread(sAssignFile);

sCentresFile = strcat('/path/to/working/directory/',filename,'.points_centres');
CX = dlmread(sCentresFile);

colours = ['b','g','c','m','y','k','r'];

if(show_plot)
    figure;
    hold on;
    if(~show_assignments)
        plot(points(1,:),points(2,:),'b.');
    else        
        for k = 0:length(CX)-1
            plot(points(1,find(assignments == k)),points(2,find(assignments == k)),strcat(colours(1+mod(k,length(colours))),'.'));
        end
    end
    plot(CX(:,1),CX(:,2),'r.');
end
