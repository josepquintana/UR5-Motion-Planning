%this method outputs a file with all the moves for UR software

function OutputMovesForUR (path, filename)
    %open the file
    file = fopen(filename, 'w');
    %output frames from begin to end (0->1)
    for key = 1 : size(path,1)
    	fprintf(file, 'movej([%g, %g, %g, %g, %g, %g], a=1, v=1, t=0, r=0)\n', path(key, 1), path(key, 2), path(key, 3), path(key, 4), path(key, 5), path(key, 6));
    end
    %wait for 2 seconds
    fprintf(file, 'sleep(2.0)\n');
    %output frames from end to begin (1->0)
    for key = size(path,1) : -1 : 1
        fprintf(file, 'movej([%g, %g, %g, %g, %g, %g], a=1, v=1, t=0, r=0)\n', path(key, 1), path(key, 2), path(key, 3), path(key, 4), path(key, 5), path(key, 6));
    end
    %wait for 2 seconds
    fprintf(file, 'sleep(2.0)\n');
    %close the file
    fclose(file);
end