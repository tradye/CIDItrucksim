run('LoadTestPaths');
funcWriteCidiPathPlan(pathStraightFast, 'sample_path_structs/pathStraightFast');
funcWriteCidiPathPlan(pathStraightSlow, 'sample_path_structs/pathStraightSlow');
funcWriteCidiPathPlan(pathLCFast, 'sample_path_structs/pathLCFast');
funcWriteCidiPathPlan(pathLCSlow, 'sample_path_structs/pathLCSlow');
run('LoadTestPaths_draft');
funcWriteCidiPathPlan(pathLaneChange_double_f, 'sample_path_structs/pathLaneChange_double_f');
funcWriteCidiPathPlan(pathLaneChange_double_s, 'sample_path_structs/pathLaneChange_double_s');
funcWriteCidiPathPlan(pathCircle_large_fast, 'sample_path_structs/pathCircle_large_fast');
funcWriteCidiPathPlan(pathCircle_small_slow, 'sample_path_structs/pathCircle_small_slow');
funcWriteCidiPathPlan(pathFigureEight, 'sample_path_structs/pathFigureEight');
funcWriteCidiPathPlan(pathGentleHighwayCurve, 'sample_path_structs/pathGentleHighwayCurve');

function funcWriteCidiPathPlan(inCidiPath, dir)

    const_prefixname='plan_';
    mkdir(dir);
    
    % readCidiPath and write to a series of text files
    target_points = readCidiPathPoints(inCidiPath);

    field_names = fieldnames(target_points);
    disp(field_names);
    for field_name = field_names'
    write_field_name = field_name{1};
        if strcmp(write_field_name,'curvature_change_rate')
            write_field_name = 'curvature_rate'; 
        end %strcmp
        filename = sprintf('./%s/%s%s.txt', ...
            dir, const_prefixname, write_field_name);
        disp(filename);
        fid=fopen(filename,'w');
        fprintf(fid,'%d\n',target_points.(field_name{1}));
        fclose(fid);
    end
        

    
end % writeCidiPathToPlan