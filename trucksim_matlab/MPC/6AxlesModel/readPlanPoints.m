% function [X,Y,THETA,V] = readPlanPoints()
function [target_points] = readPlanPoints()

    prefixname='plan_';
% filename=sprintf('%spoints_x.txt',prefixname);
    filename=sprintf('%sx.txt',prefixname);
    fid=fopen(filename);    
    X=fscanf(fid,'%f');
    fclose(fid);

    filename=sprintf('%sy.txt',prefixname);
    fid=fopen(filename);    
    Y=fscanf(fid,'%f');
    fclose(fid);

    filename=sprintf('%sv.txt',prefixname);
    fid=fopen(filename);    
    V=fscanf(fid,'%f');
    fclose(fid);

    filename=sprintf('%stheta.txt',prefixname);
    fid=fopen(filename);    
    THETA=fscanf(fid,'%f');
    fclose(fid);
    
    filename=sprintf('%scurvature.txt',prefixname);
    fid=fopen(filename);    
    Curvature=fscanf(fid,'%f');
    fclose(fid);
    
    filename=sprintf('%scurvature_rate.txt',prefixname);
    fid=fopen(filename);    
    Curvature_change_rate=fscanf(fid,'%f');
    fclose(fid);

    field1 = 'x'; value1 = X;
    field2 = 'y'; value2 = Y;
    field3 = 'theta'; value3 = THETA;
    field4 = 'v'; value4 = V;
    field5 = 'curvature'; value5 = Curvature;
    field6 = 'curvature_change_rate'; value6 = Curvature_change_rate;

    target_points = struct(field1,value1,field2,value2,field3,value3,field4,value4,field5,value5,field6,value6);% target_points is 4*1000 structs
                                                                                                                                                              

end