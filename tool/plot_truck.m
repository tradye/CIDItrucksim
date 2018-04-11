global t_sim;
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

prefixname_='truck_';

filename=sprintf('%sx.txt',prefixname_);
fid=fopen(filename);    
TRUCK_X=fscanf(fid,'%f');
fclose(fid);

filename=sprintf('%sy.txt',prefixname_);
fid=fopen(filename);    
TRUCK_Y=fscanf(fid,'%f');
fclose(fid);

filename=sprintf('%slateralerr.txt',prefixname_);
fid=fopen(filename);    
Lateral_ERR=fscanf(fid,'%f');
fclose(fid);

filename=sprintf('%sheadingerr.txt',prefixname_);
fid=fopen(filename);    
Heading_ERR=fscanf(fid,'%f');
fclose(fid);

T=[];
for i = 1:1:(30 * 100 + 1)
    T(i) = 0.01 *i;
end

figure(1)
plot(X,Y,'r');
hold on
plot(TRUCK_X,TRUCK_Y,'b');
xlabel('X(m)');
ylabel('Y(m)');

figure(2)
subplot(2,1,1);
% for i = 1:1:3000
%     plot(T,Lateral_ERR);
% end
plot(T,Lateral_ERR,'g');
xlabel('采样时间 T(s)');
ylabel('lateral_error(m)');

subplot(2,1,2);
% for i = 1:1:3000
%     plot(T,Heading_ERR);
% end
plot(T,Heading_ERR,'r');
xlabel('采样时间 T(s)');
ylabel('heading_error(rad)');



