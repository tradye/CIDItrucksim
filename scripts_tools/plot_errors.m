% M1 = csvread('/home/cidi-xh/xuhao/tmpfile/lqr_steer_angle.csv');
% M = dlmread('/home/cidi-xh/xuhao/tmpfile/test.csv');%both of two ways OK
% M1 = dlmread('/home/cidi-xh/xuhao/files/调试数据/20180614/lqr_steer_angle_1.5_+_0.02.csv');
% 
% control_time = M1(:,1);
% steer_angle = M1(:,2);
% steer_angle_feedback = M1(:,3);
% steer_angle_feedforward = M1(:,4);
% lateral_error = M1(:,5);
% heading_error = M1(:,6);
% 
% figure(1)
% subplot(3,1,1);    
% plot(control_time,steer_angle,'g');
% xlabel('header time');
% ylabel('lqr steer angle (deg)');
% hold on
% subplot(3,1,2);    
% plot(control_time,steer_angle_feedback,'r');
% xlabel('header time');
% ylabel('lqr steer angle feedback (deg)');
% hold on
% subplot(3,1,3);    
% plot(control_time,steer_angle_feedforward,'b');Vehicle model identification byintegrated prediction error minimization
% xlabel('header time');
% ylabel('lqr steer angle feedforward (deg)');
% hold on

% M2 = dlmread('/home/cidi-xh/xuhao/tmpfile/record_for_mpc_test_no_title_1.csv');
% 
% control_time_2 = M2(:,1);
% steer_angle_2 = M2(:,2);
% steer_angle_feedback_2 = M2(:,3);
% steer_angle_feedforward_2 = M2(:,4);
% lateral_error_2 = M2(:,5);
% heading_error_2 = M2(:,6);
% 
% figure(2)
% subplot(3,1,1);    
% plot(control_time_2,steer_angle_2,'g');
% xlabel('header time');
% ylabel('cidi mpc steer angle (deg)');
% hold on
% subplot(3,1,2);    
% plot(control_time_2,steer_angle_feedback_2,'r');
% xlabel('header time');
% ylabel('cidi mpc steer angle feedback (deg)');
% hold on
% subplot(3,1,3);    
% plot(control_time_2,steer_angle_feedforward_2,'b');
% xlabel('header time');
% ylabel('cidi mpc steer angle feedforward (deg)');

% M1 = dlmread('/home/cidi-xh/xuhao/files/调试数据/20180614/lqr_steer_angle_1.5_+_0.02.csv');
fid = fopen('/home/cidi-xh/xuhao/files/调试数据/20180614/lqr_steer_angle_1.5_+_0.02.csv');%query time1.5，正前馈，Q(1,1)= 0.02
cell_title = textscan(fid, '%s %s %s %s %s %s ',1 , 'delimiter', ',');
cell_data = textscan(fid, '%.4f %.4f %.4f %.4f %.4f %.4f','delimiter', ',');
fclose(fid);

data = cell2mat(cell_data);
control_time = data(:,1);
steer_angle = data(:,2);
steer_angle_feedback = data(:,3);
steer_angle_feedforward = data(:,4);
lateral_error = data(:,5);
heading_error = data(:,6);
%%%%%%%%
fid1 = fopen('/home/cidi-xh/xuhao/files/调试数据/20180614/lqr_steer_angle_1.5_+.csv');%query time1.5，正前馈，Q(1,1)= 0.01
cell_title1 = textscan(fid1, '%s %s %s %s %s %s ',1 , 'delimiter', ',');
cell_data1 = textscan(fid1, '%.4f %.4f %.4f %.4f %.4f %.4f','delimiter', ',');
fclose(fid1);

data1 = cell2mat(cell_data1);
control_time1 = data1(:,1);
steer_angle1 = data1(:,2);
steer_angle_feedback1 = data1(:,3);
steer_angle_feedforward1 = data1(:,4);
lateral_error1 = data1(:,5);
heading_error1 = data1(:,6);
num = size(control_time,1);
num1 = size(control_time1,1);
littler = litter(num,num1);
for i = 1:1: littler
    T(i) = i;
end 
%%%%%%%%
fid = fopen('/home/cidi-xh/xuhao/files/调试数据/20180614/lqr_steer_angle_0.2.csv');%query time0.2
cell_title2 = textscan(fid, '%s %s %s %s %s %s ',1 , 'delimiter', ',');
cell_data2 = textscan(fid, '%.4f %.4f %.4f %.4f %.4f %.4f','delimiter', ',');
fclose(fid);

data2 = cell2mat(cell_data2);
control_time2 = data2(:,1);
steer_angle2 = data2(:,2);
steer_angle_feedback2 = data2(:,3);
steer_angle_feedforward2 = data2(:,4);
lateral_error2 = data2(:,5);
heading_error2 = data2(:,6);

num2 = size(control_time2,1);
%%%%%%%%
fid = fopen('/home/cidi-xh/xuhao/files/调试数据/20180614/lqr_steer_angle_0.8.csv');%query time0.8
cell_title3 = textscan(fid, '%s %s %s %s %s %s ',1 , 'delimiter', ',');
cell_data3 = textscan(fid, '%.4f %.4f %.4f %.4f %.4f %.4f','delimiter', ',');
fclose(fid);

data3 = cell2mat(cell_data3);
control_time3 = data3(:,1);
steer_angle3 = data3(:,2);
steer_angle_feedback3 = data3(:,3);
steer_angle_feedforward3 = data3(:,4);
lateral_error3 = data3(:,5);
heading_error3 = data3(:,6);

num3 = size(control_time3,1);
%%%%%%%%
fid = fopen('/home/cidi-xh/xuhao/files/调试数据/20180614/lqr_steer_angle_1.5.csv');%query time1.5
cell_title4 = textscan(fid, '%s %s %s %s %s %s ',1 , 'delimiter', ',');
cell_data4 = textscan(fid, '%.4f %.4f %.4f %.4f %.4f %.4f','delimiter', ',');
fclose(fid);

data4 = cell2mat(cell_data4);
control_time4 = data4(:,1);
steer_angle4 = data4(:,2);
steer_angle_feedback4 = data4(:,3);
steer_angle_feedforward4 = data4(:,4);
lateral_error4 = data4(:,5);
heading_error4 = data4(:,6);

num4 = size(control_time4,1);

littler1 = litter(num4,num3);
for i = 1:1: littler1
    T1(i) = i;
end 
figure(1)
plot(T,lateral_error,'r');
hold on
plot(T,lateral_error1(1:littler,1),'b');

figure(2)
% plot(T1,lateral_error2,'r');
% hold on
plot(T1,lateral_error3(1:littler1,1),'b');
hold on
plot(T1,lateral_error4(1:littler1,1),'g');

%%%%%%%%
fid = fopen('/home/cidi-xh/xuhao/files/调试数据/20180614/lqr_steer_angle.csv');%前馈符号相减，控制参数默认，只是query time改为1.5
cell_title5 = textscan(fid, '%s %s %s %s %s %s ',1 , 'delimiter', ',');
cell_data5 = textscan(fid, '%.4f %.4f %.4f %.4f %.4f %.4f','delimiter', ',');
fclose(fid);

data5 = cell2mat(cell_data5);
control_time5 = data5(:,1);
steer_angle5 = data5(:,2);
steer_angle_feedback5 = data5(:,3);
steer_angle_feedforward5 = data5(:,4);
lateral_error5 = data5(:,5);
heading_error5 = data5(:,6);

num5 = size(control_time5,1);

littler2 = litter(num4,num5);
for i = 1:1: littler2
    T2(i) = i;
end 

figure(3)
% plot(T1,lateral_error2,'r');
% hold on
plot(T2,lateral_error4(1:littler2,1),'b');
hold on
plot(T2,lateral_error5(1:littler2,1),'g');

%%%%%%%%
fid = fopen('/home/cidi-xh/xuhao/files/调试数据/20180614/lqr_steer_angle_mannual.csv');%手动驾驶
cell_title6 = textscan(fid, '%s %s %s %s %s %s ',1 , 'delimiter', ',');
cell_data6 = textscan(fid, '%.4f %.4f %.4f %.4f %.4f %.4f','delimiter', ',');
fclose(fid);

data6 = cell2mat(cell_data6);
control_time6 = data6(:,1);
steer_angle6 = data6(:,2);
steer_angle_feedback6 = data6(:,3);
steer_angle_feedforward6 = data6(:,4);
lateral_error6 = data6(:,5);
heading_error6 = data6(:,6);

num6 = size(control_time6,1);
figure(4)
plot(control_time6,lateral_error6,'g');
