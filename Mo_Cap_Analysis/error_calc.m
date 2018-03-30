%% Daniel Galindo-Navarro ES100 Motion Capture analysis
%% load motion capture data
load('Deadlift0001.mat');
load('Squat0001.mat');


% pitch angles for DL
a_D_mcap = Deadlift0001.RigidBodies.RPYs(1,2,:);
a_D_mcap = reshape(a_D_mcap, [1, Deadlift0001.Frames]);
b_D_mcap = Deadlift0001.RigidBodies.RPYs(2,2,:);
b_D_mcap = reshape(b_D_mcap, [1, Deadlift0001.Frames]);

t_D_mcap = linspace(1, (1/Deadlift0001.FrameRate)*Deadlift0001.Frames, Deadlift0001.Frames); 

% pitch angles for SQ
a_S_mcap = Squat0001.RigidBodies.RPYs(1,2,:);
a_S_mcap = reshape(a_S_mcap, [1, Squat0001.Frames]);
b_S_mcap = Squat0001.RigidBodies.RPYs(2,2,:);
b_S_mcap = reshape(b_S_mcap, [1, Squat0001.Frames]);

t_S_mcap = linspace(1, (1/Squat0001.FrameRate)*Squat0001.Frames, Squat0001.Frames);

% flip, scale data to match sensor data.
% squat
off = 68.18;

a_S_mcap = -(a_S_mcap) + off;
b_S_mcap = -(b_S_mcap) + off;
a_D_mcap = -(a_D_mcap) + off;
b_D_mcap = -(b_D_mcap) + off;

% trim beginning of data
a_S_mcap = a_S_mcap(40:end);
b_S_mcap = b_S_mcap(40:end);
t_S_mcap = t_S_mcap(40:end);

%% Load Sensor Data

sensor_S = xlsread('Sub0_MoCap_Sqbar.xlsx','A8:C759');
sensor_S = transpose(sensor_S);
sensor_D = xlsread('Sub0_MoCap_Deadlift.xlsx','A8:C797');
sensor_D = transpose(sensor_D);

t_S_wr = sensor_S(1,:);
a_S_wr = sensor_S(2,:);
b_S_wr = sensor_S(3,:);

t_D_wr = sensor_D(1,:);
a_D_wr = sensor_D(2,:);
b_D_wr = sensor_D(3,:);
% convert to S
t_S_wr = t_S_wr/1000;
t_D_wr = t_D_wr/1000;

% interpolate to ~13.1Hz constant
desiredffs = 13.1;
[a_S_13hz, t_S_13hz] = resample(a_S_wr, t_S_wr, desiredffs);
[b_S_13hz, t_S_13hz] = resample(b_S_wr, t_S_wr, desiredffs);

[a_D_13hz, t_D_13hz] = resample(a_D_wr, t_D_wr, desiredffs);
[b_D_13hz, t_D_13hz] = resample(b_D_wr, t_D_wr, desiredffs);

%plot(t_S_wr,a_S_wr, '.-', t_S_13hz, a_S_13hz, 'o-')
%legend('Original Data, approx. 13Hz','Interpolated Data, 13Hz')
%xlabel('Time (s)')
%ylabel('Degrees')
%title('Sensor Squat Data')
hold on
plot(t_D_wr, a_D_wr, '.-', t_D_13hz, a_D_13hz, 'o-')
legend('Original Data, approx. 13Hz','Interpolated Data, 13Hz')
xlabel('Time (s)')
ylabel('Degrees')
title('Deadlift Squat Data')
hold off
% trim resampled data
start_i = 158;
len_i = 489;
a_S_13hz = a_S_13hz(start_i:start_i+len_i);
b_S_13hz = b_S_13hz(start_i:start_i+len_i);
t_S_13hz = t_S_13hz(start_i:start_i+len_i);
t_samples = linspace(1, length(a_S_13hz), length(a_S_13hz));

start_i_d = 166;
len_i_d = 438;
a_D_13hz = a_D_13hz(start_i_d:start_i_d+len_i_d);
b_D_13hz = b_D_13hz(start_i_d:start_i_d+len_i_d);
t_D_13hz = t_D_13hz(start_i_d:start_i_d+len_i_d);
t_samples_D = linspace(1, length(a_D_13hz), length(a_D_13hz));

%plot(t_D_13hz, a_D_13hz, '.-')

%% Mocap re-sample sampling rate (downsample), shift mocap data
% downsample mocap data to match.
desiredffs = 13.1;
[a_S_mcap13, t_S_mcap13] = resample(a_S_mcap, t_S_mcap, desiredffs);
[b_S_mcap13, t_S_mcap13] = resample(b_S_mcap, t_S_mcap, desiredffs);

[a_D_mcap13, t_D_mcap13] = resample(a_D_mcap, t_D_mcap, desiredffs);
[b_D_mcap13, t_D_mcap13] = resample(b_D_mcap, t_D_mcap, desiredffs);

plot(t_S_mcap,a_S_mcap, '.-', t_S_mcap13, a_S_mcap13, 'o-')
legend('Original Data, 200Hz','Interpolated Data, 13Hz')
xlabel('Time (s)')
ylabel('Degrees')
title('Downsampled Squat Motion Capture Data')

plot(t_D_mcap,a_D_mcap, '.-', t_D_mcap13, a_D_mcap13, 'o-')
legend('Original Data, 200Hz','Interpolated Data, 13Hz')
xlabel('Time (s)')
ylabel('Degrees')
title('Downsampled Deadlift Motion Capture Data')

%plot(t_S_13hz, a_S_13hz)

% shift, trim to line up with sensor data
delay = 9.157; % time, tentatively
t_S_mcap13 = t_S_mcap13 + delay;

delay_D = 11.78; % time
t_D_mcap13 = t_D_mcap13 + delay_D;

start_i = 45;
a_S_mcap13 = a_S_mcap13(start_i:start_i+len_i);
b_S_mcap13 = b_S_mcap13(start_i:start_i+len_i);
t_S_mcap13 = t_S_mcap13(start_i:start_i+len_i);
%t_samples_mc = linspace(1, length(a_S_mcap13), length(a_S_mcap13));

start_i_d = 33;
a_D_mcap13 = a_D_mcap13(start_i_d:start_i_d+len_i_d);
b_D_mcap13 = b_D_mcap13(start_i_d:start_i_d+len_i_d);
t_D_mcap13 = t_D_mcap13(start_i_d:start_i_d+len_i_d);


%% Plot Motion Capture Data
%figure('pos',[400,400, 1000,400])
%hold on
%plot(t_D_mcap, a_D)
%plot(t_D_mcap, b_D)
%plot(t_D_mcap, a_D - b_D)
%legend('a_D','b_D','a_D - b_D')
%hold off

%figure('pos',[400,400, 1000,400])
%hold on
%plot(t_S_mcap, a_S_mcap)
%plot(t_S_mcap, b_S_mcap)
%plot(t_S_mcap, a_S_mcap - b_S_mcap)
%legend('a_S','b_S','a_S - b_S')
%hold off

% downsampled
figure('pos',[400,400, 1000,400])
hold on
plot(t_S_mcap13, a_S_mcap13)
plot(t_S_mcap13, b_S_mcap13)
plot(t_S_mcap13, a_S_mcap13 - b_S_mcap13)
legend('a_S','b_S','a_S - b_S')
title('Motion Capture Squat Data')
hold off

% downsampled
figure('pos',[400,400, 1000,400])
hold on
plot(t_D_mcap13, a_D_mcap13)
plot(t_D_mcap13, b_D_mcap13)
plot(t_D_mcap13, a_D_mcap13 - b_D_mcap13)
legend('a_D','b_D','a_D - b_D')
title('Motion Capture Deadlift Data')
hold off
%% Plot Sensor Data

figure('pos',[400,400, 1000,400])
hold on
plot(t_S_13hz, a_S_13hz)
plot(t_S_13hz, b_S_13hz)
plot(t_S_13hz, a_S_13hz - b_S_13hz)
legend('a_S','b_S','a_S - b_S')
title('Sensor Squat Data')
%legend('A_mc','B_mc','A-B_mc','A_wr','B_wr','A-B_wr')
hold off

figure('pos',[400,400, 1000,400])
hold on
plot(t_D_13hz, a_D_13hz)
plot(t_D_13hz, b_D_13hz)
plot(t_D_13hz, a_D_13hz - b_D_13hz)
legend('a_S','b_S','a_S - b_S')
title('Sensor Deadlift Data')
%legend('A_mc','B_mc','A-B_mc','A_wr','B_wr','A-B_wr')
hold off

%% Plot matched MoCap and Sensor Data (no match re-sample) (timescale)
figure('pos',[400,400, 1000,400])
hold on
plot(t_S_13hz, a_S_13hz, '.')
%plot(t_samples, b_S_13hz)
%plot(t_samples, a_S_13hz - b_S_13hz)
%legend('a_S','b_S','a_S - b_S')
%legend('A_mc','B_mc','A-B_mc','A_wr','B_wr','A-B_wr')

plot(t_S_mcap13, a_S_mcap13, '*')
%plot(t_samples_mc, b_S_mcap13)
%plot(t_samples_mc, a_S_mcap13 - b_S_mcap13)
%legend('a_S','b_S','a_S - b_S')
%legend('A_mc','B_mc','A-B_mc','A_wr','B_wr','A-B_wr')
legend('Sensor','Mo Cap');
title('Asynchronous Motion Capture and Sensor Readings')
xlabel('Time (s)')
ylabel('Degree')
hold off

figure('pos',[400,400, 1000,400])
hold on
plot(t_samples_D, a_D_13hz, '.')

plot(t_samples_D, a_D_mcap13, '*')

legend('Sensor','Mo Cap');
title('Asynchronous Motion Capture and Sensor Readings, Deadlift')
xlabel('Samples')
ylabel('Degrees')
hold off


%% Match, re-sample
% trim again, a small amount
a_S_mc13_r = a_S_mcap13(11:end);
b_S_mc13_r = b_S_mcap13(11:end);
t_S_mc13_rs = t_S_mcap13(11:end);

[a_S_mc13_r, t_S_mc13_r] = resample(a_S_mc13_r, t_S_mc13_rs, desiredffs+0.27);
[b_S_mc13_r, t_S_mc13_r] = resample(b_S_mc13_r, t_S_mc13_rs, desiredffs+0.27);

a_S_mc13_r = a_S_mc13_r + 0.281;
b_S_mc13_r = b_S_mc13_r + 2.29; 

% trim again, a small amount
a_D_mc13_r = a_D_mcap13(1:428);
b_D_mc13_r = b_D_mcap13(1:428);
t_D_mc13_rs = t_D_mcap13(1:428);

[a_D_mc13_r, t_D_mc13_r] = resample(a_D_mc13_r, t_D_mc13_rs, desiredffs+0.33);
[b_D_mc13_r, t_D_mc13_r] = resample(b_D_mc13_r, t_D_mc13_rs, desiredffs+0.33);

b_D_mc13_r = b_D_mc13_r + 6.497;
a_D_mc13_r = a_D_mc13_r - 0.397;

figure('pos',[400,400, 1000,400])
hold on
plot(t_samples, a_S_13hz, '.')
plot(t_samples, b_S_13hz, '.')
%plot(t_samples, a_S_13hz - b_S_13hz, '.')
%legend('a_S','b_S','a_S - b_S')
%legend('A_mc','B_mc','A-B_mc','A_wr','B_wr','A-B_wr')

plot(t_samples, a_S_mc13_r, 'x')
plot(t_samples, b_S_mc13_r, 'xc')
%plot(t_samples_mc, a_S_mc13_r - b_S_mc13_r, '*')
%legend('a_S','b_S','a_S - b_S')
legend('Upper Back A_s','Lumbar B_s','Upper Back_{mc}','Lumbar_{mc}')
%legend('sensor','mocap');
xlabel('Time (samples)')
ylabel('Angular Degree')
title('Sensor Data vs Motion Capture Pitch Angle Data (Squat)')
hold off

figure('pos',[400,400, 1000,400])
hold on
plot(t_samples_D, a_D_13hz, '.')
plot(t_samples_D, b_D_13hz, '.')
plot(t_samples_D, a_D_mc13_r, 'x')
plot(t_samples_D, b_D_mc13_r, 'xc')
legend('Upper Back_s','Lumbar_s','Upper Back_{mc}','Lumbar_{mc}')
%legend('sensor','mocap');
xlabel('Time (samples)')
ylabel('Angular Degree')
title('Sensor Data vs Motion Capture Pitch Angle Data (Deadlift)')
hold off

%% Tool to Optimize motion capture offset 
rms_min = 100;
goal = 0;
for j = linspace(-10,10,40000)
    temp = a_S_mc13_r + j;
    
    sum = 0;
    for i = t_samples(125:end-4)
        dif2 = (a_S_13hz(i) - temp(i))^2;
        sum = sum + dif2;
    end
    rms_check = sqrt(sum/length(t_samples));
    
    if rms_check < rms_min
        rms_min = rms_check;
        goal = j;
    end
end
%% Calculate RMS error
%squat
sum = 0;
for i = t_samples(125:end-4)
    dif2 = (a_S_13hz(i) - a_S_mc13_r(i))^2;
    sum = sum + dif2;
end
rms_a_S = sqrt(sum/length(t_samples))

sum = 0;
for i = t_samples(125:end-4)
    dif2 = (b_S_13hz(i) - b_S_mc13_r(i))^2;
    sum = sum + dif2;
end
rms_b_S = sqrt(sum/length(t_samples))

%deadlift
sum = 0;
for i = t_samples_D(100:end-7)
    dif2 = (a_D_13hz(i) - a_D_mc13_r(i))^2;
    sum = sum + dif2;
end
rms_a_D = sqrt(sum/length(t_samples_D))

sum = 0;
for i = t_samples_D(100:end-7)
    dif2 = (b_D_13hz(i) - b_D_mc13_r(i))^2;
    sum = sum + dif2;
end
rms_b_D = sqrt(sum/length(t_samples_D))

mean_rms = mean([rms_a_D, rms_b_D, rms_a_S, rms_b_S])
