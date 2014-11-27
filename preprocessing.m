function preprocessing(  )
% pre-processing the raw data
close all;
clear all;

dir_name='1125data';
left_sensor_name='E8EE';
right_sensor_name='E915';

wlen = 20;
threshold = 0.001;

%% read the raw BlueRadios_*.txt file and plot left and right side
% read raw data
data = read_data(dir_name, left_sensor_name, right_sensor_name);
% synchronize
align_index = match_timestamp(data);
[data_aligned flag] = interpolate_data(data, align_index);
keyboard;

timestamp = data_aligned.timestamp;

if strcmp(dir_name,'1125data')
    train_ind = [9300 11000];
    test_ind = [16000 20000];
else
%     test_ind = [6000 min(size(testL,1),size(testR,1))];
end
test_data = calibrate_aligned(data_aligned, wlen, threshold, test_ind);
train_data = calibrate_aligned(data_aligned, wlen, threshold, train_ind);
save(strcat(dir_name,'/train_data'), 'train_data');
save(strcat(dir_name,'/test_data'), 'test_data');

kinect_data = read_kinect_data(dir_name);
save(strcat(dir_name,'/kinect_data'), 'kinect_data');
end

function kinect_data = read_kinect_data(dir_name)
kinect_listing = dir(strcat(dir_name,'/KINECT2_data_*'));
data = csvread(strcat(dir_name, '/',kinect_listing(1).name),1,0);
kinect_data.position = data(:,2:4);
kinect_data.timestamp= data(:,1);
end
%% read raw data
function data = read_data(dir_name, left_sensor_name, right_sensor_name)
as = 2048;  %sensitivity of accelerometer 16g:2048 8g:2048*2
gs = 16.4;       %sensitivity of gyroscope
qs = 1073741824; %sensitivity of quaternion

left_listing = dir(strcat(dir_name,'/*',left_sensor_name,'*'));
left_data = csvread(strcat(dir_name, '/',left_listing(1).name),1,0);

left_data(:,2) = -left_data(:,2);
left_data(:,3) = -left_data(:,3);
left_data(:,2:4) = left_data(:,2:4)/as;
left_data(:,5:7) = left_data(:,5:7)/gs*pi/180;
left_data(:,8:11) = left_data(:,8:11)/qs;

right_listing=dir(strcat(dir_name,'/*', right_sensor_name,'*'));
right_data = csvread(strcat(dir_name, '/',right_listing(1).name),1,0);
right_data(:,2) = -right_data(:,2);
right_data(:,3) = -right_data(:,3);
right_data(:,2:4) = right_data(:,2:4)/as;
right_data(:,5:7) = right_data(:,5:7)/gs*pi/180;
right_data(:,8:11) = right_data(:,8:11)/qs;


data.left = left_data;
data.right = right_data;
data.timestamp = left_data(:,1);
end

%% match timestamp
function index = match_timestamp(data)
time1 = data.left(:,1);
time2 = data.right(:,1);
if time1(1) - time2(1) < 0
    index = find_starting_index(time1, time2);
elseif time1(1) - time2(1) > 0
    temp = find_starting_index(time2, time1);
    index = [temp(2) temp(1)];
end

sync_step = 200*1; % sync for every 10 seconds
for i = (index(1,1)+sync_step):sync_step:length(time1)
    j = i + index(end,2) - index(end,1);
    if j > length(time2)
        break;
    end
    while (j < length(time2)) && (abs(time2(j+1) - time1(i)) <= abs(time2(j) - time1(i)))
        j = j + 1;
    end
    if j == length(time2)
        break;
    end
    while (abs(time2(j-1) - time1(i)) <= abs(time2(j) - time1(i)))
        j = j - 1;
    end
    index = [index; [i j]];

end
end
function index = find_starting_index(time1, time2)
i = 1;
while time1(i) < time2(1)
     i = i + 1;
end
while (i > 1) && (abs(time1(i-1)-time2(1)) <= abs(time1(i)-time2(1)))
    i = i - 1;
end

j = 1;
while abs(time2(j+1) - time1(i)) <= abs(time2(j) - time1(i))
    j = j + 1;
end
while (j > 1) && (time2(j-1) == time2(j))
    j = j - 1;
end
index = [i j];
end
%% interpolate data
function [data_out flag] = interpolate_data(data, index)

dataL = data.left(:,2:11);
dataR = data.right(:,2:11);
flagL = differentiate_index(data.left(:,end));
flagR = differentiate_index(data.right(:,end));
dataL_aligned = dataL;
dataR_aligned = dataR(1:index(end,2),:);

flag = zeros(index(end,1), 1);
for i = 1:size(index,1)-1
    % angular_rate(index(i,1):index(i+1,1), 1) = gyro1(index(i,1):index(i+1,1));
    % right side length, resample dataRight with lengthR_origin up to
    % dataLeft, lengthL_target
    lengthR_origin = index(i+1,2)-index(i,2)+1;
    lengthL_target = index(i+1,1)-index(i,1)+1;
    before_align = dataR(index(i,2):index(i+1,2),:);
    after_align = resample(before_align, lengthL_target, lengthR_origin); 
    if i ~= 1
         after_align(1,:) = (dataR_aligned(index(i,1), :) + after_align(1,:))/2;    
    end
    dataR_aligned(index(i,1):index(i+1,1), :) = after_align;
    
    if (sum(flagL(index(i,1):index(i+1,1)))+sum(flagR(index(i,2):index(i+1,2)))) > 0
        flag(index(i,1):index(i+1,1)) = 1;
    end
end
figure; subplot(2,1,1); plot(dataL_aligned(:,1:3));xlim([0 length(data.timestamp)]);title('left side');
subplot(2,1,2); plot(dataR_aligned(:,1:3));xlim([0 length(data.timestamp)]);title('right side');
data_out.left=dataL_aligned;
data_out.right = dataR_aligned;
data_out.timestamp = data.timestamp;
end

function flag = differentiate_index(index)
d_index = diff(index);
ind = find(d_index < -10000);
for i = 1:length(ind)
    d_index(ind(i)) = (index(ind(i)+1)-(-32768))+(32767-index(ind(i)))+1;
end
flag = [0; (d_index ~= 1)];
end
function plot_zv_all(train, test)
figure;
subplot(2,1,1);plot(train.left.acc);hold on;
for i = 1:2
    plot([train.left.zv_init(i,1) train.left.zv_init(i,1)], get(gca, 'YLim'), 'r--');
    plot([train.left.zv_init(i,2) train.left.zv_init(i,2)], get(gca, 'YLim'), 'g--');
end
hold off; title('train, left');
subplot(2,1,2);plot(train.right.acc);hold on;
for i = 1:2
    plot([train.right.zv_init(i,1) train.right.zv_init(i,1)], get(gca, 'YLim'), 'r--');
    plot([train.right.zv_init(i,2) train.right.zv_init(i,2)], get(gca, 'YLim'), 'g--');
end
hold off; title('train, right');
figure;
subplot(2,1,1);plot(test.left.acc);hold on;
for i = 1:2
    plot([test.left.zv_init(i,1) test.left.zv_init(i,1)], get(gca, 'YLim'), 'r--');
    plot([test.left.zv_init(i,2) test.left.zv_init(i,2)], get(gca, 'YLim'), 'g--');
end
hold off; title('test, left');
subplot(2,1,2);plot(test.right.acc);hold on;
for i = 1:2
    plot([test.right.zv_init(i,1) test.right.zv_init(i,1)], get(gca, 'YLim'), 'r--');
    plot([test.right.zv_init(i,2) test.right.zv_init(i,2)], get(gca, 'YLim'), 'g--');
end
hold off; title('test, right');
end
function raw = read_raw(dir_name, left_sensor_name, right_sensor_name)

left_listing = dir(strcat(dir_name,'/BlueRadios_',left_sensor_name,'*.txt'));
left_data = csvread(strcat(dir_name, '/',left_listing(1).name),1,0);

right_listing=dir(strcat(dir_name,'/BlueRadios_', right_sensor_name,'*.txt'));
right_data = csvread(strcat(dir_name, '/',right_listing(1).name),1,0);

as = 2048; %sensitivity of accelerometer 16g:2048 8g:2048*2
gs = 16.4;       %sensitivity of gyroscope
qs = 1073741824; %sensitivity of quaternion

left_data(:,2) = -left_data(:,2);
left_data(:,3) = -left_data(:,3);
left_data(:,2:4) = left_data(:,2:4)/as;
left_data(:,5:7) = left_data(:,5:7)/gs;
left_data(:,8:11) = left_data(:,8:11)/qs;

raw.left.acc = left_data(:,2:4);
raw.left.gyr = left_data(:,5:7)*pi/180;
raw.left.quat = left_data(:,8:11);
left_acc_energy = raw.left.acc(:,1).^2 + raw.left.acc(:,2).^2 + raw.left.acc(:,3).^2;

right_data(:,2) = -right_data(:,2);
right_data(:,3) = -right_data(:,3);
right_data(:,2:4) = right_data(:,2:4)/as;
right_data(:,5:7) = right_data(:,5:7)/gs;
right_data(:,8:11) = right_data(:,8:11)/qs;

raw.right.acc = right_data(:,2:4);
raw.right.gyr = right_data(:,5:7)*pi/180;
raw.right.quat = right_data(:,8:11);
right_acc_energy = raw.right.acc(:,1).^2 + raw.right.acc(:,2).^2 + raw.right.acc(:,3).^2;


figure; subplot(2,1,1);plot(left_acc_energy); title('left','FontSize',15);
subplot(2,1,2); plot(right_acc_energy);title('right','FontSize',15);
end


function out = calibrate_aligned(in, wlen, threshold, ind)
% calibrate gyr and quat and acc
% strip gravity from acc reading

    % determine initial window
    zv_window_left = determine_initial_window(in.left(:,1:3), wlen, threshold);
    zv_window_right = determine_initial_window(in.right(:,1:3), wlen, threshold);

    initial_window_size = min(zv_window_left(1,2), zv_window_right(1,2))-100;
    figure; subplot(2,1,1); hold on; plot(in.left(:,1:3));
    for i = 1:2
    plot([1 1], get(gca, 'YLim'), 'r--');
    plot([initial_window_size initial_window_size], get(gca, 'YLim'), 'g--');
    end
    xlim([0 size(in.left,1)]);
    title('left');
    subplot(2,1,2); hold on; plot(in.right(:,1:3));
    for i = 1:2
    plot([1 1], get(gca, 'YLim'), 'r--');
    plot([initial_window_size initial_window_size], get(gca, 'YLim'), 'g--');
    end
    xlim([0 size(in.left,1)]);
    title('right');

    acc = in.left(:,1:3);
    gyr = in.left(:,4:6);
    quat = in.left(:,7:10);
    % calibrate gyr data
    gyr_mean = mean(gyr(1:initial_window_size,:),1);
    gyr = (gyr - repmat(gyr_mean, size(gyr, 1), 1));
    
    % calibrate quaternion data
    quat_mean = mean(quat(1:initial_window_size,:),1);
    quat = quatdivide(quat, quat_mean); 
    
    % calibrate acc data and subtract the gravity component
    % this can be done only after the quaternion calibration
    acc_mean = mean(acc(1:initial_window_size,:),1);
    acc_g = gravity_subtraction(acc, acc_mean, quat); % acc in global frame
    
    out.left.acc = acc(ind(1):ind(2), :);
    out.left.gyr = gyr(ind(1):ind(2), :);
    out.left.quat = quat(ind(1):ind(2), :);
    out.left.acc_g = acc_g(ind(1):ind(2), :);
    clear acc gyr quat acc_g;
    
    acc = in.right(:,1:3);
    gyr = in.right(:,4:6);
    quat = in.right(:,7:10);
    
    gyr_mean = mean(gyr(1:initial_window_size,:),1);
    gyr = (gyr - repmat(gyr_mean, size(gyr, 1), 1));
    
    quat_mean = mean(quat(1:initial_window_size,:),1);
    quat = quatdivide(quat, quat_mean); 
    
    acc_mean = mean(acc(1:initial_window_size,:),1);
    acc_g = gravity_subtraction(acc, acc_mean, quat); % acc in global frame
  
    out.right.acc = acc(ind(1):ind(2), :);
    out.right.gyr = gyr(ind(1):ind(2), :);
    out.right.quat = quat(ind(1):ind(2), :);
    out.right.acc_g = acc_g(ind(1):ind(2), :);
    
    out.timestamp = in.timestamp(ind(1):ind(2));

end

function global_acc = gravity_subtraction(acc,acc_mean, quat)
% map acc to global frame and subtract the gravity
% acc in the zero velocity window is the gravity
global_acc = zeros(size(acc));
for i = 1:length(acc)
    a_temp = [0 acc(i,:)];
    a_temp = quatmultiply(quatmultiply(quat(i,:),a_temp),quatconj(quat(i,:)));
    global_acc(i,:) = a_temp(2:4) - acc_mean;
end

end

function initial = determine_initial_window(acc, wlen, threshold)
% 1st row: beginning zero velocity window
% 2nd row: endding zero velocity window

wlen = 2*wlen;
initial = zeros(2,2);
for i = 1:length(acc(:,1))
    if var(acc(i:i+wlen,1).^2 + acc(i:i+wlen,2).^2 + acc(i:i+wlen,3).^2) > threshold 
        break;
   end
end
if i > 1
    i = i - 1;
end
initial(1,:) = [1 i];
acc = flipud(acc);

for i = 1:length(acc(:,1))
    if var(acc(i:i+wlen,1).^2 + acc(i:i+wlen,2).^2 + acc(i:i+wlen,3).^2) > threshold
        break;
    end
end
if i > 1
    i = i - 1;
end
initial(2,:) = [(length(acc(:,1)) - i + 1) length(acc(:,1))];
acc = flipud(acc);


end
