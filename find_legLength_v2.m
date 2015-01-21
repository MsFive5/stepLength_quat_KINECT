function find_legLength()
close all;


dir_name = '1201maodata1';
plotYes = 1;

% loading data
load(strcat(dir_name, '/test_data.mat'));
load(strcat(dir_name, '/kinect_data.mat'));


wlen=25; threshold.left = 5; threshold.right = 3;
[ms_point] = stance_detection(test_data, wlen, threshold, plotYes);
keyboard;

% heel strike detection and walk start detection

[hs] = heel_strike_detection(test_data, ms_point, plotYes);
keyboard;
[start_left, end_left] = walk_start_detection(train_data.left.acc, plotYes);
[start_right, end_right] = walk_start_detection(train_data.right.acc, plotYes);

[pend_indL, pend_indR] = ...
    define_pendulum_index(hsl, hsr, start_left, end_left, start_right, end_right);

% first step on left
for i = 1 : 3
leg_lengthR=find_leg_length(kinect_data, train_data, pend_indR(i,:),'r')
leg_lengthL=find_leg_length(kinect_data, train_data, pend_indL(i,:),'l')
end
end

function leg_length=find_leg_length(kinect_data, train_data, pendulum_index,side)
wlen=10;
pos_1 = get_kinect_position(kinect_data, train_data.timestamp(pendulum_index(1)));
pos_2 = get_kinect_position(kinect_data, train_data.timestamp(pendulum_index(2)));
stride_length = norm(pos_1-pos_2);
if strcmp(side,'r')
quat = train_data.right.quat;
else
quat = train_data.left.quat;    
end
% figure; subplot(2,1,1);plot(sqrt(quat.^2*[1;1;1;1]));subplot(2,1,2); plot(quat);
% q1 = median(quat(pendulum_index(1)-wlen:pendulum_index(1)+wlen, :));
% q2 = median(quat(pendulum_index(2)-wlen:pendulum_index(2)+wlen, :));
q1 = quat(pendulum_index(1),:);
q2 = quat(pendulum_index(2),:);

quat_diff = quatmultiply(q1,quatconj(q2));
leg_length = stride_length / sin(acos(quat_diff(1))) / 2;
end

function position = get_kinect_position(kinect_data, target_timestamp)
% given timestamp from sensor signal, return 3D position information from
% kinect data
[c,i]=min(abs(kinect_data.timestamp - target_timestamp));
%display(c);
position = kinect_data.position(i,:);
end

function [left_pendulum_index, right_pendulum_index] = ...
    define_pendulum_index(hsl, hsr, start_left, end_left, start_right, end_right)
if hsl(1) < hsr(1) && hsl(end) < hsr(end)
    % case 1
    start_ind = start_right; end_ind = end_left;
    right_pendulum_index = [start_ind hsl(1);...
        hsr(1:end-1) hsl(2:end)];
    left_pendulum_index = [hsl(1:end-1) hsr(1:end-1); hsl(end) end_ind];
elseif hsl(1) < hsr(1) && hsl(end) > hsr(end)
    % case 2
    start_ind = start_right; end_ind = end_right;
    right_pendulum_index = [start_ind hsl(1);  ...
        hsr(1:end-1) hsl(2:end-1);hsr(end) end_ind];
    left_pendulum_index = [hsl(1:end-1) hsr];
elseif hsl(1) > hsr(1) && hsl(end) < hsr(end)
    % case 3
    start_ind = start_left; end_ind = end_left;
    right_pendulum_index = [hsr(1:end-1) hsl];
    left_pendulum_index = [start_ind hsr(1);...
        hsl(1:end-1) hsr(2:end-1); hsl(end) end_ind];
else
    start_ind = start_left; end_ind = end_right;
    right_pendulum_index = [hsr(1:end-1) hsl(1:end-1);...
        hsr(end) end_ind];    
    left_pendulum_index = [start_ind hsr(1); hsl(1:end-1) hsr(2:end)];
end
end

function [hs] = heel_strike_detection(data, ms_point, plotYes)
acc_e = data.left.acc.^2*[1;1;1];
hs.l = [];
for i=1:length(ms_point.left)-1;
    current_stride = acc_e(ms_point.left(i):ms_point.left(i+1));
    [~,ind]=max(current_stride);
    hs.l=[hs.l;ind-1+ms_point.left(i)];
end
acc_e = data.right.acc.^2*[1;1;1];
hs.r = [];
for i=1:length(ms_point.right)-1;
    current_stride = acc_e(ms_point.right(i):ms_point.right(i+1));
    [~,ind]=max(current_stride);
    hs.r=[hs.r;ind-1+ms_point.right(i)];
end

if plotYes
figure; subplot(2,1,1); hold on; plot(data.left.acc.^2*[1;1;1]);title('left');
for i = 1:size(hs.l,1)
    plot([hs.l(i) hs.l(i)], get(gca, 'YLim'), 'g--');
end
hold off;
subplot(2,1,2); hold on; plot(data.right.acc.^2*[1;1;1]);title('right');
for i = 1:size(hs.r,1)
    plot([hs.r(i) hs.r(i)], get(gca, 'YLim'), 'r--');
end
hold off;
end
end

function [ms_point] = stance_detection(data, wlen, threshold, plotYes)
gyr_energy_left = data.left.gyr .^ 2 * [1;1;1];
gyr_energy_right = data.right.gyr .^ 2 * [1;1;1];

ms_window.left = find_ms(gyr_energy_left,wlen,threshold.left);
ms_window.right = find_ms(gyr_energy_right, wlen, threshold.right);
ms_point.left = round(mean(ms_window.left,2));
ms_point.right = round(mean(ms_window.right,2));
if plotYes
figure; subplot(2,1,1); hold on; plot(data.left.acc.^2*[1;1;1]);title('left');
for i = 1:size(ms_window.left,1)
    plot([ms_point.left(i) ms_point.left(i)], get(gca, 'YLim'), 'r--');
end
hold off;
subplot(2,1,2); hold on; plot(data.right.acc.^2*[1;1;1]);title('right');
for i = 1:size(ms_window.right,1)
    plot([ms_point.right(i) ms_point.right(i)], get(gca, 'YLim'), 'r--');
end
hold off;
end
end

function [zv_window zv_threshold] = find_ms(gyr_energy, wlen, threshold)
% find the mid-stance phase using gyr energy

zv_point = collect_zv(gyr_energy, wlen, threshold); 

[zv_window zv_threshold] = merge_zv(zv_point, gyr_energy);

end

function [new_window] = find_ms_stable(old_window, gyr_energy, wlen)
% refine the old zv window such that the gyr_energy is the most stable
new_window = nan(size(old_window));
num_stride = size(old_window, 1);

for i = 1 : num_stride
    % refine a new window within each stride
    start_point = old_window(i,1);
    end_point = old_window(i,2)-wlen;
    % initialize variance of energy
    energy_var = var(gyr_energy(start_point:start_point+wlen)); 
    new_window(i,1)=start_point+1;
    new_window(i,2)=start_point+1+wlen;
    for j = start_point+1 : end_point
        if var(gyr_energy(j:j+wlen)) < energy_var
            energy_var = var(gyr_energy(j:j+wlen));
            new_window(i,1) = j;
            new_window(i,2) = j + wlen;
        end
    end
end

figure; hold on; plot(gyr_energy);
for i = 1:num_stride
    plot([new_window(i,1) new_window(i,1)], get(gca, 'YLim'), 'r--');
    plot([new_window(i,2) new_window(i,2)], get(gca, 'YLim'), 'g--');

end
end

function zv_point = collect_zv(gyr_energy, wlen, threshold)
zv_point = [];
gyr_energy_mean = zeros(size(gyr_energy));
for i = 1 : 2*wlen+1 : length(gyr_energy)
    window_start = i - wlen;
    window_end = i + wlen;
    if window_start < 1
        window_start = 1;
    end
    if window_end > length(gyr_energy)
        window_end = length(gyr_energy);
    end
    gyr_energy_mean(i) = mean(gyr_energy(window_start:window_end));
    if (gyr_energy_mean(i) < threshold)
        zv_point = [zv_point window_start:window_end];
    end
end
zv_point = unique(zv_point);
end

function [zv_window zv_threshold] = merge_zv(zv_point, gyr_energy)
zv_window = [];
i = 1;
window_start = zv_point(i);
window_point = window_start;
while i < length(zv_point)
     i = i + 1;
     if (zv_point(i) - window_point > 1)
         window_end = zv_point(i-1);
         zv_window = [zv_window; [window_start window_end]];
         window_start = zv_point(i);
         window_point = window_start;
     else
         window_point = zv_point(i);
     end 
end
window_end = zv_point(i);
zv_window = [zv_window; [window_start window_end]];
zv_threshold = zeros(length(zv_window(:,1)),1);
for i = 1:length(zv_window(:,1))
    zv_threshold(i) = mean(gyr_energy(zv_window(i,1):zv_window(i,2)));
end
end

function [start_ind, end_ind] = walk_start_detection(acc, plotYes)
wlen=20;
acc_e = sqrt(acc.^2*[1;1;1]);
for ii = 1 : length(acc_e)-wlen
    if var(acc_e(ii:ii+wlen))>0.005
        break;
    end
end
start_ind = ii;
clear ii;
acc_e = flipud(acc_e);
for ii = 1 : length(acc_e)-wlen
    if var(acc_e(ii:ii+wlen))>0.01
        break;
    end
end
end_ind = length(acc_e)-ii;
acc_e = flipud(acc_e);
if plotYes
    figure; plot(acc_e); hold on; 
    plot([start_ind start_ind],get(gca,'YLim'),'Color',[1 0 0])
    plot([end_ind end_ind],get(gca,'YLim'),'Color',[0 1 0])
end

end