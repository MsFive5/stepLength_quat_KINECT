% use the quaternion to calculate the step length
% if I update sth here and there
% what will happen.

function stepLength_from_quaternion()
%% loading data 
close all;
clear all;

leg_length = 0.7372;
total_walking_distance = 50*30.48*0.01;
dir_name='1125data';
load(strcat(dir_name, '/test_data.mat'));
plotYes = 1;


[hsl, ~] = heel_strike_detection(test_data.left.acc, plotYes);
[hsr, ~] = heel_strike_detection(test_data.right.acc, plotYes);

[start_left, end_left] = walk_start_detection(test_data.left.acc, plotYes);
[start_right, end_right] = walk_start_detection(test_data.right.acc, plotYes);

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

SL_l = calculate_SLs(test_data.left.quat,left_pendulum_index,leg_length);
SL_r = calculate_SLs(test_data.right.quat,right_pendulum_index, leg_length);
SL_total = sum(SL_l)+sum(SL_r);
display((SL_total-total_walking_distance)/total_walking_distance);
end

function [k_out, v_out] = heel_strike_detection(acc, plotYes)
acc_e = sqrt(acc.^2*[1;1;1]);
[k, v] = v_findpeaks(acc_e,'q',100);
v_out = v(v>1.2);
k_out = k(v>1.2);
k_out =round(k_out);
if plotYes
figure; hold on; plot(acc_e); plot(k_out,v_out,'r.'); 
title('heel strike detection')
end
end

function [start_ind, end_ind] = walk_start_detection(acc, plotYes)
wlen=20;
acc_e = sqrt(acc.^2*[1;1;1]);
for ii = 1 : length(acc_e)-wlen
    if var(acc_e(ii:ii+wlen))>0.01
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
function SLs = calculate_SLs(quat, pend_ind, leg_length)
% calculate the step lengths out of the quaternion change
% quat: quaternion  pend_ind: pendulum index
SLs = zeros(size(pend_ind, 1),1);
wlen=10;
for ii = 1 : size(pend_ind,1)
    q1 = median(quat(pend_ind(ii,1)-wlen:pend_ind(ii,1)+wlen, :));
    q2 = median(quat(pend_ind(ii,2)-wlen:pend_ind(ii,2)+wlen, :));
    quat_diff = quatmultiply(q1,quatconj(q2));
    SLs(ii) = leg_length * sin(acos(quat_diff(1))) * 2;
end
end


