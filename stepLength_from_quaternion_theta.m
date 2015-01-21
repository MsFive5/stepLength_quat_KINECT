% use the quaternion to calculate the step length

function stepLength_from_quaternion()
%% loading data 
close all;
clear all;

% leg_length = 0.7092;
target_walking_distance = 50*30.48*0.01;
dir_name='1201maodata2';

load(strcat(dir_name, '/test_data.mat'));
load(strcat(dir_name, '/leg_length.mat'));
plotYes = 1;

[hs, start_ind, end_ind] = heel_strike_detection(test_data, plotYes);

total_walking_distance = calculate_total_distance(hs, start_ind, ...
    end_ind, test_data, leg_length);

display((total_walking_distance-target_walking_distance)/target_walking_distance);


end
%% heel strike and walk start detection
function [hs, start_ind, end_ind] = heel_strike_detection(data, plotYes)
% gravity in sensor frame
% left side
clear hs;

gravity_s = data.left.gravity_s;
temp = sum(gravity_s .* repmat([0 1 0], size(gravity_s,1),1),2);
thetal = acos(temp/norm(gravity_s(1,:)));
acc = data.left.acc;
acc_le=acc.^2*[1;1;1];
[start_ind.left, end_ind.left] = walk_start_detection(acc,1);
[k,v]=v_findpeaks(thetal(start_ind.left:end_ind.left),'v',130);
k = k(v<2.8);
toe_off.left = k + start_ind.left - 1;
hs.l = zeros(length(toe_off.left)-1,1);


% right side
gravity_s = data.right.gravity_s;
temp = sum(gravity_s .* repmat([0 1 0], size(gravity_s,1),1),2);
temp1 =temp/norm(gravity_s(1,:));
temp1 = min(temp1, 1); temp1 = max(temp1,-1);
thetar = acos(temp1);
acc = data.right.acc;
acc_re=acc.^2*[1;1;1];
[start_ind.right, end_ind.right] = walk_start_detection(acc,1);
[k,v]=v_findpeaks(medfilt1(thetar(start_ind.right:end_ind.right),10),'v',130);
k=k(v<2.8);
toe_off.right = k + start_ind.right - 1;
hs.r = zeros(length(toe_off.right)-1,1);

if plotYes
figure; subplot(2,1,1); plot(acc_le); hold on; 
for i = 1 : length(toe_off.left)-1
    [kkl,~]=v_findpeaks(acc_le(toe_off.left(i):toe_off.left(i+1)),'q',100);
    hs.l(i)=kkl(1)+toe_off.left(i)-1;
    plot([hs.l(i) hs.l(i)],get(gca,'YLim'),'g');    
end
l_lb = toe_off.left(i+1);
l_ub = min(toe_off.left(i+1)+round(mean(diff(toe_off.left))*2),length(acc_le));
[kkl,~] = v_findpeaks(acc_le(l_lb:l_ub),'q',100);
hs.l = [hs.l;kkl(1)+toe_off.left(i+1)-1];
plot([hs.l(i+1) hs.l(i+1)],get(gca,'YLim'),'g'); 
hold off;
clear i;

subplot(2,1,2); plot(acc_re); hold on;
for i = 1 : length(toe_off.right)-1
    [kkr,~]=v_findpeaks(acc_re(toe_off.right(i):toe_off.right(i+1)),'q',100);
    hs.r(i)=kkr(1)+toe_off.right(i)-1;
    plot([hs.r(i) hs.r(i)],get(gca,'YLim'),'g');
end
r_lb = toe_off.right(i+1);
r_ub = min(toe_off.right(i+1)+round(mean(diff(toe_off.right))*2),length(acc_re));
[kkr,~] = v_findpeaks(acc_re(r_lb:r_ub),'q',100);
hs.r = [hs.r;kkr(1)+toe_off.right(i+1)-1];
plot([hs.r(i+1) hs.r(i+1)],get(gca,'YLim'),'g'); 

hold off;

hs.l = round(hs.l);
hs.r = round(hs.r);

figure; subplot(2,1,1); plot(thetal); hold on; 
for i = 1 : length(toe_off.left)
    plot([toe_off.left(i) toe_off.left(i)],get(gca,'YLim'),'g');    
end

subplot(2,1,2); plot(medfilt1(thetar,10)); hold on;
for i = 1 : length(toe_off.right)
    plot([toe_off.right(i) toe_off.right(i)],get(gca,'YLim'),'g');   
end

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

%% calculate step length
function total_distance = calculate_total_distance(hs, start_ind, ...
    end_ind, test_data, leg_length)
step_length_left = zeros(size(hs.l));
for i = 1 : length(hs.l)
    step_length_left(i) = calculate_one_step(hs.l(i), hs.r, test_data.left.quat, leg_length);
end
clear i;
step_length_right = zeros(size(hs.r));
for i = 1 : length(hs.r)
    step_length_right(i) = calculate_one_step(hs.r(i), hs.l, test_data.right.quat, leg_length);
end
total_distance = sum(step_length_right) + sum(step_length_left);
end

function step_length = calculate_one_step(hs_ind, oppo_side_hs, quat, leg_length)
for j = 1 : length(oppo_side_hs)
    if oppo_side_hs(j) > hs_ind
        break;
    end
end

indA = hs_ind;
indB = oppo_side_hs(j);
if indA < indB 
wlen=5;
% q1 = median(quat(indA-wlen:indA+wlen, :));
% q2 = median(quat(indB-wlen:indB+wlen, :));
q1 = quat(indA,:);
q2 = quat(indB,:);

quat_diff = quatmultiply(q1,quatconj(q2));
step_length = leg_length * sin(acos(quat_diff(1))) * 2;
else
    step_length = 0;
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


