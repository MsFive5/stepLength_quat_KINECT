function find_legLength()
close all;


dir_name = '1201maodata1';
plotYes = 1;
rmv.l=[];
add.l=[];

rmv.r=[];
add.r=[];

if strcmp(dir_name, '1215data_leftstraight1')
    rmv.l=[1];
elseif strcmp(dir_name, '1215data_leftstraight2')
    rmv.l=[1 3];
    add.l=[1033];
elseif strcmp(dir_name, '1216xiaodata1')
    rmv.r=[3];
    add.l=[1081 1818];
    add.r = [1684];
elseif strcmp(dir_name, '1216xiaodata2')
    rmv.r = [3];
    add.r = [1440];
elseif strcmp(dir_name, '1201maodata1')
    rmv.l=[4];
    rmv.r=[1,5];
end
% loading data
load(strcat(dir_name, '/train_data.mat'));
load(strcat(dir_name, '/kinect_data.mat'));

% heel strike detection and walk start detection
[hsl,hsr] = heel_strike_detection(train_data, plotYes);
keyboard;
[hsl,hsr] = adjust_manually(hsl,hsr,rmv,add);
figure;subplot(2,1,1); plot(train_data.left.acc);hold on;
for i = 1 : length(hsl)
plot([hsl(i) hsl(i)],get(gca,'YLim'),'Color',[0 1 0])
end
hold off;
subplot(2,1,2); plot(train_data.left.quat); hold on;

[start_left, end_left] = walk_start_detection(train_data.left.acc, plotYes);
[start_right, end_right] = walk_start_detection(train_data.right.acc, plotYes);

[pend_indL, pend_indR] = ...
    define_pendulum_index(hsl, hsr, start_left, end_left, start_right, end_right);

% first step on left
leg_lengthR = zeros(1,3);
leg_lengthL = zeros(1,3);
for i = 1 : 3
leg_lengthR(i)=find_leg_length(kinect_data, train_data, pend_indR(i,:),'r')
leg_lengthL(i)=find_leg_length(kinect_data, train_data, pend_indL(i,:),'l')
end
leg_length = median([leg_lengthR leg_lengthL])
save(strcat(dir_name,'/leg_length'), 'leg_length');
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

function [hsl, hsr] = heel_strike_detection(data, plotYes)
acc_le = data.left.acc.^2*[1;1;1];
[k, v] = v_findpeaks(acc_le,'q',150);
vvl = v(v>1.4);
kkl = k(v>1.4);
kkl =round(kkl);
hsl=kkl;

acc_re = data.right.acc.^2*[1;1;1];
[k, v] = v_findpeaks(acc_re,'q',120);
vvr = v(v>1.4);
kkr = k(v>1.4);
kkr =round(kkr);
hsr=kkr;

if plotYes
figure;subplot(2,1,1); hold on; plot(acc_le); plot(kkl,vvl,'r.');
title('heel strike detection left'); hold off;
subplot(2,1,2); hold on; plot(acc_re); plot(kkr,vvr,'r.');
title('heel strike detection right'); hold off;
end
end

function [hsl,hsr] = adjust_manually(hsl,hsr,rmv,add)
hsl(rmv.l)=[];
hsr(rmv.r)=[];
for j=1:length(add.l);
    for i = 1 : length(hsl)
        if add.l(j)<hsl(i)
            break;
        end
    end
    hsl = [hsl(1:i-1);add.l(j);hsl(i:end)];
end
for j=1:length(add.r);
    for i = 1 : length(hsr)
        if add.r(j)<hsr(i)
            break;
        end
    end
    hsr = [hsr(1:i-1);add.r(j);hsr(i:end)];
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