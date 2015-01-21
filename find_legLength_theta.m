function find_legLength()
close all;



dir_name = '1216xiaodata2';
plotYes = 1;

% loading data
load(strcat(dir_name, '/train_data.mat'));
load(strcat(dir_name, '/kinect_data.mat'));

% heel strike detection and walk start detection
[hs, start_ind, end_ind] = heel_strike_detection(train_data, plotYes);


leg_length = calculate_leg_length(hs, train_data, kinect_data)

leg_length = median(leg_length);
% [pend_ind] = def_pend_ind_train(hs,start_ind, end_ind);
% 
% % first step on left
% leg_lengthR = zeros(1,2);
% leg_lengthL = zeros(1,2);
% for i = 1 : 2
%     leg_lengthR(i)=find_leg_length(kinect_data, train_data, pend_ind.r(i,:),'r')
%     leg_lengthL(i)=find_leg_length(kinect_data, train_data, pend_ind.l(i,:),'l')
% end
% leg_length = median([leg_lengthR leg_lengthL])
save(strcat(dir_name,'/leg_length'), 'leg_length');
end

function leglength = calculate_leg_length(hs, train_data, kinect_data)
% left leg length
leglength = [];

for i = 1 : length(hs.l)
    leglength = [leglength,...
        calculate_one_leglength(hs.l(i), hs.r, train_data.left.quat, kinect_data, train_data)];
end

% right leg length
for i = 1 : length(hs.r)
    leglength = [leglength,...
        calculate_one_leglength(hs.r(i), hs.l, train_data.right.quat, kinect_data, train_data)];
end

leglength(leglength==0) = [];
end

function leg_length = calculate_one_leglength(hs_ind, oppo_side_hs, quat, kinect_data, train_data)


for j = 1 : length(oppo_side_hs)
    if oppo_side_hs(j) > hs_ind
        break;
    end
end

indA = hs_ind;
indB = oppo_side_hs(j);


if indA < indB 
    
    pos_1 = get_kinect_position(kinect_data, train_data.timestamp(indA));
    pos_2 = get_kinect_position(kinect_data, train_data.timestamp(indB));
    step_length = norm(pos_1-pos_2);

    wlen=5;
    q1 = median(quat(indA-wlen:indA+wlen, :));
    q2 = median(quat(indB-wlen:indB+wlen, :));
    quat_diff = quatmultiply(q1,quatconj(q2));
    leg_length = step_length / sin(acos(quat_diff(1))) / 2;

else
    leg_length = 0;
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
    define_pendulum_index(hs, start_ind, end_ind)
if hs.l(1) < hs.r(1) && hs.l(end) < hs.r(end)
    % case 1
    startI = start_ind.right; endI = end_ind.left;
    right_pendulum_index = [startI hsl(1);...
        hs.r(1:end-1) hs.l(2:end)];
    left_pendulum_index = [hs.l(1:end-1) hs.r(1:end-1); hs.l(end) endI];
elseif hs.l(1) < hs.r(1) && hs.l(end) > hs.r(end)
    % case 2
    startI = start_ind.right; endI = end_ind.right;
    right_pendulum_index = [startI hs.l(1);  ...
        hs.r(1:end-1) hs.l(2:end-1);hs.r(end) endI];
    left_pendulum_index = [hs.l(1:end-1) hs.r];
elseif hs.l(1) > hs.r(1) && hs.l(end) < hs.r(end)
    % case 3
    startI = start_ind.left; endI = end_ind.left;
    right_pendulum_index = [hs.r(1:end-1) hs.l];
    left_pendulum_index = [startI hs.r(1);...
        hs.l(1:end-1) hs.r(2:end-1); hs.l(end) endI];
else
    startI = start_ind.left; endI = end_ind.right;
    right_pendulum_index = [hs.r(1:end-1) hs.l(1:end-1);...
        hs.r(end) endI];    
    left_pendulum_index = [startI hs.r(1); hs.l(1:end-1) hs.r(2:end)];
end
end

function [pend_ind] = def_pend_ind_train(hs, start_ind, end_ind)


if hs.l(1) < hs.r(1)
% left foot step first
    leftInd = 1;
    rightInd = 1;
    pend_ind.r = [start_ind.right hs.l(1)];
    leftInd = leftInd+1;
    while leftInd <= length(hs.l) && rightInd <= length(hs.r)
        pend_ind.r = [pend_ind.r;hs.r(rightInd) hs.l(leftInd)];
        rightInd = rightInd + 1;
        leftInd = leftInd + 1; 
    end
    leftInd = 1;
    rightInd = 1;
    pend_ind.l = [];
    while leftInd <= length(hs.l) && rightInd <= length(hs.r)
        pend_ind.l = [pend_ind.l;hs.l(leftInd) hs.r(rightInd)];
        rightInd = rightInd + 1;
        leftInd = leftInd + 1; 
    end
else
% right foot step first
    leftInd = 1;
    rightInd = 1;
    pend_ind.l = [start_ind.left hs.r(1)];
    rightInd = rightInd+1;
    while leftInd <= length(hs.l) && rightInd <= length(hs.r)
        pend_ind.l = [pend_ind.l;hs.l(leftInd) hs.r(rightInd)];
        rightInd = rightInd + 1;
        leftInd = leftInd + 1; 
    end
    leftInd = 1;
    rightInd = 1;
    pend_ind.r = [];
    while leftInd <= length(hs.l) && rightInd <= length(hs.r)
        pend_ind.r = [pend_ind.r;hs.r(rightInd) hs.l(leftInd)];
        rightInd = rightInd + 1;
        leftInd = leftInd + 1; 
    end
end



end

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

subplot(2,1,2); plot(acc_re); hold on;
for i = 1 : length(toe_off.right)-1
    [kkr,~]=v_findpeaks(acc_re(toe_off.right(i):toe_off.right(i+1)),'q',100);
    hs.r(i)=kkr(1)+toe_off.right(i)-1;
    plot([hs.r(i) hs.r(i)],get(gca,'YLim'),'g');
end
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