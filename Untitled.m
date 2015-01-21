function hs_detection()
close all;
dir_name = '1202lidata1';
load(strcat(dir_name,'/test_data'));
gravity_s = test_data.left.gravity_s;
a= sum(gravity_s .* repmat([0 1 0], size(gravity_s,1),1),2);
thetal = acos(a/norm(gravity_s(1,:)));
acc = test_data.left.acc;
acc_e=acc.^2*[1;1;1];
[start_ind, end_ind] = walk_start_detection(acc,1);
clear gravity_s;
gravity_s = test_data.right.gravity_s;
acc = test_data.right.acc;
acc_e = acc.^2*[1;1;1];
a= sum(gravity_s .* repmat([0 1 0], size(gravity_s,1),1),2);
thetar = acos(max(a/norm(gravity_s(1,:)), repmat([-1],length(a),1)));

% figure; subplot(4,1,1);plot(thetal);subplot(4,1,2); plot(thetar);
% subplot(4,1,3); plot(test_data.left.acc); subplot(4,1,4); plot(test_data.right.acc);

[k,v]=v_findpeaks(thetar(start_ind:end_ind),'v',130);

toe_off = k+start_ind-1;
figure; subplot(2,1,1); plot(thetar);hold on; plot(toe_off,v,'r.');hold off;
subplot(2,1,2); plot(acc_e); hold on;

for i = 1 : length(toe_off)-1
    [kk,vv]=v_findpeaks(acc_e(toe_off(i):toe_off(i+1)),'q',100);
    hs=kk(1)+toe_off(i)-1;
    
    plot([hs hs],get(gca,'YLim'),'g');
    
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