close all;
load('test_data');
gravity_s = test_data.left.gravity_s;
a= sum(gravity_s .* repmat([0 1 0], size(gravity_s,1),1),2);
thetal = acos(a/norm(gravity_s(1,:)));

clear gravity_s;
gravity_s = test_data.right.gravity_s;
a= sum(gravity_s .* repmat([0 1 0], size(gravity_s,1),1),2);
thetar = acos(max(a/norm(gravity_s(1,:)), repmat([-1],length(a),1)));

figure; subplot(4,1,1);plot(thetal);subplot(4,1,2); plot(thetar);
subplot(4,1,3); plot(test_data.left.acc); subplot(4,1,4); plot(test_data.right.acc);
 
