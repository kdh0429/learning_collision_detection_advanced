clc
% for j = 1:6
% subplot(2,3,j)
% plot(ReducedDRCLData(:,31+j)-ReducedDRCLData(:,1+j))
% hold on
% plot(10*ReducedDRCLData(:,65))
% end

for i = 1:size(ReducedDRCLData,1)
    acc_err(i) = norm(ReducedDRCLData(i,62:64));
end
plot(acc_err)
hold on
plot(10 *ReducedDRCLData(:,65))
hold off