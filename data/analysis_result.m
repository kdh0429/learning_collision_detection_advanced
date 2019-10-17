clc
clear all
cd ..
cd ..
cd Real_Robot_Code
cd 2nd
cd build_ws_impact_cut
Result = load("Result.txt");

l = Result(:,8:14) - Result(:,1:7);
logit = sum(Result(:,8:14) - Result(:,1:7),2)+Result(:,16)-Result(:,15);
res = 1./(1+exp(logit));


exp_modular_value = exp(Result(:,8:14) - Result(:,1:7));
for i=1:7
    subplot(2,4,i)
    plot(exp_modular_value(:,i))
    hold off
end

subplot(2,4,8)
plot(l)
hold on
plot(Result(:,17))
hold off