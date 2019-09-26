clear all
clc
format long

cd ..
DoosanMSeries_RoboticsToolBox_Simulator;
cd TestSet

TestData = load('DRCL_Data_Test.txt');
ReducedData = zeros(fix(size(TestData,1)/10), size(TestData,2));
for row = 1:size(ReducedData,1)
    ReducedData(row,:) = TestData(10*(row-1)+1,:);
end

% 가속도 보상
AngleM = ReducedData(:,8:13);
VelD = ReducedData(:,26:31);
ToolAccM = ReducedData(:,62:64);

ToolGrav = zeros(size(ReducedData,1),3);
ToolAccD = zeros(size(ReducedData,1),3);
BaseAccD = zeros(size(ReducedData,1),6);
for m = 1:size(ReducedData,1)
    R_T = bot.fkine(AngleM(m,:)).R()';
    ToolGrav(m,:) = (R_T*[0.0;0.0; -9.80])';
    if m == 1
        BaseAccD(m,:) = bot.jacob0(AngleM(m,:))*(VelD(m+1,:) - VelD(m,:))'*100.0 + bot.jacob_dot(AngleM(m,:),VelD(m,:));
    else
        BaseAccD(m,:) = bot.jacob0(AngleM(m,:))*(VelD(m,:) - VelD(m-1,:))'*100.0 + bot.jacob_dot(AngleM(m,:),VelD(m,:));
    end
    ToolAccD(m,:) = (R_T*BaseAccD(m,1:3)')';
    if mod(m, 10000) == 0
        m
    end
end
ReducedData(:,62:64) = ToolAccM+ToolGrav-ToolAccD;

save('Reduced_DRCL_Data_Test.txt', 'ReducedData', '-ascii', '-double', '-tabs')