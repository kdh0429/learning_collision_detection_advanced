clear all
clc
format long

Original_hz = 1000;
Reduced_hz = 100;
Reduced_interval = Original_hz / Reduced_hz;

cd ..
DoosanMSeries_RoboticsToolBox_Simulator;
cd TestSet

%% Collision
TestData = load('DRCL_Data_Test.txt');
ReducedData = zeros(fix(size(TestData,1)/Reduced_interval), size(TestData,2));
for row = 1:size(ReducedData,1)
    ReducedData(row,:) = TestData(Reduced_interval*(row-1)+1,:);
end

% 가속도 보상 & 동적토크 모터 관성 추가
AngleM = ReducedData(:,8:13);
VelD = ReducedData(:,26:31);
ToolAccM = ReducedData(:,62:64);
TorqueDyn = ReducedData(:,32:37);

ToolGrav = zeros(size(ReducedData,1),3);
ToolAccD = zeros(size(ReducedData,1),3);
BaseAccD = zeros(size(ReducedData,1),6);
AccD = zeros(size(ReducedData,1),6);
for m = 1:size(ReducedData,1)
    R_T = bot.fkine(AngleM(m,:)).R()';
    if m == 1
        AccD(m,:) = (VelD(m+1,:) - VelD(m,:))*Reduced_hz;
    else
        AccD(m,:) = (VelD(m,:) - VelD(m-1,:))*Reduced_hz;
    end
    ToolGrav(m,:) = (R_T*[0.0;0.0; -9.80])';
    BaseAccD(m,:) = bot.jacob0(AngleM(m,:))*AccD(m,:)' + bot.jacob_dot(AngleM(m,:),VelD(m,:));
    BaseAccD(m,:) = bot.jacob0(AngleM(m,:))*AccD(m,:)' + bot.jacob_dot(AngleM(m,:),VelD(m,:));
    ToolAccD(m,:) = (R_T*BaseAccD(m,1:3)')';
    if mod(m, 10000) == 0
        m
    end
end
for j = 1:6
    TorqueDyn(:,j) = TorqueDyn(:,j) + bot.links(j).Jm*bot.links(j).G*bot.links(j).G * AccD(:,j);
end
ReducedData(:,62:64) = ToolAccM+ToolGrav-ToolAccD;
ReducedData(:,32:37) = TorqueDyn;

save('Reduced_DRCL_Data_Test.txt', 'ReducedData', '-ascii', '-double', '-tabs')

clear all;

%% Free
Original_hz = 1000;
Reduced_hz = 100;
Reduced_interval = Original_hz / Reduced_hz;

cd ..
DoosanMSeries_RoboticsToolBox_Simulator;
cd TestSet

TestData = load('DRCL_Data_Test_Free.txt');
ReducedData = zeros(fix(size(TestData,1)/Reduced_interval), size(TestData,2));
for row = 1:size(ReducedData,1)
    ReducedData(row,:) = TestData(Reduced_interval*(row-1)+1,:);
end

% 가속도 보상 & 동적토크 모터 관성 추가
AngleM = ReducedData(:,8:13);
VelD = ReducedData(:,26:31);
ToolAccM = ReducedData(:,62:64);
TorqueDyn = ReducedData(:,32:37);

ToolGrav = zeros(size(ReducedData,1),3);
ToolAccD = zeros(size(ReducedData,1),3);
BaseAccD = zeros(size(ReducedData,1),6);
AccD = zeros(size(ReducedData,1),6);
for m = 1:size(ReducedData,1)
    R_T = bot.fkine(AngleM(m,:)).R()';
    if m == 1
        AccD(m,:) = (VelD(m+1,:) - VelD(m,:))*Reduced_hz;
    else
        AccD(m,:) = (VelD(m,:) - VelD(m-1,:))*Reduced_hz;
    end
    ToolGrav(m,:) = (R_T*[0.0;0.0; -9.80])';
    BaseAccD(m,:) = bot.jacob0(AngleM(m,:))*AccD(m,:)' + bot.jacob_dot(AngleM(m,:),VelD(m,:));
    BaseAccD(m,:) = bot.jacob0(AngleM(m,:))*AccD(m,:)' + bot.jacob_dot(AngleM(m,:),VelD(m,:));
    ToolAccD(m,:) = (R_T*BaseAccD(m,1:3)')';
    if mod(m, 10000) == 0
        m
    end
end
for j = 1:6
    TorqueDyn(:,j) = TorqueDyn(:,j) + bot.links(j).Jm*bot.links(j).G*bot.links(j).G * AccD(:,j);
end
ReducedData(:,62:64) = ToolAccM+ToolGrav-ToolAccD;
ReducedData(:,32:37) = TorqueDyn;

save('Reduced_DRCL_Data_Test_Free.txt', 'ReducedData', '-ascii', '-double', '-tabs')