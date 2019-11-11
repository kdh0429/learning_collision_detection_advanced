clc

for j = 1:6
subplot(2,3,j)
plot(ReducedDRCLData(:,25+j)-ReducedDRCLData(:,13+j))
hold on
plot(0.02*ReducedDRCLData(:,65))
hold off
end

% for i = 1:size(ReducedDRCLData,1)
%    err_norm(i) = norm(ReducedDRCLData(i,62:64)); 
% end
% plot(err_norm)
% hold on
% plot(5*ReducedDRCLData(:,65))
% hold off

% DoosanMSeries_RoboticsToolBox_Simulator;
% 
% Data(:,1) = DRCLData1(:,1) - DRCLData1(1,1);
% 
% Time = DRCLData1(:,1);
% VelD = DRCLData1(:,26:31);
% AccD = zeros(size(DRCLData1,1),6);
% for m = 1:size(DRCLData1,1)
%     if m == 1
%         AccD(m,:) = (VelD(m+1,:) - VelD(m,:))*100.0;
%     else
%         AccD(m,:) = (VelD(m,:) - VelD(m-1,:))*100.0;
%     end
% end
% TorqueM = DRCLData1(:,2:7);
% TorqueDyn = DRCLData1(:,32:37);
% for m = 1:size(TorqueDyn,2)
%     TorqueDyn(:,m) = TorqueDyn(:,m) + bot.links(m).Jm*bot.links(m).G*bot.links(m).G * AccD(:,m);   % ���� ���� ��ũ ����
% end
% TorqueFric = DRCLData1(:,50:55);
% 
% f1 = figure();
% f2 = figure();
% 
% TitleName = 'Joint ';
% figure(f1);
% for m = 1:6
%     h(m) = subplot(2,3,m);
%     plot(DRCLData1(:,1),TorqueM(:,m),'r',DRCLData1(:,1),TorqueDyn(:,m),'b');
%     TitleName(7) = num2str(m);
%     title(TitleName);
%     xlabel('Time (s)');
%     ylabel('Torque (Nm)');
%     legend('Motor Torque', 'Dyn Torque');
% end
% 
% figure(f2);
% for m = 1:6
%     h(m+6) = subplot(2,3,m);
%     plot(DRCLData1(:,1),TorqueM(:,m),'r',DRCLData1(:,1),TorqueDyn(:,m)+TorqueFric(:,m),'b');
%     TitleName(7) = num2str(m);
%     title(TitleName);
%     xlabel('Time (s)');
%     ylabel('Torque (Nm)');
%     legend('Motor Torque', 'Dyn+Fric Torque');
% end
% 
% linkaxes(h,'x')