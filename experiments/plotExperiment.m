clear
close all
clc
%% experiment data
startRow = 1;
experimentP = 1;
experimentD = 2;
filenameP= ['experiment',num2str(experimentP),'.txt'];
filenameD= ['experiment',num2str(experimentD),'.txt'];
[timeP,evXhP,evYhP,evZhP,evXmP,evYmP,evZmP,qTildeWhP,qTildeXhP,qTildeYhP,qTildeZhP,qTildeWmP,qTildeXmP,qTildeYmP,qTildeZmP,zStarP,zStarHathP,zStarHatmP,zTildehP,zTildemP,desImageWrtImageZhatPXP,desImageWrtImageZhatPYP,desImageWrtImageZhatPZP,desImageWrtImagePXP,desImageWrtImagePYP,desImageWrtImagePZP,desImageWrtImageZhatQWP,desImageWrtImageZhatQXP,desImageWrtImageZhatQYP,desImageWrtImageZhatQZP,desImageWrtImageQWP,desImageWrtImageQXP,desImageWrtImageQYP,desImageWrtImageQZP,vcXhP,vcYhP,vcZhP,vcXmP,vcYmP,vcZmP,wcXhP,wcYhP,wcZhP,wcXmP,wcYmP,wcZmP] = readExperiment(filenameP, startRow);
[timeD,evXhD,evYhD,evZhD,evXmD,evYmD,evZmD,qTildeWhD,qTildeXhD,qTildeYhD,qTildeZhD,qTildeWmD,qTildeXmD,qTildeYmD,qTildeZmD,zStarD,zStarHathD,zStarHatmD,zTildehD,zTildemD,desImageWrtImageZhatPXD,desImageWrtImageZhatPYD,desImageWrtImageZhatPZD,desImageWrtImagePXD,desImageWrtImagePYD,desImageWrtImagePZD,desImageWrtImageZhatQWD,desImageWrtImageZhatQXD,desImageWrtImageZhatQYD,desImageWrtImageZhatQZD,desImageWrtImageQWD,desImageWrtImageQXD,desImageWrtImageQYD,desImageWrtImageQZD,vcXhD,vcYhD,vcZhD,vcXmD,vcYmD,vcZmD,wcXhD,wcYhD,wcZhD,wcXmD,wcYmD,wcZmD] = readExperiment(filenameD, startRow);

figure('Position',[100 100 640 480])
plot(timeP,zTildehP./zStarP,'b.',timeP,zTildemP./zStarP,'r.','MarkerSize',20)
xlabel('time, t (sec)','Interpreter','latex','FontSize',20,'FontName','Times New Roman')
ylabel({'$\tilde{z}_{1}^{*}$ (m)'},'Interpreter','latex','FontSize',20,'FontName','Times New Roman')
set(gca,'FontSize',20,'FontName','Times New Roman')

figure('Position',[100 100 640 480])
plot(timeP,desImageWrtImagePXP,'r.',timeP,desImageWrtImagePYP,'g.',timeP,desImageWrtImagePZP,'b.','MarkerSize',20)
% xlabel('time, t (sec)','Interpreter','latex','FontSize',20,'FontName','Times New Roman')
% ylabel({'$\tilde{z}_{1}^{*}$ (m)'},'Interpreter','latex','FontSize',20,'FontName','Times New Roman')
set(gca,'FontSize',20,'FontName','Times New Roman')

figure('Position',[100 100 640 480])
plot(timeP,desImageWrtImageZhatPXP,'r.',timeP,desImageWrtImageZhatPYP,'g.',timeP,desImageWrtImageZhatPZP,'b.','MarkerSize',20)
% xlabel('time, t (sec)','Interpreter','latex','FontSize',20,'FontName','Times New Roman')
% ylabel({'$\tilde{z}_{1}^{*}$ (m)'},'Interpreter','latex','FontSize',20,'FontName','Times New Roman')
set(gca,'FontSize',20,'FontName','Times New Roman')

figure('Position',[100 100 640 480])
plot(timeD,zTildehD./zStarD,'b.',timeD,zTildemD./zStarD,'r.','MarkerSize',20)
xlabel('time, t (sec)','Interpreter','latex','FontSize',20,'FontName','Times New Roman')
ylabel({'$\tilde{z}_{1}^{*}$ (m)'},'Interpreter','latex','FontSize',20,'FontName','Times New Roman')
set(gca,'FontSize',20,'FontName','Times New Roman')

figure('Position',[100 100 640 480])
plot(timeD,desImageWrtImagePXD,'r.',timeD,desImageWrtImagePYD,'g.',timeD,desImageWrtImagePZD,'b.','MarkerSize',20)
% xlabel('time, t (sec)','Interpreter','latex','FontSize',20,'FontName','Times New Roman')
% ylabel({'$\tilde{z}_{1}^{*}$ (m)'},'Interpreter','latex','FontSize',20,'FontName','Times New Roman')
set(gca,'FontSize',20,'FontName','Times New Roman')

figure('Position',[100 100 640 480])
plot(timeD,desImageWrtImageZhatPXD,'r.',timeD,desImageWrtImageZhatPYD,'g.',timeD,desImageWrtImageZhatPZD,'b.','MarkerSize',20)
% xlabel('time, t (sec)','Interpreter','latex','FontSize',20,'FontName','Times New Roman')
% ylabel({'$\tilde{z}_{1}^{*}$ (m)'},'Interpreter','latex','FontSize',20,'FontName','Times New Roman')
set(gca,'FontSize',20,'FontName','Times New Roman')

% axis([0 85 -3 3])
% 
% figure(2)
% subplot(2,1,1)
% plot(time,qtildew,'b.','MarkerSize',20)
% xlabel('time, t (sec)','Interpreter','latex','FontSize',20,'FontName','Times New Roman')
% ylabel({'$\tilde{q}_{0}$ '},'Interpreter','latex','FontSize',20,'FontName','Times New Roman')
% set(gca,'FontSize',20,'FontName','Times New Roman')
% 
% subplot(2,1,2)
% plot(time,qtildeijknorms,'b.','MarkerSize',20)
% xlabel('time, t (sec)','Interpreter','latex','FontSize',20,'FontName','Times New Roman')
% ylabel({' $\left \| \tilde{q}_{v} \right \|$ '},'Interpreter','latex','FontSize',20,'FontName','Times New Roman')
% set(gca,'FontSize',20,'FontName','Times New Roman')
% 
% figure(3)
% plot(time,evxyznorms,'b.','MarkerSize',20)
% xlabel('time, t (sec)','Interpreter','latex','FontSize',20,'FontName','Times New Roman')
% ylabel({' $\left \| e_{v} \right \|$ '},'Interpreter','latex','FontSize',20,'FontName','Times New Roman')
% set(gca,'FontSize',20,'FontName','Times New Roman')
% 
% figure()
% plot(time,left_bumper,'b.','MarkerSize',20)
% xlabel('time, t (sec)','Interpreter','latex','FontSize',20,'FontName','Times New Roman')
% ylabel({'left bumper'},'Interpreter','latex','FontSize',20,'FontName','Times New Roman')
% set(gca,'FontSize',20,'FontName','Times New Roman')
% 
% figure()
% plot(time,vc(:,1),'r.',time,vc(:,2),'g.',time,vc(:,3),'b.')
% xlabel('time, t (sec)','Interpreter','latex','FontSize',20,'FontName','Times New Roman')
% ylabel({'cam velocity translational'},'Interpreter','latex','FontSize',20,'FontName','Times New Roman')
% set(gca,'FontSize',20,'FontName','Times New Roman')
% 
% figure()
% plot(time,wc(:,1),'r.',time,wc(:,2),'g.',time,wc(:,3),'b.')
% xlabel('time, t (sec)','Interpreter','latex','FontSize',20,'FontName','Times New Roman')
% ylabel({'cam velocity angular'},'Interpreter','latex','FontSize',20,'FontName','Times New Roman')
% set(gca,'FontSize',20,'FontName','Times New Roman')
% 
% figure()
% plot(time,Pdescam_wrt_world(:,1),'r.',time,Pcalccam_wrt_world(:,1),'g.',time,Pactcam_wrt_world(:,1),'b.')
% xlabel('time, t (sec)','Interpreter','latex','FontSize',20,'FontName','Times New Roman')
% ylabel({'x position cam wrt world'},'Interpreter','latex','FontSize',20,'FontName','Times New Roman')
% legend('desired', 'calculated', 'actual');
% set(gca,'FontSize',20,'FontName','Times New Roman')
% 
% figure()
% plot(time,Pdescam_wrt_world(:,2),'r.',time,Pcalccam_wrt_world(:,2),'g.',time,Pactcam_wrt_world(:,2),'b.')
% xlabel('time, t (sec)','Interpreter','latex','FontSize',20,'FontName','Times New Roman')
% ylabel({'y position cam wrt world'},'Interpreter','latex','FontSize',20,'FontName','Times New Roman')
% legend('desired', 'calculated', 'actual');
% set(gca,'FontSize',20,'FontName','Times New Roman')
% 
% figure()
% plot(time,Pdescam_wrt_world(:,3),'r.',time,Pcalccam_wrt_world(:,3),'g.',time,Pactcam_wrt_world(:,3),'b.')
% xlabel('time, t (sec)','Interpreter','latex','FontSize',20,'FontName','Times New Roman')
% ylabel({'z position cam wrt world'},'Interpreter','latex','FontSize',20,'FontName','Times New Roman')
% legend('desired', 'calculated', 'actual');
% set(gca,'FontSize',20,'FontName','Times New Roman')