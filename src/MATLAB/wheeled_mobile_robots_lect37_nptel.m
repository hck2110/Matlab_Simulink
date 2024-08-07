 %% Dynamic model of Land based mobile robot   
 clear all;
 clc;
 close all;

dt=0.1;
ts=20;
t=0:dt:ts;

% Initial Conditions
eta(:,1)= [0;0;0.1];
% %Desired positions 
% eta_d(:,1)=[-1;-0.5;-pi/4];
% eta_desired_dot=[0;0;0];
%Vehicle parameters
a=0.05;d=0.2;l=0.2;

for i=1:length(t)
    %Desired positions trajectory 
eta_d(:,i)=[2*sin(0.1*t(i));2-2*cos(0.1*t(i));0];
eta_desired_dot=[0.2*cos(0.1*t(i));0.2*sin(0.1*t(i));0];
 eta_tilda=eta_d(:,i)-eta(:,i); %error
psi=eta(3,i);
J=[cos(psi),-sin(psi),0;
       sin(psi),cos(psi),0;
       0,0,1];
zeta(:,i)=inv(J)*(eta_desired_dot+diag([8,4,8])*eta_tilda);
W=[-a/3,-a/3,(2*a)/3;
     (sqrt(3)*a)/3,-(sqrt(3)*a)/3,sym(0);
     a/(3*l);a/(3*l);a/(3*l)];
 w=inv(W)*zeta(:,i);
 zeta(:,i)=W*w;
eta(:,i+1)=eta(:,i)+(1-exp(-1*t(i)))*J*zeta(:,i)*dt;
end
% l=0.4;
% w=2*d;
% %mobile robot coordinates
% mr_co=[-l/2,l/2,l/2,-l/2,-l/2;
%        -w/2,-w/2,w/2,w/2,-w/2;];
% figure
% for i=1:5:length(t) %animation starts here
%     psi=eta(3,i)
%     R_psi=[cos(psi) -sin(psi);
%            sin(psi)   cos(psi);];
%     v_pos=R_psi*mr_co;
%     fill(v_pos(1,:)+eta(1,i),v_pos(2,:)+eta(2,i),'g');
%     hold on;
%     grid on;
%     axis([-1 10 -1 10]);
%     axis square;
%     plot(eta(1,1:i),eta(2,1:i),'b')
%     legend('MR','Path')
%     set(gca,'fontsize',24)
%     xlabel('x,[m]');
%     ylabel('y,[m]');
%     pause(0.01);
%     hold off
% end
%  figure
% plot(t,eta(1,1:i),'r-',t,eta(2,1:i),'b-.',t,eta(3,1:i),'k--');
% legend('x,[m]','y,[m]','\psi,[rad]');
% set(gca,'fontsize',24)
% grid on
%  xlabel('t[sec]');
%  ylabel('\eta[units]');
% 
%  figure
%  plot(t,zeta(1,1:i),'r-',t,zeta(2,1:i),'b-.',t,eta(3,1:i),'k--');
% legend('u[m/s]','v[m/s]','r[rad/s]');
%  set(gca,'fontsize',24)
%  grid on
%  xlabel('t[sec]');
%  ylabel('\zeta[units]');
% % 
% % figure
% % plot(t,eta(3,1:i),'g-');
% % set(gca,'fontsize',24)
% % xlabel('t[sec]');
% % ylabel('\psi[rad]');
figure
subplot(3,1,1)
plot(t,eta_d(1,:),'r--',t,eta(1,1:i),'b')
legend('Desired','Actual');
subplot(3,1,2)
plot(t,eta_d(2,:),'r--',t,eta(2,1:i),'b')
legend('Desired','Actual');
subplot(3,1,3)
plot(t,eta_d(3,:),'r--',t,eta(3,1:i),'b')
legend('Desired','Actual');

