 %% Dynamic model of Land based mobile robot   

 clear all;
 clc;
 close all;
%% Simualtion Parametrs
dt=0.1;
ts=100;
t=0:dt:ts;

%% Initial Conditions
eta0 = [0;0;pi/4];
zeta0=[0;0;0];
eta(:,1)= eta0;
zeta(:,1) = zeta0;

%% Robot Parameters
m=10; %mass of the vehicle
Iz=0.1;  %Inertia of the vehicle
xbc=0;ybc=0;
d=0.5;
%% State propogation
for i=1:length(t)
    u=zeta(1,i);
    v=zeta(2,i);
    r=zeta(3,i);
    %% Inertia Matrix
    D=[m,0,-m*ybc;
        0,m,m*xbc;
        -m*ybc,m*xbc,Iz+m*(xbc^2+ybc^2);];
n_v=[-m*r*(v+xbc*r);
      m*r*(u-ybc*r);
      m*r*(xbc*u+ybc*v);];

%% Input Vector (forward dynamics)
tau(:,i)=[1;0.5;0];

%% Jacobian Matrix
psi=eta(3,i);
J_eta=[cos(psi),-sin(psi),0;
       sin(psi),cos(psi),0;
       0,0,1];
%%zeta_dot(:,i)=inv(D)*(tau(:,i)-n_v);
zeta_dot(:,i)=inv(D)*(tau(:,i)-n_v-0.5*zeta(:,i)); %% from equation 2
zeta(:,i+1)= zeta (:,i)+dt*zeta_dot(:,i); %velocity update
eta(:,i+1)=eta(:,i)+dt*(J_eta*(zeta(:,i)+dt*zeta_dot(:,i))) % state update
end
%%Animaton(mobile robot motion animation)
l=0.4;
w=2*d;
%mobile robot coordinates
mr_co=[-l/2,l/2,l/2,-l/2,-l/2;
       -w/2,-w/2,w/2,w/2,-w/2;];
figure
for i=1:5:length(t) %animation starts here
    psi=eta(3,i)
    R_psi=[cos(psi) -sin(psi);
           sin(psi)   cos(psi);];
    v_pos=R_psi*mr_co;
    fill(v_pos(1,:)+eta(1,i),v_pos(2,:)+eta(2,i),'g');
    hold on;
    grid on;
    axis([-1 10 -1 10]);
    axis square;
    plot(eta(1,1:i),eta(2,1:i),'b')
    legend('MR','Path')
    set(gca,'fontsize',24)
    xlabel('x,[m]');
    ylabel('y,[m]');
    pause(0.01);
    hold off
end
 figure
plot(t,eta(1,1:i),'r-',t,eta(2,1:i),'b-.',t,eta(3,1:i),'k--');
legend('x,[m]','y,[m]','\psi,[rad]');
set(gca,'fontsize',24)
grid on
 xlabel('t[sec]');
 ylabel('\eta[units]');
 
 figure
 plot(t,zeta(1,1:i),'r-',t,zeta(2,1:i),'b-.',t,eta(3,1:i),'k--');
legend('u[m/s]','v[m/s]','r[rad/s]');
 set(gca,'fontsize',24)
 grid on
 xlabel('t[sec]');
 ylabel('\zeta[units]');
% 
% figure
% plot(t,eta(3,1:i),'g-');
% set(gca,'fontsize',24)
% xlabel('t[sec]');
% ylabel('\psi[rad]');