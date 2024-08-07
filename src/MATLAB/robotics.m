clear all;
clc;
close all;
dt=0.1;
ts=10;
t=0:dt:ts;
x0=0.5;
y0=0.5;
psi0=pi/4;
eta0=[x0;y0;psi0];
eta(:,1)=eta0;


%%Loop starts here
for i=1:length(t)
    psi=eta(3,i);
    J_psi=[cos(psi) -sin(psi) 0;
           sin(psi) cos(psi)  0;
            0         0       1];
    %inputs
    u=0.3;
    v=0;
    r=0.2;
    %velocity input commands
    zeta(:,i)=[u;v;r];
     %time derivative of generailsed coordinates
    eta_dot(:,i)=J_psi*zeta(:,i);

    %%Position propogation using Euler Method
    eta(:,i+1)=eta(:,i)+dt*eta_dot(:,i); %Status update
end
figure
plot(t,eta(1,1:i),'r-');
set(gca,'fontsize',24)
xlabel('t[sec]');
ylabel('x[m]');

figure
plot(t,eta(2,1:i),'b-');
set(gca,'fontsize',24)
xlabel('t[sec]');
ylabel('y[m]');

figure
plot(t,eta(3,1:i),'g-');
set(gca,'fontsize',24)
xlabel('t[sec]');
ylabel('\psi[rad]');
plot(t,eta(1,1:i),'r-');
hold on;
plot(t,eta(2,1:i),'b--');
hold on;
plot(t,eta(3,1:i),'m-.');
legend('x,[m]','y,[m]','\psi,[rad]');
set(gca,'fontsize',24)
ylabel('\eta');
xlabel('t[s]');

%Animaton(mobile robot motion animation)
l=0.6;
w=0.4;
%%mobile robot coordinates
mr_co=[-l/2,l/2,l/2,-l/2,-l/2;
       -w/2,-w/2,w/2,w/2,-w/2;];
figure
for i=1:length(t) %animation starts here
    psi=eta(3,i)
    R_psi=[cos(psi) -sin(psi);
           sin(psi)   cos(psi);];
    v_pos=R_psi*mr_co;
    fill(v_pos(1,:)+eta(1,i),v_pos(2,:)+eta(2,i),'g');
    hold on;
    grid on;
    axis([-1 3 -1 3]);
    axis square;
    plot(eta(1,1:i),eta(2,1:i),'b')
    legend('MR','Path')
    set(gca,'fontsize',24)
    xlabel('x,[m]');
    ylabel('y,[m]');
    pause(0.1);
    hold off
end

