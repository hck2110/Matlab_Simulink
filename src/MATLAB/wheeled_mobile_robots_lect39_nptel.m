clear all;
close all;
clc;
dt=0.1;
ts=4;
t=0:dt:ts;
eta(:,1)=[0;0;0];
eta_d(:,1)=[-1;-1;-pi/4];
K_u=2;K_r=4;
a=0.05;d=0.2;l=0.3;
for i=1:length(t)
    rho=sqrt((eta_d(1)-eta(1,i))^2+(eta_d(2)-eta(2,i))^2);
    e=[eta_d(1:2);atan2(eta_d(2)-eta(2,i),eta_d(1)-eta(1,i))]-eta(:,i);
    % if rho<0.05
    %     e=eta_d-eta(:,i);
    %    % e=0*e;
    % end
    psi=eta(3,i);
    J=[cos(psi),-sin(psi),0;
       sin(psi),cos(psi),0;
       0,0,1];
zeta(:,i)=inv(J)*(diag([2,2,4])*e);
u=zeta(1,i);r=zeta(3,i);
% W=[a/2,a/2;
%     -a/(2*d),a/(2*d)];
%  w=inv(W)*[u;r];
%  zetam=W*w;
%  u=zetam(1);r=zetam(2);
%  phi(i)=atand(l*r/u);
%  V(i)=u;
%  W=[cos(psi);sin(psi);tand(phi(i))/1];
%  zeta(:,i)=W*V(i);
 eta(:,i+1)=eta(:,i)+(1-exp(-1*t(i)))*[cos(eta(3,i)),0;sin(eta(3,i)),0;0,1]*[u;r]*dt;
%eta(:,i+1)=eta(:,i)+(1-exp(-1*t(i)))*J*zeta(:,i)*dt;
end
veh_box=0.5*[-0.4 0.6 0.6+0.3*cosd(-90:90) 0.6 -0.4 -0.4;-0.3,-0.3,0.3*sind(-90:90),0.3,0.3,-0.3]
cas_p=0.5*[0.6;0];
wheel_b=0.5*[-0.2 0.2 0.2 -0.2 -0.2;-0.05 -0.05 0.05 0.05 -0.05];
 
for i=1:length(t)
    psi=eta(3,i);
    x(i)=eta(1,i);
    y(i)=eta(2,i);
    R=[cos(psi),-sin(psi);sin(psi),cos(psi)];
    v_m=R*veh_box;
    c_m=R*cas_p;
    w_m1=R*(wheel_b+[0;0.35/2]);
    w_m2=R*(wheel_b+[0;-0.35/2])
    fill(v_m(1,:)+x(i),v_m(2,:)+y(i),'y');
    hold on
    fill(w_m1(1,:)+x(i),w_m1(2,:)+y(i),'r');
    fill(w_m2(1,:)+x(i),w_m2(2,:)+y(i),'r');
    fill(c_m(1)+0.05*cosd(0:360)+x(i),c_m(2)+0.05*sind(0:360)+y(i),'g');
    plot(eta(1,1:i),eta(2,1:i),'b-')
    plot(eta_d(1),eta_d(2),'k*')
    plot([eta_d(1),eta_d(1)+0.2*cos(eta_d(3))],[eta_d(2),eta_d(2)+0.2*sin(eta_d(3))])
    plot(eta_d(1)+0.1*cosd(0:360),eta_d(2)+0.1*sind(0:360),'c--')
    xmin=min(eta(1,:))-0.5;
    xmax=max(eta(1,:))+0.5;
    ymin=min(eta(2,:))-0.5;
    ymax=max(eta(2,:))+0.5;
    axis([xmin xmax ymin ymax])
    axis equal
    grid on
    xlabel('x,[m]')
    ylabel('y,[m]');
    pause(0.1)
    hold off
    
    
end

    