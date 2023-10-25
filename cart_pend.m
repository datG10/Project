clear all, close all, clc

m = 0.23;
M = 2.4;
l = 0.36;
g = 9.81;

para = [m,M,l,g];

%x0 = [pi; 0.1;2; 0];
x0 = [pi; 0;2; 0];
%x0 = [pi; 0.01;2; 0];
%x0 = [0; 0;2; 0.1];

[A,B] = Jacobian(x0,0,para);
C = [1 0 0 0; 0 0 1 0];
D = [0;0];

sys=ss(A,B,C,D);
t=0:0.05:10; 
u=zeros(size(t));
%u=0.1*ones(size(t));
[y,t,x]=lsim(sys,u,t,x0);


%Inverted Pendlume after LQR Design
Q =eye(4);%equal penalty for state 
% if u need more agressive and fast tune Q

%Q = 0.1*[C'*C];
%R = 1;
R = 0.01;% less input penalty
[K,S,P] = lqr(sys,Q,R);
%closed loop system using optimal feedback gain K
Ac = A- B*K;
sysc=ss(Ac,B,C,D);
[yc,t,xc]=lsim(sysc,u,t,x0);

%robot open loop animation

 % f1 = figure();
 % for i=1:length(t)
 % plot_robot(x(i,:),para);
 % end
 % 
 %pole zero map for open loop system

 % 
 % f1= figure();
 % for i=1:length(t)
 % plot_robot(xc(i,:),para);
 % end

% Set the figure size explicitly
f1 = figure('Position', [100, 100, 701, 280]);
hold on
f = cell(length(t),1); 

% Open the video writer object
obj = VideoWriter('pend3.avi');
obj.Quality = 100;
obj.FrameRate = 30;

open(obj);

% Write the frames to the video file
for i = 1:length(x)
    % Set the figure size again before capturing the frame
    set(gcf, 'Position', [100, 100, 1250, 750]);
    plot_robot(xc(i,:),para);
    f{i} = getframe(gcf);
    writeVideo(obj, f{i});
end

% Close the video writer object
close(obj);
% responces of open loop system
f5 = figure();
subplot(4,1,1)
plot(t,x(:,1),'r','Linewidth',2,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',10)
ylabel('\theta(rad)');
xlabel('time(s)')
grid on

subplot(4,1,2)
plot(t,x(:,2),'b','Linewidth',2,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',10)
ylabel('\omega(rad/s)');
xlabel('time(s)')
grid on

subplot(4,1,3)
plot(t,x(:,3),'g','Linewidth',2,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',10)
ylabel('x(m)');
xlabel('time(s)')
grid on

subplot(4,1,4)
plot(t,x(:,4),'m','Linewidth',2,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',10)
ylabel('v (m/s)');
xlabel('time(s)')
grid on
% responces of closed loop system
f6 = figure();
title('step responces')
subplot(4,1,1)
plot(t,xc(:,1),'r','Linewidth',2,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',10)
ylabel('\theta(rad)');
xlabel('time(s)')
grid on

subplot(4,1,2)
plot(t,xc(:,2),'b','Linewidth',2,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',10)
ylabel('\omega(rad/s)');
xlabel('time(s)')
grid on

subplot(4,1,3)
plot(t,xc(:,3),'g','Linewidth',2,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',10)
ylabel('x(m)');
xlabel('time(s)')
grid on

subplot(4,1,4)
plot(t,xc(:,4),'m','Linewidth',2,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',10)
ylabel('v (m/s)');
xlabel('time(s)')
grid on

%function computing Jacobian

function [Jx,Ju] = Jacobian(x,u,para)

m = para(1);
M = para(2);
l = para(3);
g = para(4);
x1 = x(1);
x2 = x(2);

d1 =(l*(M + m) - l*m*cos(x1)^2);  
a = (- l*m*x2^2*cos(x1)^2 + l*m*x2^2*sin(x1)^2 + g*(M + m)*cos(x1) + u*sin(x1));
b = (2*l*m*cos(x1)*sin(x1)*(l*m*cos(x1)*sin(x1)*x2^2 + u*cos(x1) - g*sin(x1)*(M + m)));

jx21 = (a/d1) +(b/d1^2);
jx22 = - (2*l*m*x2*cos(x1)*sin(x1))/d1;

d2 = (- m*cos(x1)^2 + M + m);
c = (l*m*x2^2*cos(x1) - g*m*cos(x1)^2 + g*m*sin(x1)^2)/d2;
d = (2*m*cos(x1)*sin(x1)*(l*m*sin(x1)*x2^2 + u - g*m*cos(x1)*sin(x1)))/d2^2;
jx41 = c-d;
jx42 = (2*l*m*x2*sin(x1))/d2;


ju21 =-cos(x1)/(l*(M + m) - l*m*cos(x1)^2);
ju41 =1/(- m*cos(x1)^2 + M + m);

Jx = [ 0         1       0      0;
       jx21      jx22    0      0;
       0          0      0      1;
       jx41       jx42   0      0];
   
Ju = [0;ju21;0;ju41];
   
end

% function plotting robot
function plot_robot(x,para)

m = para(1);
M = para(2);
l = para(3);

W = M/5; 
H = M/10; 
wr = 0.1;         
mr = 0.3*sqrt(m); 

y = 0;
xp = x(3) + l*sin(x(1));
yp = y+l*cos(x(1));

plot([-10 10],[-H-wr/2 -H-wr/2],'k','LineWidth',2), hold on
rectangle('Position',[x(3)-W/2,-H,W,H],'Curvature',.1,'FaceColor',[.5 0.5 1],'LineWidth',1.5); % Draw cart
rectangle('Position',[x(3)-.9*W/2,-H-wr/2,wr,wr],'Curvature',1,'FaceColor',[0 0 0],'LineWidth',1.5); % Draw wheel
rectangle('Position',[x(3)+.9*W/2-wr,-H-wr/2,wr,wr],'Curvature',1,'FaceColor',[0 0 0],'LineWidth',1.5); % Draw wheel
plot([x(3) xp],[0 yp],'k','LineWidth',2); % Draw pendulum
rectangle('Position',[xp-mr/2,yp-mr/2,mr,mr],'Curvature',1,'FaceColor',[1 0.1 .1],'LineWidth',1.5);

axis([-5 5 -0.5 0.5]);
axis equal
set(gcf,'Position',[100 100 1000 600])
drawnow
hold off

end

