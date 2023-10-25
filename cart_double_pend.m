
m1 = 0.23;
m2 = 0.23;
M = 2.4;
l1 = 0.36;
l2 = 0.36;
g = 9.81;

x0 = [2;0;pi/3;0.01;-pi/6; 0.1];

a1 = -(m1*l1+m2*g*l1)/(m1+m2+M);
a2 = -m2*g*l2/((m1+m2)*l1^2);
a3 = g/l1;
a4 = 1/l1;
a5 = -m2*l2/(l1*(m1+m2));
a6 = 1/l2;
a7 = l1/l2;
a8 = g/l2;

b1 = 1/(m1+m2+M);
b2 = -1/(m1+m2)*l1;
b3 = -1/m2*l2;

A = [0   1   0   0   0    0;
     0   0   a1  0   a2   0;
     0   0   0   1   0    0;
     a3  a4  0   0   a5   0;
     0   0   0   0   0    1;
     0   a6  a7  0   a8   0];

B = [0;b1;0;b2;0;b3];

C = [1 0 0 0 0 0; 
     0 0 1 0 0 0;
     0 0 0 0 1 0];

D = [0;0;0];


sys=ss(A,B,C,D);
t=0:0.05:10; 
u=zeros(size(t));
%u=0.1*ones(size(t));
[y,t,x]=lsim(sys,u,t,x0);


% %Q =eye(6);
% 
% % %Q = 0.1*eye(6);
 Q = diag([1000,1,1,1,1,1]);
% %R = 0.01;
% 
R = 0.01;
 [K,S,P] = lqr(sys,Q,R);

%K = place(A,B,[-25 -14 -8 -7 -9 -2]);

%closed loop system using optimal feedback gain K
Ac = A- B*K;
sysc=ss(Ac,B,C,D);
[yc,t,xc]=lsim(sysc,u,t,x0);

 % plot_robot(x0,m1,m2,M,l1,l2);
 % f1 = figure();

 % for i=1:length(t)
 % 
 %    plot_robot(x(i,:),m1,m2,M,l1,l2);
 % 
 % end


% Set the figure size explicitly
f1 = figure('Position', [100, 100, 701, 280]);
hold on
f = cell(length(t),1); 

% Open the video writer object
obj = VideoWriter('pend2.avi');
obj.Quality = 100;
obj.FrameRate = 30;

open(obj);

% Write the frames to the video file
for i = 1:length(x)
    % Set the figure size again before capturing the frame
    set(gcf, 'Position', [100, 100, 1250, 750]);
    plot_robot(xc(i,:),m1,m2,M,l1,l2);
    f{i} = getframe(gcf);
    writeVideo(obj, f{i});
end

% Close the video writer object
close(obj);



 % f2 = figure();
 % 
 % for i=1:length(t)
 % 
 %    plot_robot(xc(i,:),m1,m2,M,l1,l2);
 % 
 % end

f3 = figure();

subplot(3,2,1)
plot(t,x(:,1),'r','Linewidth',2,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',10)
ylabel('x(m)');
xlabel('time(s)')
grid on

subplot(3,2,2)
plot(t,x(:,2),'b','Linewidth',2,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',10)
ylabel('v(m/s)');
xlabel('time(s)')
grid on

subplot(3,2,3)
plot(t,x(:,3),'g','Linewidth',2,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',10)
ylabel('\theta1(rad)');
xlabel('time(s)')
grid on

subplot(3,2,4)
plot(t,x(:,4),'m','Linewidth',2,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',10)
ylabel('\omega1(rad/s)');
xlabel('time(s)')
grid on

subplot(3,2,5)
plot(t,x(:,5),'c','Linewidth',2,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',10)
ylabel('\theta2(rad)');
xlabel('time(s)')
grid on

subplot(3,2,6)
plot(t,x(:,6),'y','Linewidth',2,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',10)
ylabel('\omega2 (rad/s)');
xlabel('time(s)')
grid on

% responces of closed loop system

f4 = figure();
title('step responces')
subplot(3,2,1)
plot(t,xc(:,1),'r','Linewidth',2,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',10)
ylabel('x(m)');
xlabel('time(s)')
grid on

subplot(3,2,2)
plot(t,xc(:,2),'b','Linewidth',2,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',10)
ylabel('v(m/s)');
xlabel('time(s)')
grid on

subplot(3,2,3)
plot(t,xc(:,3),'g','Linewidth',2,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',10)
ylabel('\theta1(rad)');
xlabel('time(s)')
grid on

subplot(3,2,4)
plot(t,xc(:,4),'m','Linewidth',2,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',10)
ylabel('\omega1 (rad/s)');
xlabel('time(s)')
grid on

subplot(3,2,5)
plot(t,xc(:,5),'c','Linewidth',2,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',10)
ylabel('\theta2(rad)');
xlabel('time(s)')
grid on

subplot(3,2,6)
plot(t,xc(:,6),'y','Linewidth',2,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',10)
ylabel('\omega2(rad/s))');
xlabel('time(s)')
grid on


function plot_robot(x,m1,m2,M,l1,l2)

W = M/5; 
H = M/10; 
wr = 0.1;         
mr = 0.3*sqrt(m1); 
mr1 =  0.3*sqrt(m2);

x1 = x(1) + l1*sin(x(3));
y1 = l1*cos(x(3));

x2 = x(1)+l1*sin(x(3))+l2*sin(x(5));
y2 = l1*cos(x(3))+l2*cos(x(5));

plot([-10 10],[-H-wr/2 -H-wr/2],'k','LineWidth',2), hold on
rectangle('Position',[x(1)-W/2,-H,W,H],'Curvature',.1,'FaceColor',[.5 0.5 1],'LineWidth',1.5); % Draw cart
rectangle('Position',[x(1)-.9*W/2,-H-wr/2,wr,wr],'Curvature',1,'FaceColor',[0 0 0],'LineWidth',1.5); % Draw wheel
rectangle('Position',[x(1)+.9*W/2-wr,-H-wr/2,wr,wr],'Curvature',1,'FaceColor',[0 0 0],'LineWidth',1.5); % Draw wheel
plot([x(1) x1],[0 y1],'k','LineWidth',2);
rectangle('Position',[x1-mr/2,y1-mr/2,mr,mr],'Curvature',1,'FaceColor',[0 0.1 .1],'LineWidth',1.5);
hold on
plot([x1 x2],[y1 y2],'r','LineWidth',2)
rectangle('Position',[x2-mr1/2,y2-mr1/2,mr1,mr1],'Curvature',1,'FaceColor',[1 0.1 .1],'LineWidth',1.5);

axis([-5 5 -1.0 1.5]);
axis equal
set(gcf,'Position',[100 100 1000 600])
drawnow
hold off

end