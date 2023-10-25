m1 = 0.23;
l1 = 0.36;
m2 = 0.23;
l2 = 0.36;
g = 9.81;

x0 = [pi/6;0;-pi/6,;0];

f1 = figure();
plot_pend(x0,m1,m2,l1,l2)


% 
% t = 0:.01:1;
% 
% 
% [t,x] = ode45(@(t,x)invertedPend(x,g,l,u,m),t,x0);
% 
%  f2 = figure();
% 
%  for i=1:length(t)
% 
%     plot_pend(x(i,:),g,l);
% 
%  end

function dx = invertedPend(x,g,l,u,m)

dx(1,1) = x(2);
dx(2,1) = -(g/l)*sin(x(1))+u/m*l^2;
dx(3,1) = x(4);
dx(4,1) = -(g/l)*sin(x(1))+u/m*l^2;


end


function plot_pend(x,m1,m2,l1,l2)

xp = l1*sin(x(1));
yp = -l1*cos(x(1));


xp2 = l1*sin(x(1))+l2*sin(x(3));
yp2 = -l1*cos(x(1))-l2*cos(x(3));

mr = 0.3*sqrt(m1); 
mr1 = 0.3*sqrt(m2); 

plot([0,0],[-5 5],'k','LineWidth',1,'LineStyle','--'), hold on
plot([-5 5],[0,0],'k','LineWidth',1,'LineStyle','--'), hold on
plot([0 xp],[0 yp],'k','LineWidth',2); % Draw pendulum
plot([xp xp2],[yp yp2],'k','LineWidth',2); % Draw pendulum
rectangle('Position',[xp-mr/2,yp-mr/2,mr,mr],'Curvature',1,'FaceColor',[1 0.1 .1],'LineWidth',1.5);
rectangle('Position',[xp2-mr1/2,yp2-mr1/2,mr1,mr1],'Curvature',1,'FaceColor',[1 0.1 .1],'LineWidth',1.5);
axis([-5 5 -0.5 0.5]);
axis equal
set(gcf,'Position',[100 100 1000 400])
drawnow
hold off

end
