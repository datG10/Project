m = 0.23;
l = 1.36;
g = 9.81;
x0 = [pi/6;0];
sigma0 = 1;
omega = 1.0;
t = 0:0.05:10;
y = @(t)sigma0*cos(omega*t);
u=@(t)sigma0*(omega^2)*sin(omega*t); 
yp1 =y(t);
[t,x] = ode45(@(t,x)invertedPend(x,g,l,u(t)),t,x0);



% Set the figure size explicitly
f1 = figure('Position', [100, 100, 701, 280]);

hold on
 f = cell(length(t),1); 
% for i=1:length(t)
%     plot_pend(x(i,:),g,l,yp1(i));
%     f{i} = getframe(gcf);
% end
% Open the video writer object
obj = VideoWriter('pend.avi');
obj.Quality = 100;
obj.FrameRate = 30;


open(obj);

% Write the frames to the video file
for i = 1:length(x)
    % Set the figure size again before capturing the frame
    set(gcf, 'Position', [100, 100, 1250, 750]);
    plot_pend(x(i,:), g, l, yp1(i));
    f{i} = getframe(gcf);
    writeVideo(obj, f{i});
end

% Close the video writer object
close(obj);


function dx = invertedPend(x,g,l,u)
dx(1,1) = x(2);
dx(2,1) = -(g/l)*sin(x(1)) - u;
end
function plot_pend(x,m,l,y)
xp = l*sin(x(1));
yp = l*cos(x(1))+y;
mr = 0.2*sqrt(m); 
plot([0,0],[-5 5],'k','LineWidth',1,'LineStyle','--'), hold on
plot([-5 5],[0,0],'k','LineWidth',1,'LineStyle','--'), hold on
plot([0 xp],[y yp],'k','LineWidth',2); % Draw pendulum
rectangle('Position',[xp-mr/2,yp-mr/2,mr,mr],'Curvature',1,'FaceColor',[1 0.1 .1],'LineWidth',1.5);
axis([-5 5 -0.5 0.5]);
axis equal
set(gcf,'Position',[100 100 1000 600])
drawnow
hold off
end
