%% Run z-score
% Data
%y = [1 1 1.1 1 0.9 1 1 1.1 1 0.9 1 1.1 1 1 0.9 1 1 1.1 1 1,...
%    1 1 1.1 0.9 1 1.1 1 1 0.9 1 1.1 1 1 1.1 1 0.8 0.9 1 1.2 0.9 1,...
%    1 1.1 1.2 1 1.5 1 3 2 5 3 2 1 1 1 0.9 1,...
%    1 3 2.6 4 3 3.2 2 1 1 0.8 4 4 2 2.5 1 1 1];

y = xlsread('Sub4_Effort_sqbar_set3.xlsx', 'D130:D879');

% Settings
threshold = 10;

% Get results
signals = StaticAlgo(y,threshold);

figure; subplot(2,1,1); 

hold on;
x = 1:length(y); bounds = threshold.*ones(length(y),1);
plot(x(x), y(1) + bounds,'LineWidth',1,'Color','green','LineWidth',1.5);
plot(x(x), y(1) - bounds,'LineWidth',1,'Color','green','LineWidth',1.5);
plot(x(x), y(1).*ones(length(y),1),'LineWidth',1,'Color','cyan','LineWidth',1.5);
plot(1:length(y),y,'b');
title('Subject 4 Squat Heavy')
xlabel('Time (samples)')
ylabel('Back Curvature (\Delta\theta)')
subplot(2,1,2);
stairs(signals,'r','LineWidth',1.5); ylim([-1.5 1.5]);
ylabel('Feedback Triggered')

hold off

y = xlsread('Sub2_Effort_dead_set3.xlsx', 'D208:D778');
signals = StaticAlgo(y,threshold);

figure; subplot(2,1,1); 

hold on;
x = 1:length(y); bounds = threshold.*ones(length(y),1);
plot(x(x), y(1) + bounds,'LineWidth',1,'Color','green','LineWidth',1.5);
plot(x(x), y(1) - bounds,'LineWidth',1,'Color','green','LineWidth',1.5);
plot(x(x), y(1).*ones(length(y),1),'LineWidth',1,'Color','cyan','LineWidth',1.5);
plot(1:length(y),y,'b');
title('Subject 2 Deadlift Heavy')
xlabel('Time (samples)')
ylabel('Back Curvature (\Delta\theta)')
subplot(2,1,2);
stairs(signals,'r','LineWidth',1.5); ylim([-1.5 1.5]);
ylabel('Feedback Triggered')

hold off

y = xlsread('Sub3_Effort_dead_set3.xlsx', 'D92:D855');
signals = StaticAlgo(y,threshold);

figure; subplot(2,1,1); 

hold on;
x = 1:length(y); bounds = threshold.*ones(length(y),1);
plot(x(x), y(1) + bounds,'LineWidth',1,'Color','green','LineWidth',1.5);
plot(x(x), y(1) - bounds,'LineWidth',1,'Color','green','LineWidth',1.5);
plot(x(x), y(1).*ones(length(y),1),'LineWidth',1,'Color','cyan','LineWidth',1.5);
plot(1:length(y),y,'b');
title('Subject 3 Deadlift Heavy')
xlabel('Time (samples)')
ylabel('Back Curvature (\Delta\theta)')
subplot(2,1,2);
stairs(signals,'r','LineWidth',1.5); ylim([-1.5 1.5]);
ylabel('Feedback Triggered')

hold off

