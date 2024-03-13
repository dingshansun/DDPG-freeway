clear; 
% clc
% close all
%% Initial state
x=[zeros(22,1);0];
u=[200,200,1];
scenario=4;
for i=1:60
    x=Freeway_model_Noise(x,u,scenario);
end
xx=[];
u=[100,100,1];
% u=[100*ones(2,900);u_ramp];
% u=repelem(U,1,6);
for i=1:900
    x=Freeway_model_Noise(x,u,scenario);
    xx=[xx x];
end
rou_11=xx(1,:);
v_11=xx(2,:);
q_11=xx(3,:);
rou_12=xx(4,:);
v_12=xx(5,:);
q_12=xx(6,:);
rou_13=xx(7,:);
v_13=xx(8,:);
q_13=xx(9,:);
rou_14=xx(10,:);
v_14=xx(11,:);
q_14=xx(12,:);
q_o1=xx(13,:);
w_o1=xx(14,:);
q_o2=xx(15,:);
w_o2=xx(16,:);
rou_21=xx(17,:);
v_21=xx(18,:);
q_21=xx(19,:);
rou_22=xx(20,:);
v_22=xx(21,:);
q_22=xx(22,:);
TTS=10/3600.*((rou_11+rou_12+rou_13+rou_14+rou_21+rou_22).*1000./1000.*2+w_o1+w_o2);
%%
fprintf('TTS is %.3f veh*h \n', sum(TTS))
figure();
t=1/360:1/360:2.5;
subplot(3,2,1)
plot(t, v_11, '-', 'linewidth', 1.0);
hold on;
plot(t, v_12, '--', 'linewidth', 1.0);
hold on;
plot(t, v_13, ':', 'linewidth', 1.0);
hold on;
plot(t, v_14, '-.', 'linewidth', 1.0);
hold on;
plot(t, v_21, '-o', 'MarkerIndices',1:30:length(v_21), 'linewidth', 1.0);
hold on;
plot(t, v_22, '-x',  'MarkerIndices',1:30:length(v_22), 'linewidth', 1.0);
legend('Segmeng 1-1','Segmeng 1-2','Segmeng 1-3','Segmeng 1-4','Segmeng 2-1','Segmeng 2-2')
xlabel('Time [h]');
ylabel('Speed [km/h]')

subplot(3,2,2)
plot(t, q_11, '-', 'linewidth', 1.0);
hold on;
plot(t, q_12, '--', 'linewidth', 1.0);
hold on;
plot(t, q_13, ':', 'linewidth', 1.0);
hold on;
plot(t, q_14, '-.', 'linewidth', 1.0);
hold on;
plot(t, q_21, '-o', 'MarkerIndices',1:30:length(q_21), 'linewidth', 1.0);
hold on;
plot(t, q_22, '-x',  'MarkerIndices',1:30:length(q_22), 'linewidth', 1.0);
% legend('Segmeng 1-1','Segmeng 1-2','Segmeng 1-3','Segmeng 1-4','Segmeng 2-1','Segmeng 2-2')
xlabel('Time [h]');
ylabel('Flow [veh/h]')

subplot(3,2,3)
plot(t, rou_11, '-', 'linewidth', 1.0);
hold on;
plot(t, rou_12, '--', 'linewidth', 1.0);
hold on;
plot(t, rou_13, ':', 'linewidth', 1.0);
hold on;
plot(t, rou_14, '-.', 'linewidth', 1.0);
hold on;
plot(t, rou_21, '-o', 'MarkerIndices',1:30:length(rou_21), 'linewidth', 1.0);
hold on;
plot(t, rou_22, '-x',  'MarkerIndices',1:30:length(rou_22), 'linewidth', 1.0);
% legend('Segmeng 1-1','Segmeng 1-2','Segmeng 1-3','Segmeng 1-4','Segmeng 2-1','Segmeng 2-2')
xlabel('Time [h]');
ylabel('Density [veh/km]')

subplot(3,2,4)
plot(t, demando1(61:960,scenario), '-', 'linewidth', 1.0);
hold on;
plot(t, demando2(61:960,scenario), '--', 'linewidth', 1.0);
xlabel('Time [h]');
ylabel('Demand [veh/h]');
legend('O_1','O_2')
ylim([0 4000])

subplot(3,2,5)
plot(t, q_o1, '-', 'linewidth', 1.0);
hold on;
plot(t, q_o2, '--', 'linewidth', 1.0);
legend('O_1','O_2')
xlabel('Time [h]');
ylabel('Original flow [veh/h]')
% ylim([0 4000])

subplot(3,2,6)
plot(t,w_o1,'-', 'linewidth', 1.0);
hold on;
plot(t,w_o2,'--', 'linewidth', 1.0);
legend('O_1','O_2')
xlabel('Time [h]');
ylabel('Queue length [veh]')

% figure()
% plot(t,u_ramp,'-','linewidth',1.0);
% ylim([0 1]);
% xlabel('Time [h]');
% ylabel('Ramp metering')

figure()
plot(t, Noise_o1(61:960,scenario), '-', 'linewidth', 1.0);
hold on;
plot(t, Noise_o2(61:960,scenario), '--', 'linewidth', 1.0);
xlabel('Time [h]');
ylabel('Demand [veh/h]');
legend('O_1','O_2')
% ylim([0 4000])
title(['Scenario ' num2str(scenario) ' real demand'])

figure()
plot(t, demando1(61:960,scenario), '-', 'linewidth', 1.0);
hold on;
plot(t, demando2(61:960,scenario), '--', 'linewidth', 1.0);
xlabel('Time [h]');
ylabel('Demand [veh/h]');
legend('O_1','O_2')
ylim([0 4000])
title(['Scenario ' num2str(scenario) ' predicted demand'])