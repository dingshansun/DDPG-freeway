Reward=Training.EpisodeReward';
N=3900;M=50;
y=nan(N,1);
for i=1:N
    y(i)=mean(Reward(i:i+M));
end
% figure();
hold on;
plot(1:N,y,'LineWidth', 1.5);