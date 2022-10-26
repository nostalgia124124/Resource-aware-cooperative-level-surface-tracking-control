clear all;

iter = 42000;

% init PCA directions
V1 = [-1 0 0;
      0 0 -1;
      0 1 0];
V2 = [0 -1 0;
      0 0 -1;
      1 0 0];

% init robots positions 
r1 = [13 51 91.5;
      13 52 90.5;
      13 48.5 91;
      13 50 92]*0.01;
r2 = [47 30 74.5;
      50 30 75.5;
      53 30 75;
      54 30 76]*0.01;
  
Nz = 4;  % number of a robot swarm

% control parameters
k0 = 100;
k1 = 2;
k2 = 100;

tic
[history_Z1,history_P1] = runswarm(r1, iter, Nz,k0,k1,k2,V1);
[history_Z2,history_P2] = runswarm(r2, iter, Nz,k0,k1,k2,V2);
toc



%% plot simulation results

mycolorlist = [[0.98,0.69,0.58];[0.98,0.49,0.38];[0.98,0.29,0.18];[0.98,0.1,0]];
mycolorlist2 = [[0.72,0.65,0.8];[0.52,0.45,0.8];[0.32,0.25,0.8];[0.12,0.05,0.8]];

timelist = 1:iter;
figure();

for i=1:4
        plot3(history_P1(timelist,i,1),history_P1(timelist,i,2),history_P1(timelist,i,3),'-','LineWidth',1,'Color', mycolorlist(i,:));
        hold on;
        plot3(history_P2(timelist,i,1),history_P2(timelist,i,2),history_P2(timelist,i,3),'-.','LineWidth',1,'Color', mycolorlist2(i,:));
        hold on;
end


figure();
Z1 = reshape(history_Z1(1,:,:),[Nz,iter]);
Z2 = reshape(history_Z2(1,:,:),[Nz,iter]);


for i = 1:4
    plot(timelist, Z1(i,:),'-.','LineWidth',1,'Color',mycolorlist(i,:));
    hold on;
    plot(timelist, Z2(i,:),'-','LineWidth',1,'Color',mycolorlist2(i,:));
    hold on;
end
    
xlabel('time(min)');
ylabel('pollutant concentration(ppm)');
