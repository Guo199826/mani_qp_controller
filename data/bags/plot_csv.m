% Guo Yu
% Plot results from csv file
% Please change the base's reference frame in FrankaEmikaPandaRobot.m!
% (try to combine several loops together
addpath(genpath('..\..\..\dqrobotics-toolbox-matlab'));
position_guid = readmatrix('joint_position_guid.csv'); 
position_real = readmatrix('joint_position_real.csv'); 
velocity_guid = readmatrix('joint_velocity_guid.csv'); 
velocity_real = readmatrix('joint_velocity_real.csv'); 

% remove time stamp and other useless columns
position_guid = position_guid(:,5:end);
position_real = position_real(:,5:end);
velocity_guid = velocity_guid(:,5:end);
velocity_real = velocity_real(:,5:end);
[num_rows, num_columns] = size(position_guid);
[num_rows_real, num_columns_real] = size(position_real);
[num_rows_v, num_columns_v] = size(velocity_guid);
[num_rows_v_real, num_columns_v_real] = size(velocity_real);
dt = 0.01;
linewidth_g=1;
linewidth_r=1;

close all;
% Plot joint position %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
hold on;
% guidance
plot([1:num_rows].*dt, position_guid(:,1), '--','color','r','Linewidth',linewidth_g);
plot([1:num_rows].*dt, position_guid(:,2), '--','color','g','Linewidth',linewidth_g);
plot([1:num_rows].*dt, position_guid(:,3), '--','color','b','Linewidth',linewidth_g);
plot([1:num_rows].*dt, position_guid(:,4), '--','color','c','Linewidth',linewidth_g);
plot([1:num_rows].*dt, position_guid(:,5), '--','color','m','Linewidth',linewidth_g);
plot([1:num_rows].*dt, position_guid(:,6), '--','color','y','Linewidth',linewidth_g);
plot([1:num_rows].*dt, position_guid(:,7), '--','color','k','Linewidth',linewidth_g);
% real traj
plot([1:num_rows_real].*dt, position_real(:,1), '-','color','r','Linewidth',linewidth_r);
plot([1:num_rows_real].*dt, position_real(:,2), '-','color','g','Linewidth',linewidth_r);
plot([1:num_rows_real].*dt, position_real(:,3), '-','color','b','Linewidth',linewidth_r);
plot([1:num_rows_real].*dt, position_real(:,4), '-','color','c','Linewidth',linewidth_r);
plot([1:num_rows_real].*dt, position_real(:,5), '-','color','m','Linewidth',linewidth_r);
plot([1:num_rows_real].*dt, position_real(:,6), '-','color','y','Linewidth',linewidth_r);
plot([1:num_rows_real].*dt, position_real(:,7), '-','color','k','Linewidth',linewidth_r);
set(gca,'fontsize',14);
xlim([0 num_rows*dt])
xlabel('$t$','fontsize',22,'Interpreter','latex');
ylabel('$q$','fontsize',22,'Interpreter','latex');
legend('q_d_1','q_d_2','q_d_3','q_d_4','q_d_5','q_d_6','q_d_7','q_1','q_2','q_3','q_4','q_5','q_6','q_7');
xlabel('Time (dt=0.01s)');
ylabel(['Joint Position']);
title(['Plot of joint position during guidance']);
grid on;

% Plot joint velocity %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
velocity_guid_ = diff(position_guid,1,1)/dt;
velocity_real_ = diff(position_real,1,1)/dt;
% figure
% hold on;
% % guidance
% % plot([1:num_rows-1].*dt, velocity_guid_(:,1), '--','color','r','Linewidth',linewidth_g);
% % plot([1:num_rows-1].*dt, velocity_guid_(:,2), '--','color','g','Linewidth',linewidth_g);
% % plot([1:num_rows-1].*dt, velocity_guid_(:,3), '--','color','b','Linewidth',linewidth_g);
% % plot([1:num_rows-1].*dt, velocity_guid_(:,4), '--','color','c','Linewidth',linewidth_g);
% % plot([1:num_rows-1].*dt, velocity_guid_(:,5), '--','color','m','Linewidth',linewidth_g);
% % plot([1:num_rows-1].*dt, velocity_guid_(:,6), '--','color','y','Linewidth',linewidth_g);
% % plot([1:num_rows-1].*dt, velocity_guid_(:,7), '--','color','k','Linewidth',linewidth_g);
% % real traj
% plot([1:num_rows_real-1].*dt, velocity_real_(:,1), '-','color','r','Linewidth',linewidth_r);
% plot([1:num_rows_real-1].*dt, velocity_real_(:,2), '-','color','g','Linewidth',linewidth_r);
% plot([1:num_rows_real-1].*dt, velocity_real_(:,3), '-','color','b','Linewidth',linewidth_r);
% plot([1:num_rows_real-1].*dt, velocity_real_(:,4), '-','color','c','Linewidth',linewidth_r);
% plot([1:num_rows_real-1].*dt, velocity_real_(:,5), '-','color','m','Linewidth',linewidth_r);
% plot([1:num_rows_real-1].*dt, velocity_real_(:,6), '-','color','y','Linewidth',linewidth_r);
% plot([1:num_rows_real-1].*dt, velocity_real_(:,7), '-','color','k','Linewidth',linewidth_r);
% set(gca,'fontsize',14);
% xlim([0 num_rows*dt])
% xlabel('$t$','fontsize',22,'Interpreter','latex');
% ylabel('$q$','fontsize',22,'Interpreter','latex');
% legend('dq_1','dq_2','dq_3','dq_4','dq_5','dq_6','dq_7');
% xlabel('Time (dt=0.01s)');
% ylabel(['Joint velocity']);
% grid on;
% title(['Joint velocity calculated from position']);

figure
hold on;
% guidance
plot([1:num_rows_v].*dt, velocity_guid(:,1), '--','color','r','Linewidth',linewidth_g);
plot([1:num_rows_v].*dt, velocity_guid(:,2), '--','color','g','Linewidth',linewidth_g);
plot([1:num_rows_v].*dt, velocity_guid(:,3), '--','color','b','Linewidth',linewidth_g);
plot([1:num_rows_v].*dt, velocity_guid(:,4), '--','color','c','Linewidth',linewidth_g);
plot([1:num_rows_v].*dt, velocity_guid(:,5), '--','color','m','Linewidth',linewidth_g);
plot([1:num_rows_v].*dt, velocity_guid(:,6), '--','color','y','Linewidth',linewidth_g);
plot([1:num_rows_v].*dt, velocity_guid(:,7), '--','color','k','Linewidth',linewidth_g);
% real traj
plot([1:num_rows_v_real].*dt, velocity_real(:,1), '-','color','r','Linewidth',linewidth_r);
plot([1:num_rows_v_real].*dt, velocity_real(:,2), '-','color','g','Linewidth',linewidth_r);
plot([1:num_rows_v_real].*dt, velocity_real(:,3), '-','color','b','Linewidth',linewidth_r);
plot([1:num_rows_v_real].*dt, velocity_real(:,4), '-','color','c','Linewidth',linewidth_r);
plot([1:num_rows_v_real].*dt, velocity_real(:,5), '-','color','m','Linewidth',linewidth_r);
plot([1:num_rows_v_real].*dt, velocity_real(:,6), '-','color','y','Linewidth',linewidth_r);
plot([1:num_rows_v_real].*dt, velocity_real(:,7), '-','color','k','Linewidth',linewidth_r);
set(gca,'fontsize',14);
xlim([0 num_rows*dt])
xlabel('$t$','fontsize',22,'Interpreter','latex');
ylabel('$q$','fontsize',22,'Interpreter','latex');
legend('dq_1','dq_2','dq_3','dq_4','dq_5','dq_6','dq_7');
xlabel('Time (dt=0.01s)');
ylabel(['Joint velocity']);
title(['Plot of joint velocity during guidance']);
grid on;

% Plot joint acceleration %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
acc_guid_ = diff(velocity_guid_,1,1)/dt;
acc_real_ = diff(velocity_real_,1,1)/dt;
acc_guid = diff(velocity_guid,1,1)/dt;
acc_real = diff(velocity_real,1,1)/dt;
% figure
% hold on;
% % guidance
% plot([1:num_rows-2].*dt, acc_guid_(:,1), '--','color','r','Linewidth',linewidth_g);
% plot([1:num_rows-2].*dt, acc_guid_(:,2), '--','color','g','Linewidth',linewidth_g);
% plot([1:num_rows-2].*dt, acc_guid_(:,3), '--','color','b','Linewidth',linewidth_g);
% plot([1:num_rows-2].*dt, acc_guid_(:,4), '--','color','c','Linewidth',linewidth_g);
% plot([1:num_rows-2].*dt, acc_guid_(:,5), '--','color','m','Linewidth',linewidth_g);
% plot([1:num_rows-2].*dt, acc_guid_(:,6), '--','color','y','Linewidth',linewidth_g);
% plot([1:num_rows-2].*dt, acc_guid_(:,7), '--','color','k','Linewidth',linewidth_g);
% % real traj
% plot([1:num_rows_real-2].*dt, acc_real_(:,1), '-','color','r','Linewidth',linewidth_r);
% plot([1:num_rows_real-2].*dt, acc_real_(:,2), '-','color','g','Linewidth',linewidth_r);
% plot([1:num_rows_real-2].*dt, acc_real_(:,3), '-','color','b','Linewidth',linewidth_r);
% plot([1:num_rows_real-2].*dt, acc_real_(:,4), '-','color','c','Linewidth',linewidth_r);
% plot([1:num_rows_real-2].*dt, acc_real_(:,5), '-','color','m','Linewidth',linewidth_r);
% plot([1:num_rows_real-2].*dt, acc_real_(:,6), '-','color','y','Linewidth',linewidth_r);
% plot([1:num_rows_real-2].*dt, acc_real_(:,7), '-','color','k','Linewidth',linewidth_r);
% set(gca,'fontsize',14);
% xlim([0 num_rows*dt])
% xlabel('$t$','fontsize',22,'Interpreter','latex');
% ylabel('$q$','fontsize',22,'Interpreter','latex');
% legend('ddq_1','ddq_2','ddq_3','ddq_4','ddq_5','ddq_6','ddq_7');
% xlabel('Time (dt=0.01s)');
% ylabel(['Joint acceleration']);
% grid on;
% title(['Joint acceleration calculated from position']);

figure
hold on;
% guidance
plot([1:num_rows_v-1].*dt, acc_guid(:,1), '--','color','r','Linewidth',linewidth_g);
plot([1:num_rows_v-1].*dt, acc_guid(:,2), '--','color','g','Linewidth',linewidth_g);
plot([1:num_rows_v-1].*dt, acc_guid(:,3), '--','color','b','Linewidth',linewidth_g);
plot([1:num_rows_v-1].*dt, acc_guid(:,4), '--','color','c','Linewidth',linewidth_g);
plot([1:num_rows_v-1].*dt, acc_guid(:,5), '--','color','m','Linewidth',linewidth_g);
plot([1:num_rows_v-1].*dt, acc_guid(:,6), '--','color','y','Linewidth',linewidth_g);
plot([1:num_rows_v-1].*dt, acc_guid(:,7), '--','color','k','Linewidth',linewidth_g);
% real traj
plot([1:num_rows_v_real-1].*dt, acc_real(:,1), '-','color','r','Linewidth',linewidth_r);
plot([1:num_rows_v_real-1].*dt, acc_real(:,2), '-','color','g','Linewidth',linewidth_r);
plot([1:num_rows_v_real-1].*dt, acc_real(:,3), '-','color','b','Linewidth',linewidth_r);
plot([1:num_rows_v_real-1].*dt, acc_real(:,4), '-','color','c','Linewidth',linewidth_r);
plot([1:num_rows_v_real-1].*dt, acc_real(:,5), '-','color','m','Linewidth',linewidth_r);
plot([1:num_rows_v_real-1].*dt, acc_real(:,6), '-','color','y','Linewidth',linewidth_r);
plot([1:num_rows_v_real-1].*dt, acc_real(:,7), '-','color','k','Linewidth',linewidth_r);
set(gca,'fontsize',14);
xlim([0 num_rows*dt])
xlabel('$t$','fontsize',22,'Interpreter','latex');
ylabel('$q$','fontsize',22,'Interpreter','latex');
legend('ddq_1','ddq_2','ddq_3','ddq_4','ddq_5','ddq_6','ddq_7');
xlabel('Time (dt=0.01s)');
ylabel(['Joint acceleration']);
title(['Plot of joint acceleration during guidance']);
grid on;

% Plot cartesian position%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robot = FrankaEmikaPandaRobot.kinematics();
xt_traj_guid = zeros(3,num_rows);
xt_traj_real = zeros(3,num_rows_real);
for row = 1:num_rows
    % forward kinematic
    xt = robot.fkm(position_guid(row,:));
    xt_tran = vec3(translation(xt));
    xt_traj_guid(:,row) = xt_tran;
end
for row_real = 1:num_rows_real
    % forward kinematic
    xt_real = robot.fkm(position_real(row_real,:));
    xt_tran_real = vec3(translation(xt_real));
    xt_traj_real(:,row_real) = xt_tran_real;
end

figure
hold on;
% guidance
plot([1:num_rows].*dt, xt_traj_guid(1,:), '--','color','r','Linewidth',linewidth_g);
plot([1:num_rows].*dt, xt_traj_guid(2,:), '--','color','g','Linewidth',linewidth_g);
plot([1:num_rows].*dt, xt_traj_guid(3,:), '--','color','b','Linewidth',linewidth_g);
% real traj
plot([1:num_rows_real].*dt, xt_traj_real(1,:), '-','color','r','Linewidth',linewidth_r);
plot([1:num_rows_real].*dt, xt_traj_real(2,:), '-','color','g','Linewidth',linewidth_r);
plot([1:num_rows_real].*dt, xt_traj_real(3,:), '-','color','b','Linewidth',linewidth_r);
set(gca,'fontsize',14);
xlim([0 num_rows*dt])
xlabel('$t$','fontsize',22,'Interpreter','latex');
ylabel('$q$','fontsize',22,'Interpreter','latex');
legend('x','y','z');
xlabel('Time (dt=0.01s)');
ylabel(['Cartesian Position']);
title(['Plot of cartesian position during guidance']);
grid on;

% Plot distance between current and desired ME %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
d = zeros(num_rows_real,1);
ev_real = zeros(6,num_rows_real);
ev_guid = zeros(6,num_rows_real);
for row_real = 1:num_rows_real
    % ME: real traj
    Jt_geom = geomJ(robot,position_real(row_real,:));
    Me_ct = Jt_geom * Jt_geom';
    % singular value of Jacobian
    ev_real(:,row_real) = svd(Jt_geom);

    % ME: from guidance
    Jt_geom_d = geomJ(robot,position_guid(row_real,:));
    Me_d = Jt_geom_d * Jt_geom_d'; 
    % singular value of Jacobian
    ev_guid(:,row_real) = svd(Jt_geom_d);
    % distance between real and desired
    d(row_real,1) = norm(logm(Me_d^-.5*Me_ct*Me_d^-.5),'fro');
end
figure
hold on;
% distance
plot([1:num_rows_real].*dt, d(:,1), '-','color','r','Linewidth',linewidth_r);
set(gca,'fontsize',14);
xlim([0 num_rows*dt])
xlabel('$t$','fontsize',22,'Interpreter','latex');
ylabel('$q$','fontsize',22,'Interpreter','latex');
legend('x','y','z');
xlabel('Time (dt=0.01s)');
ylabel(['Distance d']);
title(['Plot of distance between real and desired ME']);
grid on;

% Plot singular value of Jacobian %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (check if min. sing value satisfied) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
hold on;
% guidance
plot([1:num_rows_real].*dt, ev_guid(1,:), '--','color','r','Linewidth',linewidth_r);
plot([1:num_rows_real].*dt, ev_guid(2,:), '--','color','g','Linewidth',linewidth_r);
plot([1:num_rows_real].*dt, ev_guid(3,:), '--','color','b','Linewidth',linewidth_r);
plot([1:num_rows_real].*dt, ev_guid(4,:), '--','color','c','Linewidth',linewidth_r);
plot([1:num_rows_real].*dt, ev_guid(5,:), '--','color','m','Linewidth',linewidth_r);
plot([1:num_rows_real].*dt, ev_guid(6,:), '--','color','k','Linewidth',linewidth_r);
% real
plot([1:num_rows_real].*dt, ev_real(1,:), '-','color','r','Linewidth',linewidth_r);
plot([1:num_rows_real].*dt, ev_real(2,:), '-','color','g','Linewidth',linewidth_r);
plot([1:num_rows_real].*dt, ev_real(3,:), '-','color','b','Linewidth',linewidth_r);
plot([1:num_rows_real].*dt, ev_real(4,:), '-','color','c','Linewidth',linewidth_r);
plot([1:num_rows_real].*dt, ev_real(5,:), '-','color','m','Linewidth',linewidth_r);
plot([1:num_rows_real].*dt, ev_real(6,:), '-','color','k','Linewidth',linewidth_r);
set(gca,'fontsize',14);
xlim([0 num_rows*dt])
xlabel('$t$','fontsize',22,'Interpreter','latex');
ylabel('$q$','fontsize',22,'Interpreter','latex');
legend('ev_1','ev_2','ev_3','ev_4','ev_5','ev_6');
xlabel('Time (dt=0.01s)');
ylabel(['Eigenvalue']);
title(['Plot Eigenvalue of ME']);
grid on;


