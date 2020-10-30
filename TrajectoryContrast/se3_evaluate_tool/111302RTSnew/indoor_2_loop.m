clc;
clear;
close all;
addpath('DATAS','FUNCTIONS');
flag=0;
mintake_off_stamp=0;
minRMS=100;
G1=importdata ('huskyloam1.txt');    % provided by EuRoC
aft_mappep_to_init1=G1.data;
cell_aft_mappep_t1=G1.textdata(2:size(G1.textdata,1),1:1);
aft_mapped_t1=zeros(size(cell_aft_mappep_t1,1),1);
for i=1:size(cell_aft_mappep_t1,1)
    aft_mapped_t1(i,1)=str2double(cell2mat(cell_aft_mappep_t1(i,1)))/1e9;
end
%先把大致的坐标移动到原点,因为这里sim3DataAlignment是一个七参数模型,会估计尺度，不先平移坐标尺度变化会很大
GNGGA1(:,1)=aft_mappep_to_init1(:,1);
GNGGA1(:,2)=aft_mappep_to_init1(:,2);
GNGGA1(:,3)=aft_mappep_to_init1(:,3);

est1=[aft_mapped_t1,GNGGA1];
EST1=importdata ('huskyrts1.csv');
est_data1=EST1.data;
cell_est_t1=EST1.textdata(2:size(EST1.textdata,1),1:1);
est_t1=zeros(size(cell_est_t1,1),1);
for i=1:size(cell_est_t1,1)
    est_t1(i,1)=str2double(cell2mat(cell_est_t1(i,1)));
end

gt1=[est_t1,est_data1];
take_off_stamp1 = est1(1,1)-gt1(1,1);

for take_off_stamp = -take_off_stamp1-2.0:0.1:-take_off_stamp1+2.0
    
    G=importdata ('huskyloam1.txt');    % provided by EuRoC
    aft_mappep_to_init=G.data;
    cell_aft_mappep_t=G.textdata(2:size(G.textdata,1),1:1);
    aft_mapped_t=zeros(size(cell_aft_mappep_t,1),1);
    for i=1:size(cell_aft_mappep_t,1)
        aft_mapped_t(i,1)=str2double(cell2mat(cell_aft_mappep_t(i,1)))/1e9;
    end
    %先把大致的坐标移动到原点,因为这里sim3DataAlignment是一个七参数模型,会估计尺度，不先平移坐标尺度变化会很大
    GNGGA(:,1)=aft_mappep_to_init(:,1);
    GNGGA(:,2)=aft_mappep_to_init(:,2);
    GNGGA(:,3)=aft_mappep_to_init(:,3);
    est=[aft_mapped_t,GNGGA];
    litao=importdata ('huskyrts1.csv');
    litao_data=litao.data;
    cell_est_t=litao.textdata(2:size(litao.textdata,1),1:1);
    litao_t=zeros(size(cell_est_t,1),1);
    for i=1:size(cell_est_t,1)
        litao_t(i,1)=str2double(cell2mat(cell_est_t(i,1)));
    end
    gt=[litao_t,litao_data];
    %% need to adjust the time standard of ground truth
    gt(:,1) = gt(:,1)-take_off_stamp;
    %% extract timestamp and 3d position from 'gt' and 'est', these are all we need for alignment
    time_gt = gt(:,1);
    P_gt = gt(:,2:4)';
    time_es = est(:,1);
    P_es = est(:,2:4)';
    %% alignment
    [Ids_es2, Ids_gt2] = findIds (time_es, time_gt, 0.0035);  %0.0035是可以保证既没有多解也没有少很多数据的值
    if size(Ids_es2,1) < size(Ids_gt2,1)
        Ids_gt2(size(Ids_es2,1)+1:size(Ids_gt2,1),:)=[];
    else
        if size(Ids_es2,1) > size(Ids_gt2,1)
            Ids_es2(size(Ids_gt2,1)+1:size(Ids_es2,1),:)=[];
        end
    end
    Ids_es=Ids_es2;
    Ids_gt=Ids_gt2;
    %[R_es, t_es, s_es] = sim3DataAlignment (P_es(:,Ids_es), P_gt(:,Ids_gt));
    [R_es, t_es] = se3DataAlignment(P_es(:,Ids_es), P_gt(:,Ids_gt));
    s_es=1;
    %% do not miss '/s_es' to maintain the estimated scale
    P_es_aligned = R_es*P_es + repmat(t_es,1,size(P_es,2))/s_es;
    %% draw trajectory
    time_matched = time_es(Ids_es);
    P_es_matched = P_es_aligned(:,Ids_es);
    P_gt_matched = P_gt(:,Ids_gt);
    %% draw error in XYZ axes
    errX = P_es_aligned(1,Ids_es)-P_gt(1,Ids_gt);
    errY = P_es_aligned(2,Ids_es)-P_gt(2,Ids_gt);
    errZ = P_es_aligned(3,Ids_es)-P_gt(3,Ids_gt);
    %% some printing
    errVec = P_es_aligned(1:3,Ids_es)-P_gt(1:3,Ids_gt);
    N = size(errVec,2);
    RMSE_trans = 0;
    for i = 1:N
        RMSE_trans = RMSE_trans+norm(errVec(:,i))^2;
    end
    RMSE_trans = sqrt(RMSE_trans/N);
    if RMSE_trans<minRMS
        minRMS=RMSE_trans;
        mintake_off_stamp=take_off_stamp;
    end
	if RMSE_trans < 0.1
        flag=1;
        errXYZ=sqrt(errX.^2+errY.^2+errZ.^2);
        errXYZ_up=sort(errXYZ);
        errXYZ90=errXYZ_up(floor(0.90*size(errXYZ,2)));
        errXYZ95=errXYZ_up(floor(0.95*size(errXYZ,2)));
        fprintf('abs mean error in [X Y Z error_90 error_95]: [%fm %fm %fm %fm %fm]\n',mean(abs(errX)),mean(abs(errY)),mean(abs(errZ)),errXYZ90,errXYZ95);
        fprintf('RMSE of translation is %fm\n',RMSE_trans);
		figure;
		plot3(P_gt_matched(1,:),P_gt_matched(2,:),P_gt_matched(3,:),'r-');
		hold on;
		plot3(P_es_matched(1,:),P_es_matched(2,:),P_es_matched(3,:),'b-');
		for i = 1:size(P_gt_matched,2)  % draw difference, this block is time consuming
			plot3([P_gt_matched(1,i),P_es_matched(1,i)],...
				[P_gt_matched(2,i),P_es_matched(2,i)],[P_gt_matched(3,i),P_es_matched(3,i)],'y-');
		end
		axis equal;
		grid on;
		lgd = legend('ground truth','Fusion-estimated');
		set(lgd,'Fontname','Times New Roman','FontWeight','bold','FontSize',15);
		title('真值与多源传感器融合导航结果对比','FontSize',15);
		figure;
		subplot(311);
		plot(errX);
		title('position error of axis-X (m)');
		subplot(312);
		plot(errY);
		title('position error of axis-Y (m)');
		subplot(313);
		plot(errZ);
		title('position error of axis-Z (m)');
        
        figure;
        cdfplot(errXYZ);
        title('error CDF figure');
        xlabel('m');
        ylabel('百分比');
        hold on
        
        y1=0.90;
        x1=errXYZ90;
        plot([x1 x1], [0 y1],'--k')
        plot([0 x1], [y1 y1],'--k')
        y2=0.95;
        x2=errXYZ95;
        plot([x2 x2], [0 y2],'--k')
        plot([0 x2], [y2 y2],'--k')
		break;
    end
end

if flag==0
    take_off_stamp=mintake_off_stamp;
    G=importdata ('huskyloam1.txt');    % provided by EuRoC
    aft_mappep_to_init=G.data;
    cell_aft_mappep_t=G.textdata(2:size(G.textdata,1),1:1);
    aft_mapped_t=zeros(size(cell_aft_mappep_t,1),1);
    for i=1:size(cell_aft_mappep_t,1)
        aft_mapped_t(i,1)=str2double(cell2mat(cell_aft_mappep_t(i,1)))/1e9;
    end
    %先把大致的坐标移动到原点,因为这里sim3DataAlignment是一个七参数模型,会估计尺度，不先平移坐标尺度变化会很大
    GNGGA(:,1)=aft_mappep_to_init(:,1);
    GNGGA(:,2)=aft_mappep_to_init(:,2);
    GNGGA(:,3)=aft_mappep_to_init(:,3);
    est=[aft_mapped_t,GNGGA];
    litao=importdata ('huskyrts1.csv');
    litao_data=litao.data;
    cell_est_t=litao.textdata(2:size(litao.textdata,1),1:1);
    litao_t=zeros(size(cell_est_t,1),1);
    for i=1:size(cell_est_t,1)
        litao_t(i,1)=str2double(cell2mat(cell_est_t(i,1)));
    end
    gt=[litao_t,litao_data];
    %% need to adjust the time standard of ground truth
    gt(:,1) = gt(:,1)-take_off_stamp;
    %% extract timestamp and 3d position from 'gt' and 'est', these are all we need for alignment
    time_gt = gt(:,1);
    P_gt = gt(:,2:4)';
    time_es = est(:,1);
    P_es = est(:,2:4)';
    %% alignment
    [Ids_es2, Ids_gt2] = findIds (time_es, time_gt, 0.0035);  %0.0035是可以保证既没有多解也没有少很多数据的值
    if size(Ids_es2,1) < size(Ids_gt2,1)
        Ids_gt2(size(Ids_es2,1)+1:size(Ids_gt2,1),:)=[];
    else
        if size(Ids_es2,1) > size(Ids_gt2,1)
            Ids_es2(size(Ids_gt2,1)+1:size(Ids_es2,1),:)=[];
        end
    end
    Ids_es=Ids_es2;
    Ids_gt=Ids_gt2;
    %[R_es, t_es, s_es] = sim3DataAlignment (P_es(:,Ids_es), P_gt(:,Ids_gt));
    [R_es, t_es] = se3DataAlignment(P_es(:,Ids_es), P_gt(:,Ids_gt));
    s_es=1;
    %% do not miss '/s_es' to maintain the estimated scale
    P_es_aligned = R_es*P_es + repmat(t_es,1,size(P_es,2))/s_es;
    %% draw trajectory
    time_matched = time_es(Ids_es);
    P_es_matched = P_es_aligned(:,Ids_es);
    P_gt_matched = P_gt(:,Ids_gt);
    %% draw error in XYZ axes
    errX = P_es_aligned(1,Ids_es)-P_gt(1,Ids_gt);
    errY = P_es_aligned(2,Ids_es)-P_gt(2,Ids_gt);
    errZ = P_es_aligned(3,Ids_es)-P_gt(3,Ids_gt);
    %% some printing
    errVec = P_es_aligned(1:3,Ids_es)-P_gt(1:3,Ids_gt);
    N = size(errVec,2);
    RMSE_trans = 0;
    for i = 1:N
        RMSE_trans = RMSE_trans+norm(errVec(:,i))^2;
    end
    RMSE_trans = sqrt(RMSE_trans/N);
    errXYZ=sqrt(errX.^2+errY.^2+errZ.^2);
    errXYZ_up=sort(errXYZ);
    errXYZ90=errXYZ_up(floor(0.90*size(errXYZ,2)));
    errXYZ95=errXYZ_up(floor(0.95*size(errXYZ,2)));
    fprintf('abs mean error in [X Y Z error_90 error_95]: [%fm %fm %fm %fm %fm]\n',mean(abs(errX)),mean(abs(errY)),mean(abs(errZ)),errXYZ90,errXYZ95);
    fprintf('RMSE of translation is %fm\n',RMSE_trans);
    figure;
    plot3(P_gt_matched(1,:),P_gt_matched(2,:),P_gt_matched(3,:),'r-');
    hold on;
    plot3(P_es_matched(1,:),P_es_matched(2,:),P_es_matched(3,:),'b-');
    for i = 1:size(P_gt_matched,2)  % draw difference, this block is time consuming
        plot3([P_gt_matched(1,i),P_es_matched(1,i)],...
            [P_gt_matched(2,i),P_es_matched(2,i)],[P_gt_matched(3,i),P_es_matched(3,i)],'y-');
    end
    axis equal;
    grid on;
    lgd = legend('ground truth','FUSION-estimated');
    set(lgd,'Fontname','Times New Roman','FontWeight','bold','FontSize',15);
    title('真值与多源传感器融合导航结果对比','FontSize',15);
    figure;
    subplot(311);
    plot(errX);
    title('position error of axis-X (m)');
    subplot(312);
    plot(errY);
    title('position error of axis-Y (m)');
    subplot(313);
    plot(errZ);
    title('position error of axis-Z (m)');
    figure;
    cdfplot(errXYZ);
    title('error CDF figure');
    xlabel('m');
    ylabel('百分比');
    hold on
    y1=0.90;
    x1=errXYZ90;
    plot([x1 x1], [0 y1],'--k')
    plot([0 x1], [y1 y1],'--k')
    y2=0.95;
    x2=errXYZ95;
    plot([x2 x2], [0 y2],'--k')
    plot([0 x2], [y2 y2],'--k')
end