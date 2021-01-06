clc;
clear;
close all;
 uicontrol('Style','pushbutton','String','again', 'FontSize',12, ...
       'Position', [1 1 60 40], 'Callback','dijkstra');
%% set up color map for display 
cmap = [1 1 1; ...%  1 - white - 空地
        0 0 0; ...% 2 - black - 障碍 
        1 0 0; ...% 3 - red - 已搜索过的地方
        0 0 1; ...% 4 - blue - 下次搜索备选中心 
        0 1 0; ...% 5 - green - 起始点
        1 1 0;...% 6 - yellow -  到目标点的路径 
       1 0 1];% 7 - -  目标点 
   
colormap(cmap); 
map1 = zeros(10); 
wallpercent=0.4;
% % 设置障障碍 
map1(1:5, 7) = 2;
map1(8,1:3) = 2; 
map1(2:5,3:5)=2;
%map1(ceil(10^2.*rand(floor(10*10*wallpercent),1))) =2;
%  map(ceil(10.*rand),ceil(10.*rand)) = 5; % 起始点
%map(ceil(10.*rand),ceil(10.*rand)) = 6; % 目标点
% %% 建立地图
nrows = 10; 
ncols = 10;  
start_node = sub2ind(size(map1), 10,1); %sub2ind是把矩阵式索引以线性索引的形式给出
dest_node = sub2ind(size(map1),1,4); %终点
map1(dest_node) = 7;
TraverseNum = 0;
forNum = 0;
% % 距离数组初始化
distanceFromStart = Inf(nrows,ncols);  
distanceFromStart(start_node) = 0; 
distanceFromgoal = Inf(nrows,ncols);  
distanceFromgoal(dest_node) = 0; 
% % 对于每个格单元，这个数组保存其父节点的索引。 
parent = zeros(nrows,ncols); 
% % 主循环
writerObj = VideoWriter('Dijkstra1.avi');
open(writerObj);
% tic %启动秒表计时器
 while true 
  % 画出现状图
  map1(start_node) = 5;
  map1(dest_node) = 7;
  image(1.5, 1.5, map1); %绘制网格，image(x,y,C),如果x,y是一个数，表示起始坐标（坐标间距为1）；
                          %如果为一个二维数组，表示起止坐标，间距自适应计算
                            %如果为一个多维数组，用起止的元素数，表示起止坐标，间距自适应计算
  grid on; %显示方格线
  axis image; 
  %   drawnow limitrate;
    drawnow;
   % 找到距离起始点最近的节点
  [min_dist, current] = min(distanceFromStart(:)); %返回当前距离数组(距离起点）的最小值和最小值的位置索引。
  distanceFromStart
  min_dist
%   tic;
%     for t = 0.001:1.5
%         while toc < t
%         end
%     end
  %[min_dist1, current1] = min(distanceFromgoal(:)); %返回当前距离数组（距离目标点）的最小值和最小值的位置索引。
   if ((current == dest_node) || isinf(min_dist)) %搜索到目标点或者全部搜索完，结束循环。
         break; 
   end; 
% if((current==current1)|| isinf(min_dist)|| isinf(min_dist1))
%       break;
% end      
 map1(current) = 3; %将当前颜色标为红色。
 image(1.5, 1.5, map1);
  grid on; %显示方格线
  axis image;
 drawnow;
 %map1(current1)=3;
distanceFromStart(current) = Inf;  %当前区域在距离数组中设置为无穷，表示已搜索。
%distanceFromgoal(current1) = Inf;  %当前区域在距离数组中设置为无穷，表示已搜索。
 [i, j] = ind2sub(size(distanceFromStart), current); %返回当前位置的坐标
 %[i1, j1] = ind2sub(size(distanceFromgoal), current1); %返回当前位置的坐标
 neighbor = [ 
            i-1,j;... 确定当前位置的上下左右区域
            i+1,j;...确定当前位置的上下左右区域 
            i,j+1;... 确定当前位置的上下左右区域
             i,j-1]; %确定当前位置的上下左右区域。
% neighbor2 = [ 
%             i1-1,j1;... 
%             i1+1,j1;... 
%             i1,j1+1;... 
%              i1,j1-1]; %确定当前位置的上下左右区域。        
      neighbor1 = [ 
              i-1,j-1;...
              i+1,j+1;...
              i-1,j+1;...
              i+1,j-1]; %确定当前位置的对角区域。      
 outRangetest = (neighbor(:,1)<1) + (neighbor(:,1)>nrows) +...
                    (neighbor(:,2)<1) + (neighbor(:,2)>ncols ); %判断下一次搜索的区域是否超出限制。   
  outRangetest1 = (neighbor1(:,1)<1) + (neighbor1(:,1)>nrows) +...
                     (neighbor1(:,2)<1) + (neighbor1(:,2)>ncols ); %判断下一次搜索的区域是否超出限制。        
% outRangetest2 = (neighbor2(:,1)<1) + (neighbor2(:,1)>nrows) +...
%                    (neighbor2(:,2)<1) + (neighbor2(:,2)>ncols ); %判断下一次搜索的区域是否超出限制。
 locate = find(outRangetest>0); %返回超限点的行数。
   locate1 = find(outRangetest1>0); %返回超限点的行数。
 %locate2 = find(outRangetest2>0); %返回超限点的行数。
 neighbor(locate,:)=[]; %在下一次搜索区域里去掉超限点，删除某一行。
 neighborIndex = sub2ind(size(map1),neighbor(:,1),neighbor(:,2)); %返回下次搜索区域的索引号。
   neighbor1(locate1,:)=[]; %在下一次搜索区域里去掉超限点，删除某一行。
    neighborIndex1 = sub2ind(size(map1),neighbor1(:,1),neighbor1(:,2)); %返回下次搜索区域的索引号。
%     neighbor2(locate2,:)=[]; %在下一次搜索区域里去掉超限点，删除某一行。
%     neighborIndex2 = sub2ind(size(map1),neighbor2(:,1),neighbor2(:,2)); %返回下次搜索区域的索引号。

forNum = forNum + 1;
 for i=1:length(neighborIndex) 
 if (map1(neighborIndex(i))~=2) && (map1(neighborIndex(i))~=3 && map1(neighborIndex(i))~= 5)  %注意终点不在这个判断范围内，这样终点上是有路径索引信息的
     map1(neighborIndex(i)) = 4; %如果下次搜索的点不是障碍，不是起点，没有搜索过就标为蓝色。
     if((neighborIndex(i)+1==current)||(neighborIndex(i)-1==current))% || ...%判断是否在current的上方临格或下方临格（沿列方向），这是原作的代码,min_dist=23
       %       (neighborIndex(i)+nrows==current)||(neighborIndex(i)-nrows==current))%改进：如果增加在current的水平左右侧方的临格判断，那么最终的for循环会增加3次（为73次），不清楚为何如此设计，但是求得的min_dist=16会正确
        if distanceFromStart(neighborIndex(i))> min_dist + 1  %distanceFromStart是一个无穷大矩阵，在未遍历到时，肯定是大于min_dist+2的，所以这一步是用来判断是否被索引过
            TraverseNum = TraverseNum + 1;
            distanceFromStart(neighborIndex(i)) = min_dist + 1; %这行是整个代码的核心变量赋值，这个distanceFromStart就是进行路径索引规划时的“心中的谱”，具体赋的值就是对每一格(步)评估
             parent(neighborIndex(i)) = current; %如果在距离数组里，在parent里current临近的某点上(neighborIndex(i))，
                                                %记录current的线性索引值，所以parent是一个类似链表一样的存在
                                                %如果是起始点，则不会记录current在parent内，故在后续的生成rout上，就可以判断~=0了
                                               

        end 
     else
         if distanceFromStart(neighborIndex(i))> min_dist + 2  
          distanceFromStart(neighborIndex(i)) = min_dist + 2; %mind_dist+2区别于下面的mind_dist+1,表示的是对角线的几个点？
          TraverseNum = TraverseNum + 1;  
          parent(neighborIndex(i)) = current;%如果在距离数组里，在parent里current临近的点上(neighborIndex(i))，
                                                %记录current的线性索引值，所以parent是一个类似链表一样的存在
                                                %如果是起始点，则不会记录current在parent内，故在后续的生成rout上，就可以判断~=0了 
         end
     end
  end 
 end 
%  for i=1:length(neighborIndex2) 
%  if (map1(neighborIndex2(i))~=2) && (map1(neighborIndex2(i))~=3 && map1(neighborIndex2(i))~= 5) 
%      map1(neighborIndex2(i)) = 4; %如果下次搜索的点不是障碍，不是起点，没有搜索过就标为蓝色。
%    if distanceFromgoal(neighborIndex2(i))> min_dist1 + 1  
%        distanceFromgoal(neighborIndex2(i)) = min_dist1+1; 
%          parent(neighborIndex2(i)) = current1; %如果在距离数组里，。       
%    end 
%   end 
%  end 
%pause(1);
 end
%  if(map1(neighborIndex)~= 2)
%  %if(map1(neighborIndex1)~= 2 )
%   for i=1:length(neighborIndex1) 
%    if (map1(neighborIndex1(i))~=2) && (map1(neighborIndex1(i))~=3 && map1(neighborIndex1(i))~= 5 )
%      map1(neighborIndex1(i)) = 4; %如果下次搜索的点不是障碍，不是起点，没有搜索过就标为蓝色。
%     if distanceFromStart(neighborIndex1(i))> min_dist + sqrt(2) 
%        distanceFromStart(neighborIndex1(i)) = min_dist+ sqrt(2); 
%          parent(neighborIndex1(i)) = current; %如果在距离数组里，。
%     end
%    end
%   end
%  end
%  end
   frame = getframe;
    writeVideo(writerObj,frame);
 %
if (isinf(distanceFromStart(dest_node))) 
    route = [];
else
    %提取路线坐标
  route =dest_node ;%从终点反向查找至启动，并提取
  while (parent(route(1)) ~= 0) %parent里填写的是current的索引，如果踩过的话，一定不为0，通过这种方式判断是否已经搜索到这了
         route = [parent(route(1)), route]; 
         for k = 2:length(route) - 1 
               map1(route(k)) = 6; 
               image(1.5, 1.5, map1);
              grid on; 
              axis image; 
               frame = getframe;
         end
   end 
%  动态显示出路线 
        for k = 2:length(route) - 1 
              map1(route(k)) = 6; 
               image(1.5, 1.5, map1);
              grid on; 
              axis image; 
               frame = getframe;
    writeVideo(writerObj,frame);
        end  
         b=min_dist
end        
close(writerObj);
title('基于{ \color{red}Dijkstra} 算法的路径规划 ','fontsize',16)
% toc