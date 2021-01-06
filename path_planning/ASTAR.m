clc;
clear;
close all;
%% ��ʼ������
%load maze.mat map
%field=map;
 n = 10;   % field size n x n tiles  20*20�Ľ���
wallpercent = 0.15;  % this percent of field is walls   15%�Ľ�����Ϊ�谭�ǽ��
cmap = [1 1 1; ...%  1 - white - �յ�
        0 0 0; ...% 2 - black - �ϰ� 
        1 0 0; ...% 3 - red - ���������ĵط�
        0 0 1; ...% 4 - blue - �´�������ѡ���� 
        0 1 0; ...% 5 - green - ��ʼ��
        1 1 0;...% 6 - yellow -  ��Ŀ ����·�� 
       1 0 1];% 7 - -  Ŀ��� 
colormap(cmap); 
% nrows = 10; 
 %ncols = 10; 
 field = ones(n);
 grid on;
 set(gca,'YDir','normal');   
 hold on;
startposind =10;   %sub2ind��������������ת��Ϊ�������꣬�����Ǳ�Ҫ�ģ���Ϊ�����startposind���ó�[x,y]����ʽ������field([x,y])��ʱ��
goalposind =31;    %�������Ƿ���x��y��Ԫ�أ����Ƿ�����������Ϊx��y������Ԫ��
% field(ceil(n^2.*rand(floor(n*n*wallpercent),1) )) = Inf;
field(1:5, 7) = Inf;
field(8,1:3) = Inf; 
field(2:5,3:5)=Inf;
 %   field(8,10)=Inf;
% startposind = sub2ind([n,n],ceil(n.*rand),ceil(n.*rand));   %sub2ind��������������ת��Ϊ�������꣬�����Ǳ�Ҫ�ģ���Ϊ�����startposind���ó�[x,y]����ʽ������field([x,y])��ʱ��
%goalposind = sub2ind([n,n],ceil(n.*rand),ceil(n.*rand));    %�������Ƿ���x��y��Ԫ�أ����Ƿ�����������Ϊx��y������Ԫ��
field(startposind )=5;
field(goalposind )=7;
% put not a numbers (NaN) in cost chart so A* knows where to look
costchart = NaN*ones(n);      %costchart�����洢�������ʵ�ʴ��ۣ�NaN���������ݣ�����ȷ�Ĳ�����
% set the cost at the starting position to be 0
costchart(startposind) = 0;     %����ʵ�ʴ���
% make fieldpointers as a cell array  ����n*n��Ԫ��
fieldpointers = cell(n);      %fieldpointers�����洢���������Դ����
% set the start pointer to be "S" for start, "G" for goal   �������Ϊ"S",�յ�����Ϊ"G"
fieldpointers{startposind} = 'S'; fieldpointers{goalposind} = 'G';
% everywhere there is a wall, put a 0 so it is not considered   ǽ����Ϊ0
fieldpointers(field == Inf) = {0};      %�ܺõķ�ʽ��field == Inf ����ǽ��λ�ã�fieldpointers(field == Inf)������Ӧ��λ��
   field(field == Inf)=2;
 image(1.5,1.5, field); 
 grid on; 
 axis image; 
setOpen = (startposind); setOpenCosts = (0); setOpenHeuristics = (Inf);
setClosed = []; setClosedCosts = [];%��ʼ������open���close��
setOpen1 = (goalposind); setOpenCosts1 = (0); setOpenHeuristics1 = (Inf);
setClosed1 = []; setClosedCosts1 = [];%��ʼ��Ŀ����open���close��
movementdirections = {'L','R','U','D'};
%movementdirections1 = {'LU','RU','LD','RD'};
% keep track of the number of iterations to exit gracefully if no solution
counterIterations = 1;
%% create figure so we can witness the magic
 % n = length(field);
     % plot goal as a yellow square, and start as a green
     % circle��Ŀ��㻭����ɫ����ʼ��Ϊ��ɫ
%      [goalposy,goalposx] = ind2sub([n,n],goalposind);    %ע�ⷵ�ص��к��е�λ��
%      [startposy,startposx] = ind2sub([n,n],startposind);
%     plot(goalposx+0.5,goalposy+0.5,'ys','MarkerSize',6,'LineWidth',4);     %��0.5��Ϊ�˰������Ƶ��������룬'ys'��y��ʾyellow,s��ʾSquare(����)
%     plot(startposx+0.5,startposy+0.5,'go','MarkerSize',6,'LineWidth',4);   %'go'��g��ʾgreen,o��ʾCircle(Բ��)
     % add a button so that can re-do the demonstration  
uicontrol('Style','pushbutton','String','RE-DO', 'FontSize',12, ...
         'Position', [10 10 60 40], 'Callback','ASTAR');
 % end of this function 
 writerObj = VideoWriter('ASTAR2.avi');
open(writerObj);
tic
while true %ismember(A,B)������Aͬ��С�ľ�������Ԫ��1��ʾA����Ӧλ�õ�Ԫ����B��Ҳ���֣�0����û�г���
  % for the element in OPEN with the smallest cost
  field(startposind )=5;
  field(goalposind )=7;
   image(1.5,1.5,field); 
grid on; 
axis image; 
drawnow;
   if(max(ismember(setOpen,goalposind))) 
       break;
   end;    
  [temp, ii] = min(setOpenCosts + setOpenHeuristics);   %��OPEN����ѡ�񻨷���͵ĵ�temp,ii�����±�(Ҳ���Ǳ������)
%  [temp1, ii1] = min(setOpenCosts1 + setOpenHeuristics1);
%   if(setOpen(ii)==setOpen1(ii1))
%       break;
%   end;
  % find costs and heuristic of moving to neighbor spaces to goal
  % in order 'R','L','D','U'
  field(setOpen(ii))=3;
 % field(setOpen1(ii1))=3;
  %n = length(field);  % length of the field
    % convert linear index into [row column]
    [currentpos(1), currentpos(2)] = ind2sub([10 10],setOpen(ii));
   % [currentpos1(1), currentpos1(2)] = ind2sub([10 10],setOpen1(ii1));%����������չ�ĵ�ǰ����������꣬ע��currentpos(1)�������꣬currentpos(2)��������
    [goalpos(1) ,goalpos(2)] = ind2sub([10 10],goalposind);       %���Ŀ������������
   % [startpos(1) ,startpos(2)] = ind2sub([10 10],startposind);
    % places to store movement cost value and position
    cost = Inf*ones(4,1); heuristic = Inf*ones(4,1); pos = ones(4,2);  
    %cost1 = Inf*ones(4,1); heuristic1 = Inf*ones(4,1); pos1 = ones(4,2);
%       cost1 = Inf*ones(4,1); heuristic1 = Inf*ones(4,1); pos1 = ones(4,2);  
    % if we can look left, we move from the right  ���
    newx = currentpos(1) ; newy = currentpos(2)-1; %why newy subtract 1?
    if newy > 0  %���û�е��߽�
      pos(1,:) = [newx newy];   %����µ�����
      heuristic(1) = abs(goalpos(1)-newx) + abs(goalpos(2)-newy);    %heuristic(1)Ϊ������������ľ������
      % heuristic(1) = sqrt((goalpos(1)-newx)^2 + (goalpos(2)-newy)^2);    %heuristic(1)Ϊ������������ľ������
      cost(1) = setOpenCosts(ii) +  field(newx,newy);   %costsofarΪ֮ǰ���ѵĴ��ۣ�field(newy,newx)Ϊ������в���ۣ�cost(1)Ϊ�����˷�������ʵ����
    end
%     newx1 = currentpos1(1) ; newy1= currentpos1(2)-1; 
%     if newy1 > 0  %���û�е��߽�
%       pos1(1,:) = [newx1 newy1];   %����µ�����
%       heuristic1(1) = abs(startpos(1)-newx1) + abs(startpos(2)-newy1);    %heuristic(1)Ϊ������������ľ������
%       % heuristic(1) = sqrt((goalpos(1)-newx)^2 + (goalpos(2)-newy)^2);    %heuristic(1)Ϊ������������ľ������
%       cost1(1) = setOpenCosts1(ii1) +  field(newx1,newy1);   %costsofarΪ֮ǰ���ѵĴ��ۣ�field(newy,newx)Ϊ������в���ۣ�cost(1)Ϊ�����˷�������ʵ����
%     end
    % if we can look right, we move from the left  ���Ҳ�ѯ
    newx = currentpos(1); newy = currentpos(2)+1;
    if newy <= 10
      pos(2,:) = [newx newy];
     heuristic(2) = abs(goalpos(1)-newx) + abs(goalpos(2)-newy);  %heuristic(1)Ϊ������������ľ������
     % heuristic(2) = sqrt((goalpos(1)-newx)^2 + (goalpos(2)-newy)^2); 
      cost(2) =setOpenCosts(ii) +  field(newx,newy);
    end
%     newx1 = currentpos1(1); newy1 = currentpos1(2)+1;
%     if newy1 <= 10
%       pos1(2,:) = [newx1 newy1];
%      heuristic1(2) = abs(startpos(1)-newx1) + abs(startpos(2)-newy1);  %heuristic(1)Ϊ������������ľ������
%      % heuristic(2) = sqrt((goalpos(1)-newx)^2 + (goalpos(2)-newy)^2); 
%       cost1(2) =setOpenCosts1(ii1) +  field(newx1,newy1);
%     end
    % if we can look up, we move from down  ���ϲ�ѯ
    newx = currentpos(1)+1; newy = currentpos(2);
    if newx <= 10
      pos(3,:) = [newx newy];
      heuristic(3) = abs(goalpos(1)-newx) + abs(goalpos(2)-newy);    %heuristic(1)Ϊ������������ľ������
    %  heuristic(3) = sqrt((goalpos(1)-newx)^2 + (goalpos(2)-newy)^2); 
      cost(3) =setOpenCosts(ii) + field(newx,newy);
    end
%     newx1 = currentpos1(1)+1; newy1 = currentpos1(2);
%     if newx1 <= 10
%       pos1(3,:) = [newx1 newy1];
%       heuristic1(3) = abs(startpos(1)-newx1) + abs(startpos(2)-newy1);    %heuristic(1)Ϊ������������ľ������
%     %  heuristic(3) = sqrt((goalpos(1)-newx)^2 + (goalpos(2)-newy)^2); 
%       cost1(3) =setOpenCosts1(ii1) + field(newx1,newy1);
%     end
    % if we can look down, we move from up  ���²�ѯ
    newx = currentpos(1)-1; newy = currentpos(2);
    if newx > 0
      pos(4,:) = [newx newy];
      heuristic(4) = abs(goalpos(1)-newx) + abs(goalpos(2)-newy);    %heuristic(1)Ϊ������������ľ������
  %heuristic(4) = sqrt((goalpos(1)-newx)^2 + (goalpos(2)-newy)^2); 
      cost(4) = setOpenCosts(ii) +  field(newx,newy);
    end
%     newx1 = currentpos1(1)-1; newy1 = currentpos1(2);
%     if newx1 > 0
%       pos1(4,:) = [newx1 newy1];
%       heuristic1(4) = abs(startpos(1)-newx1) + abs(startpos(2)-newy1);    %heuristic(1)Ϊ������������ľ������
%   %heuristic(4) = sqrt((goalpos(1)-newx)^2 + (goalpos(2)-newy)^2); 
%       cost1(4) = setOpenCosts1(ii1) +  field(newx1,newy1);
%     end
%         % if we can look left up, we move from down  ���ϲ�ѯ
%          newx = currentpos(1)-1; newy = currentpos(2)-1;
%      if (newy >0 && newx>0)
%        pos1(1,:) = [newx newy];
%         heuristic1(1)=abs(goalpos(1)-newx) + abs(goalpos(2)-newy);    %heuristic(1)Ϊ������������ľ������
%        cost1(1) = setOpenCosts(ii)  + sqrt(2)*field(newx,newy);
%      end
%   %   if we can look right up, we move from down  ���ϲ�ѯ
%      newx = currentpos(1)-1; newy = currentpos(2)+1;
%      if (newy <= n && newx>0)
%       pos1(2,:) = [newx newy];
%      heuristic1(2) = abs(goalpos(1)-newx) + abs(goalpos(2)-newy);    %heuristic(1)Ϊ������������ľ������
%     cost1(2) = setOpenCosts(ii)  +sqrt(2)*field(newx,newy);
%      end
%  %  if we can look left down, we move from down  ���²�ѯ
%      newx = currentpos(1)+1; newy = currentpos(2)-1;
%      if (newy > 0 && newx<=n)
%        pos1(3,:) = [newx newy];
%        heuristic1(3) = abs(goalpos(1)-newx) + abs(goalpos(2)-newy);    %heuristic(1)Ϊ������������ľ������
%       cost1(3) = setOpenCosts(ii)  + sqrt(2)*field(newx,newy);
%      end
%     %if we can look right down, we move from down  ���²�ѯ
%      newx = currentpos(1)+1; newy = currentpos(2)+1;
%      if (newx <= n && newy<=n)
%        pos1(4,:) = [newx newy];    
%       heuristic1(4) = abs(goalpos(1)-newx) + abs(goalpos(2)-newy);    %heuristic(1)Ϊ������������ľ������
%        cost1(4) = setOpenCosts(ii)  + sqrt(2)*field(newx,newy);
%      end
     posinds = sub2ind([10,10],pos(:,1),pos(:,2));
   %  posinds1 = sub2ind([10,10],pos1(:,1),pos1(:,2));
    %  posinds1 = sub2ind([n n],pos1(:,1),pos1(:,2));
      setClosed = [setClosed; setOpen(ii)];     %��temp����CLOSE����
   setClosedCosts = [setClosedCosts; setOpenCosts(ii)];  %��temp�Ļ��Ѽ���ClosedCosts
  %   setClosed1 = [setClosed1; setOpen1(ii1)];     %��temp����CLOSE����
  %setClosedCosts1 = [setClosedCosts1; setOpenCosts1(ii1)];  %��temp�Ļ��Ѽ���ClosedCosts
  % update OPEN and their associated costs  ����OPEN�� ��Ϊ�������
    if (ii > 1 && ii < length(setOpen))   %temp��OPEN����м䣬ɾ��temp
    setOpen = [setOpen(1:ii-1); setOpen(ii+1:end)];
    setOpenCosts = [setOpenCosts(1:ii-1); setOpenCosts(ii+1:end)];
    setOpenHeuristics = [setOpenHeuristics(1:ii-1); setOpenHeuristics(ii+1:end)];
  elseif (ii == 1)
    setOpen = setOpen(2:end);   %temp��OPEN��ĵ�һ��Ԫ�أ�ɾ��temp
    setOpenCosts = setOpenCosts(2:end);
    setOpenHeuristics = setOpenHeuristics(2:end);
  else     %temp��OPEN������һ��Ԫ�أ�ɾ��temp
    setOpen = setOpen(1:end-1);
    setOpenCosts = setOpenCosts(1:end-1);
    setOpenHeuristics = setOpenHeuristics(1:end-1);
  end

%    if (ii1 > 1 && ii1 < length(setOpen1))   %temp��OPEN����м䣬ɾ��temp
%     setOpen1 = [setOpen1(1:ii1-1); setOpen1(ii1+1:end)];
%     setOpenCosts1 = [setOpenCosts1(1:ii1-1); setOpenCosts1(ii1+1:end)];
%     setOpenHeuristics1 = [setOpenHeuristics1(1:ii1-1); setOpenHeuristics1(ii1+1:end)];
%   elseif (ii1 == 1)
%     setOpen1 = setOpen1(2:end);   %temp��OPEN��ĵ�һ��Ԫ�أ�ɾ��temp
%     setOpenCosts1 = setOpenCosts1(2:end);
%     setOpenHeuristics1 = setOpenHeuristics1(2:end);
%   else     %temp��OPEN������һ��Ԫ�أ�ɾ��temp
%     setOpen1 = setOpen1(1:end-1);
%     setOpenCosts1 = setOpenCosts1(1:end-1);
%     setOpenHeuristics1 = setOpenHeuristics1(1:end-1);
%   end
  % for each of these neighbor spaces, assign costs and pointers; 
  % and if some are in the CLOSED set and their costs are smaller, 
  % update their costs and pointers
  for jj=1:length(posinds)      %������չ���ĸ����������
    % if cost infinite, then it's a wall, so ignore
    if(field(posinds(jj))~=3 && field(posinds(jj))~=2 && field(posinds(jj))~=5)
       field(posinds(jj))=4; 
%        if ~isinf(cost(jj))    %����˵��ʵ�ʴ��۲�ΪInf,Ҳ����û������ǽ
      % if node is not in OPEN or CLOSED then insert into costchart and 
      % movement pointers, and put node in OPEN
      if ~max([setClosed; setOpen] == posinds(jj)) %����˵㲻��OPEN���CLOSE����
        fieldpointers(posinds(jj)) = movementdirections(jj); %���˵�ķ�����ڶ�Ӧ��fieldpointers��
        costchart(posinds(jj)) = cost(jj); %��ʵ�ʴ���ֵ�����Ӧ��costchart��
        setOpen = [setOpen; posinds(jj)]; %���˵����OPEN����
        setOpenCosts = [setOpenCosts; cost(jj)];   %����OPEN��ʵ�ʴ���
        setOpenHeuristics = [setOpenHeuristics; heuristic(jj)];    %����OPEN����������
      % else node has already been seen, so check to see if we have
      % found a better route to it.
      elseif max(setOpen == posinds(jj)) %����˵���OPEN����
        I = find(setOpen == posinds(jj));   %�ҵ��˵���OPEN���е�λ��
        % update if we have a better route
        if setOpenCosts(I) > cost(jj)  %�����OPEN���еĴ˵�ʵ�ʴ��۱��������õĴ�
          costchart(setOpen(I)) = cost(jj);    %����ǰ�Ĵ��۴���costchart�У�ע��˵���costchart�е�������������������һ�µģ�setOpen(I)��ʵ����posinds(jj)������ͬfieldpointers
          setOpenCosts(I) = cost(jj);      %����OPEN���еĴ˵���ۣ�ע��˵���setOpenCosts�е���������setOpen����һ�µģ���ͬsetOpenHeuristics
          setOpenHeuristics(I) = heuristic(jj);    %����OPEN���еĴ˵���������(����Ϊ�����û�б��)
          fieldpointers(setOpen(I)) = movementdirections(jj);   %���´˵�ķ���   
        end
      end
     end
  end
%    for jj=1:length(posinds1)      %������չ���ĸ����������
%     % if cost infinite, then it's a wall, so ignore
%     if(field(posinds1(jj))~=3 && field(posinds1(jj))~=2 && field(posinds1(jj))~=7)
%        field(posinds1(jj))=4; 
% %        if ~isinf(cost(jj))    %����˵��ʵ�ʴ��۲�ΪInf,Ҳ����û������ǽ
%       % if node is not in OPEN or CLOSED then insert into costchart and 
%       % movement pointers, and put node in OPEN
%       if ~max([setClosed1; setOpen1] == posinds1(jj)) %����˵㲻��OPEN���CLOSE����
%         fieldpointers(posinds1(jj)) = movementdirections(jj); %���˵�ķ�����ڶ�Ӧ��fieldpointers��
%         costchart(posinds1(jj)) = cost1(jj); %��ʵ�ʴ���ֵ�����Ӧ��costchart��
%         setOpen1 = [setOpen1; posinds1(jj)]; %���˵����OPEN����
%         setOpenCosts1 = [setOpenCosts1; cost1(jj)];   %����OPEN��ʵ�ʴ���
%         setOpenHeuristics1 = [setOpenHeuristics1; heuristic1(jj)];    %����OPEN����������
%       % else node has already been seen, so check to see if we have
%       % found a better route to it.
%       elseif max(setOpen1 == posinds1(jj)) %����˵���OPEN����
%         I = find(setOpen1 == posinds1(jj));   %�ҵ��˵���OPEN���е�λ��
%         % update if we have a better route
%         if setOpenCosts1(I) > cost1(jj)  %�����OPEN���еĴ˵�ʵ�ʴ��۱��������õĴ�
%           costchart(setOpen1(I)) = cost1(jj);    %����ǰ�Ĵ��۴���costchart�У�ע��˵���costchart�е�������������������һ�µģ�setOpen(I)��ʵ����posinds(jj)������ͬfieldpointers
%           setOpenCosts1(I) = cost1(jj);      %����OPEN���еĴ˵���ۣ�ע��˵���setOpenCosts�е���������setOpen����һ�µģ���ͬsetOpenHeuristics
%           setOpenHeuristics1(I) = heuristic1(jj);    %����OPEN���еĴ˵���������(����Ϊ�����û�б��)
%           fieldpointers(setOpen1(I)) = movementdirections(jj);   %���´˵�ķ���   
%         end
%       end
%      end
%    end
  
if isempty(setOpen)
  %if (isempty(setOpen) && isempty(setOpen1))
      break; 
 end%��OPEN��Ϊ�գ�������Ծ��������е��Ѿ���ѯ���
  frame = getframe;
    writeVideo(writerObj,frame);
end
if max(ismember(setOpen1,goalposind))    %���ҵ�Ŀ���ʱ
%if(setOpen(ii)==setOpen1(ii1))
  disp('���ҵ�·��!');  %disp�� Display array�� disp(X)ֱ�ӽ�������ʾ����������ʾ�����֣����XΪstring����ֱ���������X
   n = length(fieldpointers);  % length of the field
    posind = goalposind;
    [px,py] = ind2sub([10,10],posind);
    p = [px py];
    p1=posind;
%    until we are at the starting position
    while ~strcmp(fieldpointers{posind},'S')    %����ѯ���ĵ㲻��'S'���ʱ
      switch fieldpointers{posind}
        case 'L' % move left  �����øõ����Դ�㷽��Ϊ��ʱ
          py = py +1;
        case 'R' % move right
          py = py - 1;
        case 'U' % move up
          px = px - 1;
        case 'D' % move down
          px = px + 1;
%         case'RD'
%           px = px - 1;  
%            py = py -1;
%         case'LD'   
%             px = px- 1; 
%              py = py + 1;
%         case'LU'  
%             px = px +1;
%             py = py +1;
%         case'RU'  
%             px = px + 1; 
%             py = py - 1;
     end
      p = [p; px py];
      % convert [row column] to linear index
      posind = sub2ind([10 10],px,py);
      p1=[p1;posind];  
    end  
      p1=flipud(p1);    
%      plot(p(:,2)+0.5,p(:,1)+0.5,'Color',1*zeros(3,1),'LineWidth',2);
    for k = 2:length(p1) - 1 
        field(p1(k)) =6;
        image(1.5, 1.5, field);
        grid on;
        axis image;
        set(gca,'YDir','normal');  
        frame = getframe;
        writeVideo(writerObj,frame);
    end      
    close(writerObj);
 else if isempty(setOpen)
%else
  disp('·��������!'); 
     end
 end
%end
title('����{ \color{red}A*} �㷨��·���滮 ','fontsize',16)
toc