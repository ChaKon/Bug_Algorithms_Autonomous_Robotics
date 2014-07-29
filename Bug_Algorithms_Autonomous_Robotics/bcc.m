
function bcc()
global epic
global go
global linearTel
global angularTel
global Time;
global stage
global IR
global X
global Y
% global X   % for sensor calibration
stage=0;
readsen=[];
distan=[];
initialize();
tic
time=0;
%%%%for Head towards goal
global Goalx
global Goaly
Goalx = input('Enter X coordinate of goal');
Goaly = input('Enter Y coordinate of goal');
%%%% for head towards goal
while(go)
%     X 
%     Y
    epic=update(epic);
     Time=toc;
    disp(sprintf('\n\n\n\n\n\nTime [s]: \t%g',Time));
    readSensors();
    computeOdometry();
%     [linearTel,angularTel]=highlevelcontroller();

%       HeadTowardsGoal();
    [linearTel,angularTel]=Bug2algorithm();
    [linear,angular]=coordinate([linearTel],[angularTel]);
    lowLevelController(linear,angular);
    time=toc;
    %%%%for sensor calibratio
%     lowLevelController(-10,0);
%     readSensors();
%     computeOdometry();
%     readsen=[readsen;IR(1,1)];
%     distan=[distan;abs(X)];
% %%%%%%sensor calibration
        
    
end
%%%%%for sensor calibration
% save('1.mat','readsen')
% save('2.mat','distan')
% load 1.mat
% load 2.mat
% figure (2);
% plot(readsen,distan)
% hold on
% coef = polyfit(readsen,distan,7);
% x = 0:3500;
% y = coef(1)*x.^7+coef(2)*x.^6+coef(3)*x.^5+coef(4)*x.^4+coef(5)*x.^3+coef(6)*x.^2+coef(7)*x.^1+coef(8)*x.^0;
% plot(x,y,'r')
% 

finalize();
end

function initialize()
global epic
global go
global linearTel
global angularTel
global X
global Y
global Theta
global L_past
global R_past
global Xlog
global Ylog
global Thetalog
global Timelog
global IRlog
global IR

close all

epic=ePicKernel;
epic=connect(epic,'COM14');


epic=activate(epic, 'proxi');
epic=activate(epic, 'pos');

go=1;
linearTel=0;
angularTel=0;
X=0;
Y=0;
Theta=0;
L_past=0;
R_past=0;
Xlog=[];
Ylog=[];
Thetalog=[];
Timelog=[];
IRlog=[];



end

function readSensors()

global epic
global IRlog
global IR

coef(7)=2.2372*10^-21;
coef(6)=- 2.7966*10^-17;
coef(5)=1.3764*10^-13;
coef(4)=- 3.4013*10^-10;
coef(3)=4.465*10^-07;
coef(2)=- 0.00030517;
coef(1)=0.10252;

[proxi,up]=get(epic,'proxi');
% IR=proxi; % sensor calibration
for i = 1:8
      value=proxi(i);
      proxi(i) = coef(7)*value^6+coef(6)*value^5+coef(5)*value^4+coef(4)*value^3+coef(3)*value^2+coef(2)*value^1+coef(1);
      proxi(i)=proxi(i)*100;
end
IR=proxi; %For high level control


[pos,up]=get(epic,'pos');

text=sprintf('Encoders: \t%g\t%g',pos(1),pos(2));
disp(text)

text=sprintf('IR [cm]: \t%g\t%g\t%g\t%g\t%g\t%g\t%g\t%g',proxi(1),proxi(2),proxi(3),proxi(4),proxi(5),proxi(6),proxi(7),proxi(8));
disp(text)

IRlog=[IRlog; proxi(1) proxi(2) proxi(3) proxi(4) proxi(5) proxi(6) proxi(7) proxi(8)];
    
end

function computeOdometry();

global epic
global X
global Y
global Theta
global L_past
global R_past
global Xlog
global Ylog
global Thetalog
global Timelog
global Time
global d

[pos,up]=get(epic,'pos');

l=pos(1);
r=pos(2);

dl = ((l-L_past) / 1000.0) * 2 * 3.141592* 0.0205; % distance covered by left wheel in meter
dr = ((r-R_past) / 1000.0) * 2 * 3.141592* 0.0205; % distance covered by right wheel in meter
d  = (dr + dl) / 2 ;
da = (dr - dl) / 0.053; % delta orientation
R_past=r;
L_past=l;


Theta=Theta+da;
if Theta>pi
    Theta=Theta-2*pi;
elseif Theta<-pi
    Theta=Theta+2*pi;
end

X=X+d*cos(Theta);
Y=Y+d*sin(Theta);

text=sprintf('Odometry: \tX[cm]:%g\tY[cm]:%g\tTheta[º]:%g',X*100,Y*100,Theta*180/pi);
disp(text)

Xlog=[Xlog X];
Ylog=[Ylog Y];
Thetalog=[Thetalog Theta];
Timelog=[Timelog Time];


end

% function [linearTel,angularTel]=teleoperation()
%     global go
%     global linearTel
%     global angularTel
%     
%     
%     
%     action=input('\n---\nPress key: w -> forward; s-> backward; d-> right; a-> left; x-> stop; q->quit; return-> update\n','s');
%    
%     speed=200;
%     
%     if action=='q' | action=='Q'
%         go=0;
%     elseif action=='w' | action=='W'
%         linearTel=speed;
%         angularTel=0;  
%     elseif action=='s'| action=='S'
%         linearTel=-speed;
%         angularTel=0;  
%     elseif action=='a'| action=='A'
%         linearTel=0;
%         angularTel=speed;  
%     elseif action=='d'| action=='D'
%         linearTel=0;
%         angularTel=-speed;  
%     elseif action=='x'| action=='X'
%         linearTel=0;
%         angularTel=0;  
%     end
%     
%     text=sprintf('Teleoperation: \tLinear vel: %g\tAngular vel: %g',linearTel,angularTel);
%     disp(text)
% 
% 
% 
% end

function [linear,angular]=coordinate(linearVector,angularVector)


[f,c]=size(linearVector);

if f==1
    linear = linearVector;
    angular = angularVector;
else
    disp('coordinate: not implemented');
    linear = 0;
    angular = 0;
end


    text=sprintf('Coordinator: \tLinear vel: %g\tAngular vel: %g',linear,angular);
    disp(text)

end

function lowLevelController(linear,angular)
    
    global epic
     
    epic=set(epic,'speed',[linear-angular linear+angular]);
    
    
    text=sprintf('Low Level Controller: \tLeft wheel: %g\tRigth wheel: %g',linear-angular,linear+angular);
    disp(text)

end

function finalize()
global epic
global Xlog
global Ylog
global Thetalog
global Timelog
global IRlog


epic=set(epic,'speed',[0 0]);
epic=update(epic);
epic=disconnect(epic);
figure(1)
plot(Xlog,Ylog);
axis equal;
title('XY trajectory [m]');
figure(2)
plot(Timelog,Thetalog*180/pi);
title('Theta [º] with respect to real time [s]');
%figure(3)
[f,c]=size(IRlog);
for i=1:f
    bar([1:8], IRlog(i,1:8));
    title('Distance sensors [cm]');
    pause(0.1)
end
end


%%%high level controller
%%%%test for go straight
% function [linearTel,angularTel]=highlevelcontroller()
%     global go
%     global IR
%     dis=2;
%     if(IR(1,1)<dis || IR(1,2)<dis || IR(1,7)<dis || IR(1,8)<dis)
%                 linearTel=0.0;
%                 angularTel=0.0;
%         
%     else
%             linearTel =80.0;
%             angularTel = 0;
%     end
% end
%%%test to turn Left
% function [linearTel,angularTel]=highlevelcontroller()
%     global go
%     global IR
%     dis=2;
%     if(IR(1,3)<dis+0.5 && IR(1,1)>dis-0.5 && IR(1,2)>dis-0.5)
%                 linearTel=50.0;
%                 angularTel=0.0;
%         
%     else
%             linearTel =0.0;
%             angularTel =100.0;
%     end
% end
%%%%%%Test for go straight and turn left
% function [linearTel,angularTel]=highlevelcontroller()
%   global go
%   global IR
%    global stage
%    
%    
%     dis=2;
%     switch (stage)
%         case 0
%             if(IR(1,1)<dis || IR(1,2)<dis || IR(1,7)<dis || IR(1,8)<dis)
%                 stage = 1;
%                 linearTel =0.0;
%                 angularTel = 0;
%                 
%             else
%                 stage = 0;
%                 linearTel =80.0;
%                 angularTel = 0;
%                 
%             end
%             case 1
%                 if(IR(1,3)<dis+0.5 && IR(1,1)>dis-0.5 && IR(1,2)>dis-0.5)
% %                       stage = 0;
%                     linearTel=0;
%                     angularTel=0;
%                     stage = 0;
%                 else
%                     linearTel=0.0;
%                     angularTel=100.0;
%                 end
%     end
% end
%%%% for obstacle following
% function [linearTel,angularTel]=highlevelcontroller()
% global go
% global IR
% global stage
% 
% 
% dis=2;
% switch (stage)
% case 0
% if(IR(1,1)<dis || IR(1,2)<dis || IR(1,7)<dis || IR(1,8)<dis)
% stage = 1;
% linearTel =0.0;
% angularTel = 0;
% 
% else
% stage = 0;
% linearTel =80.0;
% angularTel = 0;
% 
% end
% case 1
% if((IR(1,3)<(dis+0.6)) && (IR(1,1)>(dis-0.6) && (IR(1,2)>dis)))
% stage = 2;
% linearTel=0.0;
% angularTel=0.0;
% else
% stage = 1;
% linearTel=0.0;
% angularTel=100.0;
% end
% case 2
% if(IR(1,3)<(dis+0.5))
% m=10;
% l1=5;
% l2=45;
% l3=10;
% diff=IR(1,2)-IR(1,3);
% min_sen=0;
% if(IR(1,2)>IR(1,3))
% min_sen=IR(1,3);
% else
% min_sen=IR(1,2);
% end
% if(diff>1)
% if(min_sen<dis-0.5)
% linearTel=80.0;
% angularTel=-((m*min_sen)+l1);%smooth turn at cornor
% elseif(min_sen>dis)
% linearTel=90.0;
% angularTel=-((m*min_sen)+l2);% hard turn at cornor
% else
% linearTel=100.0;
% angularTel=-((m*min_sen)+l3);
% end
% elseif((diff<=1)&&(IR(1,3)<dis))
% linearTel=80.0;
% angularTel=20;
% else
% linearTel=80.0;
% angularTel=0.0;
% end
% elseif((IR(1,1)<dis) || (IR(1,8)<dis))
% stage = 1;
% linearTel=0.0;
% angularTel=0.0;
% 
% else(IR(1,2)>=3.5&& IR(1,3)>=3.5)
% stage =0;
% linearTel=80.0;
% angularTel=0.0;
% 
% 
% % 
% 
% end
% % linearTel=0;
% % angularTel=0;
% end
% end
%%% Head Towards Goal
% function HeadTowardsGoal()
% global X
% global Y
% global Theta
% global Goalx
% global Goaly
% global linearTel
% global angularTel
% global go
% % XR = X;
% % YR = Y;
% 
% %Calculating Euclidian distance to goal
% Eucl = sqrt((Goalx - X)^2+(Goaly - Y)^2);
% slope=Goaly/Goalx;
% 
% % Calculating delta
% Thresh_angle = 10;
% angleR = atan2(Goaly-Y,Goalx-X);
% disp('angleR');
% Delta = angleR - Theta;
% % Distance_mline=abs(YR-(slope)*XR)/sqrt((slope)*(slope)+1);
% Thresh_m=0.02;
% 
% if(Eucl<0.001)
%     disp('Goal');
%     go=0;
%     linearTel =0.0;
%     angularTel =0.0;
% else
%     if((Theta>(angleR+(deg2rad(Thresh_angle))) || (Theta<(angleR-(deg2rad(Thresh_angle))))))
%     angularTel = sign(angleR)*60.0;
%     linearTel = 0.0;
%     else
%     linearTel=100.0;
%     angularTel=0;
%     end
% end
% % 
% end
%%% Head Towards the goal

%%%% BUG2 algorithm
function [linearTel,angularTel]=Bug2algorithm()
global go
global IR
global X
global Y
global stage
global Theta
global Goalx
global Goaly
global M_x
global M_y
speed = 60;


% %Calculating Euclidian distance to goal


dis=2;
    switch (stage)
        case 0
        if(IR(1,1)<dis || IR(1,2)<dis || IR(1,7)<dis || IR(1,8)<dis)
        stage = 1;
        linearTel =0.0;
        angularTel = 0;
        M_x=x;
        M_y=y;
        else
        stage = 0;
        Eucl = sqrt((Goalx - X)^2+(Goaly - Y)^2);
        Thresh_angle = 10;
        angleR = atan2(Goaly-Y,Goalx-X);
        if(Eucl<0.001)
            disp('Goal');
            go=0;
            linearTel =0.0;
            angularTel =0.0;
        else
        if((Theta>(angleR+(deg2rad(Thresh_angle))) || (Theta<(angleR-(deg2rad(Thresh_angle))))))
        angularTel = sign(angleR)*60.0;
        linearTel = 0.0;
        else
        linearTel=100.0;
        angularTel=0;
        end
        end
        end
    case 1  
    if((IR(1,3)<(dis+0.6)) && (IR(1,1)>(dis-0.6) && (IR(1,2)>dis)))
    stage = 2;
    linearTel=0.0;
    angularTel=0.0;
    else
    stage = 1;
    linearTel=0.0;
    angularTel=100.0;
    end
    case 2
    if(IR(1,3)<(dis+0.5))
    m=10;
    l1=5;
    l2=45;
    l3=10;
    diff=IR(1,2)-IR(1,3);
    min_sen=0;
    if(IR(1,2)>IR(1,3))
    min_sen=IR(1,3);
    else
    min_sen=IR(1,2);
    end
    if(diff>1)
    if(min_sen<dis-0.5)
    linearTel=80.0;
    angularTel=-((m*min_sen)+l1);%smooth turn at cornor
    elseif(min_sen>dis)
    linearTel=90.0;
    angularTel=-((m*min_sen)+l2);% hard turn at cornor
    else
    linearTel=100.0;
    angularTel=-((m*min_sen)+l3);
    end
    elseif((diff<=1)&&(IR(1,3)<dis))
    linearTel=80.0;
    angularTel=20;
    else
    linearTel=80.0;
    angularTel=0.0;
    end
    elseif((IR(1,1)<dis) || (IR(1,8)<dis))
    linearTel=0.0;
    angularTel=0.0;
    stage =1;
    else(IR(1,2)>=3.5&& IR(1,3)>=3.5)
    stage =0;
%     linearTel=80.0;
%     angularTel=0.0;
%     Threshold_m=0.03;
%         slope = Goaly/Goalx;
%         Distance_to_mline=abs(Y-(slope)*X)/sqrt((slope)*(slope)+1);
%         % if robot reachs the m-line
%         if((Distance_to_mline < Threshold_m))
%             linearTel=speed;
%             angularTel=5*angularTel;
%             stage = 0;
%         end
% 
% % 
% 
% end
                EuclM = sqrt((Goalx - M_x)^2+(Goaly - M_y)^2);
                Thresh_angle = 10;
                angleR = atan2(Goaly-Y,Goalx-X);
                if(Eucl<EuclM)
                if(Eucl<0.001)
                disp('Goal');
                go=0;
                linearTel =0.0;
                angularTel =0.0;
                else
                    if((Theta>(angleR+(deg2rad(Thresh_angle))) || (Theta<(angleR-(deg2rad(Thresh_angle))))))
                        linearTel = 0.0;
                        angularTel = sign(angleR)*60.0;
                    else
                        linearTel=100.0;
                        angularTel=0;
                    end
                end
%                  disp('stage 2');
                end 
    end
    end
end
   
% linearTel=0;
% angularTel=0;
   
% 
% %%%%%Bug2 algorithm