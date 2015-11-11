%load data from raspicam

M = load('dati 7\matrix_detection.txt');


%unpack M
% Tempo1 (ms)
% Tempo2
% Tempo3
% Matrice4x4 (relativo alla board aruco  )
% Matrice4x4 ( relativo alla board 1 )
% Matrice4x4 ( relativo alla board 2 )
% Matrice4x4 ( relativo alla board 3 )
ndata = size(M,1);
t1 = M(:,1);
t2 = M(:,2);
t3 = M(:,3);
nboards = 4;
for idx=1:nboards,
    ROTT{idx} = M(:,4+(idx-1)*16:4+(idx-1)*16+15);
end;

%% plot times
if 0,
    figure;plot(t1);
    figure;plot(t2);
    figure;plot(t3);
    figure;plot([t1 t2 t3]);
end;
    

%% extract rot matrices and translation vectors
for i = 1:ndata,
    %for every snapshot
    for idx=1:nboards,
        tmpR = [ROTT{idx}(i,1:3)' ROTT{idx}(i,5:7)' ROTT{idx}(i,9:11)'];
        %set validity flag
        if norm(tmpR)>2,
            %actually >1 should be sufficient
            ROTT2{i}.valid{idx} = 0;
            tmpR = NaN * ones(3,3);
            tmpEuler = [NaN NaN NaN];
            tmpT = [NaN NaN NaN];
        else 
            ROTT2{i}.valid{idx} = 1;
            %extract T and rot matrix
            tmpT = ROTT{idx}(i,13:15)'; 
            %NOTE: must change sign of 3rd row of 4x4 omogeneous matrix
            %       because aruco in func. getModelViewMatrix changes sign of third row of the 4x4 omo mat
            %   => change sign of third row of rotation matrix
            %   => change sign of third element of translation vector 
            tmpR(3,:) = -tmpR(3,:);
            tmpT(3) = -tmpT(3);
            %TEST: this is a test to check if aruco is providing a
            %DCM_cam_to_board or DCM_board_to_cam
            tmpR = tmpR';
            
            %get euler angles
            [tmpyaw tmppitch tmproll] = dcm2angle(tmpR);
            %NOTE: dcm2angle requires a matrix that rotates vectors from
            %   fixed to moved frame (a C_n^b)
            %   and so angle2dcm construct a C_n^b 
            %   now since aruco returns DCM_cam_to_board e T_cam_to_board
            %   this must e considered when compositing transformations
            tmpEuler = [tmproll tmppitch tmpyaw];
        end
        ROTT2{i}.R{idx} = tmpR;
        ROTT2{i}.T{idx} = tmpT; 
        ROTT2{i}.Euler{idx} = tmpEuler;
    end;
end;



%% get plottable vectors 
euler_N = zeros(ndata,nboards*3);
pos_N = zeros(ndata,nboards*3);
for i = 1: size(M,1),
    %for every snapshot
    for idx=1:nboards,
        euler_N(i,(idx-1)*3+1:idx*3)=ROTT2{i}.Euler{idx};
        pos_N(i,(idx-1)*3+1:idx*3)=ROTT2{i}.T{idx};
    end;
end;

 


%% plot rotation vector
figure;
plot(euler_N(:,1:3).*180/pi);grid on;
title('board 0');
legend('roll','pitch','yaw');

figure;
plot(euler_N(:,4:6).*180/pi);grid on;
title('board 1');
legend('roll','pitch','yaw');

figure;
plot(euler_N(:,7:9).*180/pi);grid on;
title('board 2');
legend('roll','pitch','yaw');

figure;
plot(euler_N(:,10:12).*180/pi);grid on;
title('board 3');
legend('roll','pitch','yaw');


%plot translation vector
figure;
plot(pos_N(:,1:3));grid on;
title('board 0');
legend('X','Y','Z');

figure;
plot(pos_N(:,4:6));grid on;
title('board 1');
legend('X','Y','Z');

figure;
plot(pos_N(:,7:9));grid on;
title('board 2');
legend('X','Y','Z');

figure;
plot(pos_N(:,10:12));grid on;
title('board 3');
legend('X','Y','Z');

%compare RPY 2 
figure;
plot(euler_N(:,1:3:end).*180/pi);grid on;
title('Roll');
legend('board 0','board 1','board 2','board 3');

figure;
plot(euler_N(:,2:3:end).*180/pi);grid on;
title('Pitch');
legend('board 0','board 1','board 2','board 3');

figure;
plot(euler_N(:,3:3:end).*180/pi);grid on;
title('yaw');
legend('board 0','board 1','board 2','board 3');




%% find relations between boards (with board 0 as reference)
%   ipotesi 1 :aruco returns DCM_cam_to_board e T_cam_to_board
%       this must e considered when compositing transformations
%       => angle2dcm(euler_N) returns a DCM_cam_to_board 
%       => the vehicle C_n_b is then DCM_cam_to_board ^ T
%       => the ref frame is board 0 
%       => 

% compute Euler_0_to_idx and pos_0_to_idx

euler_0_to_N = euler_N.*0;
pos_0_to_N = pos_N.*0;

for i = 1: size(euler_N,1),
    %for every snapshot
    DCM_cam_to_0 = angle2dcm(euler_N(i,3),euler_N(i,2),euler_N(i,1));  % yaw pitch roll !!!!
    for idx=2:nboards,
        %compute relative pose betwwen boards euler_0_to_N
        DCM_cam_to_idx = angle2dcm(euler_N(i,(idx-1)*3+3),euler_N(i,(idx-1)*3+2),euler_N(i,(idx-1)*3+1));
        DCM_0_to_idx = DCM_cam_to_idx * (DCM_cam_to_0');
        %retrieve euler angles 
        [tmpyaw tmppitch tmproll] = dcm2angle(DCM_0_to_idx);
        euler_0_to_N(i,(idx-1)*3+1:idx*3)=[tmproll tmppitch tmpyaw];
        
        %compute pos_0_to_idx
        %   get pos_0_to_idx in camera frame 
        tmp_pos_0_to_idx_cam = pos_N(i,(idx-1)*3+1:idx*3)-pos_N(i,1:3);
        %   get pos in board 0 frame
        tmp_pos_0_to_idx_board0 = DCM_cam_to_0 * tmp_pos_0_to_idx_cam';
        pos_0_to_N(i,(idx-1)*3+1:idx*3)=tmp_pos_0_to_idx_board0';
        
    end;
end;

%plot rotation vector
figure;
plot(euler_0_to_N(:,1:3).*180/pi);grid on;
title('board 0 to 0');
legend('roll','pitch','yaw');

figure;
plot(euler_0_to_N(:,4:6).*180/pi);grid on;
title('board 0 to 1');
legend('roll','pitch','yaw');

figure;
plot(euler_0_to_N(:,7:9).*180/pi);grid on;
title('board 0 to 2');
legend('roll','pitch','yaw');

figure;
plot(euler_0_to_N(:,10:12).*180/pi);grid on;
title('board 0 to 3');
legend('roll','pitch','yaw');


%plot translation vector
figure;
plot(pos_0_to_N(:,1:3));grid on;
title('board 0 to 0 ');
legend('X','Y','Z');

figure;
plot(pos_0_to_N(:,4:6));grid on;
title('board 0 to 1');
legend('X','Y','Z');

figure;
plot(pos_0_to_N(:,7:9));grid on;
title('board 0 to 2');
legend('X','Y','Z');

figure;
plot(pos_0_to_N(:,10:12));grid on;
title('board 0 to 3');
legend('X','Y','Z');


%% compute camera absolute position, wrt board 0 
%   and compare data from all boards
euler_0_to_cam = euler_N.*0;
pos_0_to_cam = pos_N.*0;

%first compute mean of absolute positions and euler angles
board_rel_pos=zeros(1,nboards*3);
board_rel_euler=zeros(1,nboards*3);
for idx=1:nboards,
    board_rel_pos((idx-1)*3+1:idx*3) = mean(pos_0_to_N(find(not(isnan(pos_0_to_N(:,(idx-1)*3+1)))),(idx-1)*3+1:idx*3));
    board_rel_euler((idx-1)*3+1:idx*3) = mean(euler_0_to_N(find(not(isnan(euler_0_to_N(:,(idx-1)*3+1)))),(idx-1)*3+1:idx*3));
end;
disp('relative position and attitude vectors');
board_rel_pos
board_rel_euler



for i = 1: size(euler_N,1),
    %for every snapshot
    for idx=1:nboards,
        %compute relative pose betwwen boards euler_0_to_N
        DCM_cam_to_idx = angle2dcm(euler_N(i,(idx-1)*3+3),euler_N(i,(idx-1)*3+2),euler_N(i,(idx-1)*3+1));
        DCM_board_0_to_idx = angle2dcm(board_rel_euler((idx-1)*3+3),board_rel_euler((idx-1)*3+2),board_rel_euler((idx-1)*3+1));
        DCM_board_idx_to_0 = DCM_board_0_to_idx';
        pos_0_to_cam(i,(idx-1)*3+1:idx*3) = ( board_rel_pos((idx-1)*3+1:idx*3)' + DCM_board_idx_to_0 * DCM_cam_to_idx * -pos_N(i,(idx-1)*3+1:idx*3)' )';

        tmp_DCM_cam_to_0 = DCM_board_idx_to_0 * DCM_cam_to_idx;
        tmp_DCM_0_to_cam = tmp_DCM_cam_to_0';
        [tmpyaw tmppitch tmproll] = dcm2angle(tmp_DCM_0_to_cam);
        euler_0_to_cam(i,(idx-1)*3+1:idx*3) = [tmproll tmppitch tmpyaw ];
     end;
end;

%compare camera position and attitude estimates from various markers 
figure;
plot(pos_0_to_cam(:,1:3:end));grid on;
title('X');
legend('board 0','board 1','board 2','board 3');

figure;
plot(pos_0_to_cam(:,2:3:end));grid on;
title('Y');
legend('board 0','board 1','board 2','board 3');

figure;
plot(pos_0_to_cam(:,3:3:end));grid on;
title('Z');
legend('board 0','board 1','board 2','board 3');


figure;
plot(euler_0_to_cam(:,1:3:end).*180/pi);grid on;
title('roll');
legend('board 0','board 1','board 2','board 3');

figure;
plot(euler_0_to_cam(:,2:3:end).*180/pi);grid on;
title('pitch');
legend('board 0','board 1','board 2','board 3');

figure;
plot(euler_0_to_cam(:,3:3:end).*180/pi);grid on;
title('yaw');
legend('board 0','board 1','board 2','board 3');


%% implement a very simple tracking filter
%camera dynamics: 
%   pos_k+1 = pos_k + v_k * dt
%   v_k = v_k
%   
%Initialization:
%   v_k = 0;
%   pos_k = cam_pos from first frame
%
%Measures: 
%   cam_pos_0 from various boards 
%
%Corrections
%   pos_k+1 = pos_k+1 + K (measuresd_cam - pos_k)
%
filtered_cam_pos = [0 0 0];
filtered_cam_vel = [0 0 0];
DT = 0.2; %sample time ... for the moment is constant
K_vel = 0.2;
K_pos = 0.2;


filtered_cam_posV = zeros(size(pos_0_to_cam,1),3);
filtered_cam_velV = zeros(size(pos_0_to_cam,1),3);
first_fix_flag = 1;
for i = 2: size(pos_0_to_cam,1),
    %update vel 
    filtered_cam_vel = 0.95 * filtered_cam_vel; %light damping on speed? 
    %update pos 
    filtered_cam_pos = filtered_cam_pos + filtered_cam_vel * DT;
    %for every snapshot
    
    for idx=1:nboards,
        %check if a pose was measured
        if not(isnan(pos_0_to_cam(i,(idx-1)*3+1:idx*3))),
            %check if this is the first fix
            if first_fix_flag, 
                %reset state 
                filtered_cam_pos = pos_0_to_cam(i,(idx-1)*3+1:idx*3); 
                filtered_cam_vel = [0 0 0];
                first_fix_flag = 0;
            else 
                %correct state with measurements 
                filtered_cam_vel = filtered_cam_vel + K_vel * (pos_0_to_cam(i,(idx-1)*3+1:idx*3) - filtered_cam_pos);
                filtered_cam_pos = filtered_cam_pos + K_pos * (pos_0_to_cam(i,(idx-1)*3+1:idx*3) - filtered_cam_pos);
            end;
        end;
     end;
    filtered_cam_posV(i,:) = filtered_cam_pos;
    filtered_cam_velV(i,:) = filtered_cam_vel;
end;


figure;
plot(pos_0_to_cam(:,1:3:end));grid on;
hold on;
plot(filtered_cam_posV(:,1),'linewidth',3); 
title('X');
legend('board 0','board 1','board 2','board 3','filtered');

figure;
plot(pos_0_to_cam(:,2:3:end));grid on;
hold on;
plot(filtered_cam_posV(:,2),'linewidth',3); 
title('Y');
legend('board 0','board 1','board 2','board 3','filtered');

figure;
plot(pos_0_to_cam(:,3:3:end));grid on;
hold on;
plot(filtered_cam_posV(:,3),'linewidth',3); 
title('Z');
legend('board 0','board 1','board 2','board 3','filtered');

figure;
grid on;
plot(filtered_cam_velV(:,1:3),'linewidth',3); 
title('filtered vel');
legend('Vx','Vy','Vz');







