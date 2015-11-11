%load data

file = load('matrix_detection.txt');


ndata = size(file,1);

%time detection in ms
t1 = file(:,1);

%matrici di rotazione

for idx = 2:17
	ROTT{idx}=file(:,2+(idx-1)*16:2+(idx-1)*16+15);
end;

%plot time
if 0,
	figure;plot(t1);
end;	

%extract rot matrices and translation vectors
for i = 1:ndata,
	%for every snapshot
	tmpR = [ROTT(i,1:3)' ROTT(i,5:7)' ROTT(i,9:11)'];
	%set a validity flag
	if norm(tmpR)>2
		%actually >1 should be sufficient
		ROTT2{i}.valid=0;
		tmpR = NaN * ones(3,3);
		tmpEuler = [NaN NaN NaN];
		tmpT = [NaN NaN NaN];
	else
		ROTT2{i}.valid = 1;
		%extract T and R matrix
		tmpT(3) = ROTT(i,13:15);
		%testo if the R matrix is marker_to_cam or cam_to_marker
		tmpR = tmpR';

		 %get euler angles
		[tmpyaw tmppitch tmproll] = dcm2angle(tmpR);
		%NOTE: dcm2angle requires a matrix that rotates vectors from
		%	fixed to moved frame (a C_n^b)
		%   and so angle2dcm construct a C_n^b 
		%   now since aruco returns DCM_cam_to_board e T_cam_to_board
		%   this must e considered when compositing transformations
		tmpEuler = [tmproll tmppitch tmpyaw];

		ROTT2{i}.R = tmpR;
		ROTT2{i}.T = tmpT;
        ROTT2{i}.Euler = tmpEuler;
end;



euler_N = zeros(ndata, 3);
pos_N = zeros(ndata, 3);

for i = 1:size(file,1),
	euler_N(i,1:3) = ROTT2{i}.Euler;
	pos_N(i,1:3) = ROTT2{i}.T;
end;

%% plot rotation vector
figure;
plot(euler_N(:,1:3).*180/pi);grid on;
title('marker');
legend('roll','pitch','yaw');


%plot translation vector
figure;
plot(pos_N(:,1:3));grid on;
title('marker');
legend('X','Y','Z');



















