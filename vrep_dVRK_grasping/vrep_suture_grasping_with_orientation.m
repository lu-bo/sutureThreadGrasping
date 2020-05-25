Path='simulated_img/';
FolderName=datestr(now,30);   
mkdir(Path,FolderName);
Now_Path = [Path, FolderName, '/'];
%%
% ------- stereo camera cali
clearvars -except stereoParams; clc; close all;

%% camera calibration from data in folders
PATH_left_images  = 'C:\Users\Bo\OneDrive - The Hong Kong Polytechnic University\PhD Studies\Semester 4\3D Reconstruction\2017.03.20\v-repApi\virtual_chess_board_calibration\camera_1_raw\';
PATH_right_images = 'C:\Users\Bo\OneDrive - The Hong Kong Polytechnic University\PhD Studies\Semester 4\3D Reconstruction\2017.03.20\v-repApi\virtual_chess_board_calibration\camera_2_raw\';
chess_board_size = 5;
stereoParams = stereo_camera_calibration(PATH_left_images, PATH_right_images, chess_board_size); disp(stereoParams.MeanReprojectionError)

global stereoParams; 

%%
% ------- initial position setups of dVRK system and the suture 
setup_initialization;
vrep_dVRK.intial_config_setup
%%
image_path = 'C:\Users\User\OneDrive - The Hong Kong Polytechnic University\PhD Studies\Semester 4\3D Reconstruction\2017.03.20\v-repApi\';

[left_image, right_image] = dVRK_virtual_sensor.get_stereo_vision;
%left_image  = undistortImage(left_image,  stereoParams.CameraParameters1);
%right_image = undistortImage(right_image, stereoParams.CameraParameters2);

%[left_image, right_image] = rectifyStereoImages(left_image, right_image, stereoParams, 'OutputView', 'full');
%imwrite(left_image, [Now_Path, 'left_image.jpg']); imwrite(right_image, [Now_Path, 'right_image.jpg']);

%imwrite(left_image, [image_path, 'left_image.jpg']); imwrite(right_image, [image_path, 'right_image.jpg']);
%%
all_points_set_1=[]; all_points_set_2=[];
[all_points_set_1, tip_1] = DL_based_intersected_suture_thread_detection_with_clicks(left_image, 'segmented_suture_l.jpg');
% figure; imshow(left_image); hold on; scatter(all_points_set_1(:, 1), all_points_set_1(:, 2), 5, 'r');
[all_points_set_2, tip_2] = DL_based_intersected_suture_thread_detection_with_clicks(right_image, 'segmented_suture_r.jpg');
% figure; imshow(right_image); hold on; scatter(all_points_set_2(:, 1), all_points_set_2(:, 2), 5, 'r');

%%
multi_view_num = 1;
xyzsg = suture_thread_3D_computation_optimization(all_points_set_1, all_points_set_2, multi_view_num, 'vrep');

%% Reprojection of the computed 3D coordinates to two frames
[repro_u_left, repro_v_left] = vrep_3D_reprojection(xyzsg, left_image);

figure; imshow(left_image); 
%hold on; scatter(all_points_set_1(:, 1), all_points_set_1(:, 2), 5, 'r');
hold on; scatter(repro_u_left, repro_v_left, 15, 'b');
%%
figure; 
intensity = [0.1: 0.001: 0.1 + 0.001*(size(xyzsg, 1)-1)];
scatter3(xyzsg(:, 1), xyzsg(:, 2), xyzsg(:, 3), 10, intensity(:), 'filled');
xlabel('X'); ylabel('Y'); zlabel('Z');
colormap(jet); colorbar;

sf = fit([xyzsg(:, 1), xyzsg(:, 2)],xyzsg(:, 3),'poly33');
%hold on; plot(sf,[xyzsg(1:10, 1),xyzsg(1:10, 2)],xyzsg(1:10, 3));

bi_bound = 5*10^(-3); grid_size = 1*10^(-3);
X_plot3=[]; Y_plot3=[]; Z_plot3=[];
Norm_x=[]; Norm_y=[]; Norm_z=[];
for i = 82
[X_plot3(:,:, i), Y_plot3(:,:,i)] = meshgrid(xyzsg(i, 1) - bi_bound: grid_size : xyzsg(i, 1) + bi_bound,...
                                             xyzsg(i, 2) - bi_bound: grid_size : xyzsg(i, 2) + bi_bound);
Z_plot3(:,:,i) = sf(X_plot3(:,:,i), Y_plot3(:,:,i));
hold on; surf(X_plot3(:,:,i), Y_plot3(:,:,i), Z_plot3(:,:,i));
hold on; surfnorm(X_plot3(:,:,i), Y_plot3(:,:,i), Z_plot3(:,:,i))
end
[Norm_x, Norm_y, Norm_z] = surfnorm(X_plot3(:,:,i), Y_plot3(:,:,i), Z_plot3(:,:,i));

%% 
figure(5);
scatter(xyzsg(:, 1), xyzsg(:, 2), 10, intensity(:), 'filled'); hold on;
plot(xyzsg(:, 1), xyzsg(:, 2), 'r', 'LineWidth', 0.5);
xlabel('X'); ylabel('Y');
colormap(jet); colorbar;
grid on;

figure(6); 
scatter(xyzsg(:, 1), xyzsg(:, 3), 10, intensity(:), 'filled'); hold on;
plot(xyzsg(:, 1), xyzsg(:, 3), 'r', 'LineWidth', 0.5);
xlabel('X'); ylabel('Z');
colormap(jet); colorbar;
grid on;
%% 3D points fitting - (further enhancements)
figure; plot3(xyzsg)


%%
accum_len = 0;
set_threshold = 0.03;
for node_i = 2 : size(xyzsg, 1)
    accum_len = norm(xyzsg(node_i, :) - xyzsg(node_i - 1, :)) +accum_len;
    if (accum_len > set_threshold)
        break;
    end
end

vv3 = xyzsg(node_i, :) - xyzsg(node_i - 1, :); scale_vv3 = 1 / norm(vv3); v3_p = scale_vv3 * vv3; v3_n = -v3_p;
vv2 = [0 0 0]; vv2(2) = 0.1; vv2(1) = (-v3_p(2)/v3_p(1))*vv2(2); scale_vv2 = 1 / norm(vv2); v2_p = scale_vv2 * vv2; v2_n = -v2_p;

%% compute the minimal "cost" target angle
[~, current_ori] = vrep_pos_ori('J3_dx_TOOL1', 'J1_PSM1');

[rotm_32, ~, CoCo] = dVRK_get_transformation_and_coordinates('Vision_sensor_left', 'J1_PSM1', xyzsg);

base1_v1 = [v2_p; v2_n]; % y axis
base2_v1 = [v3_p; v3_n]; % z axis 
tipOffSet_v1 = [0, -0.0091, 0]';

rotm_num = 0;
for base1_i = 1 : size(base1_v1, 1)
   for base2_j = 1 : size(base2_v1, 1) 
       
        v1 = [base1_v1(base1_i, 2)*base2_v1(base2_j, 3)-base1_v1(base1_i, 3)*base2_v1(base2_j, 2), ...
              base1_v1(base1_i, 3)*base2_v1(base2_j, 1)-base1_v1(base1_i, 1)*base2_v1(base2_j, 3), ...
              base1_v1(base1_i, 1)*base2_v1(base2_j, 2)-base1_v1(base1_i, 2)*base2_v1(base2_j, 1)]; % direction of v1 is determined by two bases. 
       
        rotm_num = rotm_num + 1;
        rotm_21(:, :, rotm_num) = [v1', base1_v1(base1_i, :)', base2_v1(base2_j, :)'];
        rotation_31(:, :, rotm_num) = rotm_32 * rotm_21(:, :, rotm_num);
        
        tipOffSet_J1_PSM1(:, rotm_num) = rotation_31(:, :, rotm_num) * tipOffSet_v1;
        eulXYZ_31(:, :, rotm_num) = rotm2eul(rotation_31(:, :, rotm_num),'XYZ');
        
        orientation_cost(rotm_num, :) = sum(abs(eulXYZ_31(:, :, rotm_num) - current_ori));
    end
end
[row_rotm, col_rotm] = find(orientation_cost == min(orientation_cost));

%%
selectedEuler_31 = eulXYZ_31(:, :, row_rotm);
selectedTipOffset = tipOffSet_J1_PSM1(:, row_rotm);
Position_PSM1 = CoCo(node_i, :) + selectedTipOffset';

%init_q_6 = 0;
init_q_6 = vrep_dVRK.vrep_dVRK_IK(Position_PSM1, selectedEuler_31, init_q_6);
init_q_6*180/pi
%vrep_dVRK.vrep_grasper_open
%vrep_dVRK.vrep_grasper_close
%vrep_dVRK.intial_config_setup

%%
% pose validation
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
%if (clientID>-1)

[~, Current_system_handle] = vrep.simxGetObjectHandle(clientID, 'J3_dx_TOOL1', vrep.simx_opmode_blocking);
[~, Target_system_handle]  = vrep.simxGetObjectHandle(clientID, 'J1_PSM1', vrep.simx_opmode_blocking);
[~, EulerAngles] = vrep.simxGetObjectOrientation(clientID, Current_system_handle, Target_system_handle, vrep.simx_opmode_blocking);
[~, Position]    = vrep.simxGetObjectPosition(clientID, Current_system_handle, Target_system_handle, vrep.simx_opmode_blocking);
        
EulerAngles*180/pi
%Position

%% Initial rotation matrix between Joint 5 and 6, PSM1, V-REP

Initial_EulerAngle_65 = [-1.5707    1.5306   -0.0403];
Initial_RotationMatrix_65 = eulerAngle_2_rotationMatrix(Initial_EulerAngle_65);

vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
%if (clientID>-1)

% R_{r6}*R_{initial}*R_{r5} = R_{current_joint_5_to_6} 
[~, Current_system_handle] = vrep.simxGetObjectHandle(clientID, 'J2_TOOL1', vrep.simx_opmode_blocking);
[~, Target_system_handle]  = vrep.simxGetObjectHandle(clientID, 'J3_dx_TOOL1', vrep.simx_opmode_blocking);
[~, EulerAngle_65] = vrep.simxGetObjectOrientation(clientID, Current_system_handle, Target_system_handle, vrep.simx_opmode_blocking);
RotationMatrix_65 = eulerAngle_2_rotationMatrix(EulerAngle_65);

[~, EulerAngle_5_self] = vrep.simxGetJointPosition(clientID, Current_system_handle, vrep.simx_opmode_blocking);
EulerAngle_55 = [0 0 EulerAngle_5_self];
RotationMatrix_55 = eulerAngle_2_rotationMatrix(EulerAngle_55);


R_transform_joint6 = RotationMatrix_65*inv(RotationMatrix_55)*inv(Initial_RotationMatrix_65);
ee = rotm2eul(R_transform_joint6,'XYZ');
ee*180/pi




