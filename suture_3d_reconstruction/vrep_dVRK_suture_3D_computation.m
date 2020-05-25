Path='simulated_img/';
FolderName=datestr(now,30);   
mkdir(Path,FolderName);
Now_Path = [Path, FolderName, '/'];
%%
% ------- stereo camera cali
clearvars -except stereoParams; clc; close all;

%% camera calibration from data in folders
% change the directory ot "vrep_ECM_cali\camera_1_raw\" and "vrep_ECM_cali\camera_2_raw\"
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



