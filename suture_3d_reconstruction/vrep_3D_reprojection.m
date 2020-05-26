function [repro_u_left, repro_v_left] = vrep_3D_reprojection(xyz, left_image)

    %% Reprojection of the computed 3D coordinates to two frames in vrep simulator
    % parameters acquisitions --> xyz: row--> [x y z]
    global stereoParams; 

    c_x_left = stereoParams.CameraParameters1.PrincipalPoint(1,1);
    c_y_left = stereoParams.CameraParameters1.PrincipalPoint(1,2);
    f_x_left = stereoParams.CameraParameters1.FocalLength(1,1);
    f_y_left = stereoParams.CameraParameters1.FocalLength(1,2);

    c_x_right = stereoParams.CameraParameters2.PrincipalPoint(1,1);
    c_y_right = stereoParams.CameraParameters2.PrincipalPoint(1,2);
    f_x_right = stereoParams.CameraParameters2.FocalLength(1,1);
    f_y_right = stereoParams.CameraParameters2.FocalLength(1,2);


    repro_u_left = size(left_image, 2) - (f_x_left * (xyz(:, 1)./abs(xyz(:, 3))) + c_x_left); % x direction - reversed image projection
    repro_v_left = size(left_image, 1) - (f_y_left * (xyz(:, 2)./abs(xyz(:, 3))) + c_y_left); % y direction - reversed image projection

%     figure; imshow(left_image); hold on; scatter(repro_u_left, repro_v_left, 5, 'b');
end