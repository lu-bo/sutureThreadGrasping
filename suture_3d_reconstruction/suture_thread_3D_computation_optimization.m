function [xyzsg, multi_view_coordinates_RCM_vision] = suture_thread_3D_computation_optimization(all_points_set_1, all_points_set_2, multi_view_num, system_var, show_argm)
    
    global stereoParams
    global G_3D_length
    global P_3D_length
    global H_3D_length
    
    xyzsg =[];
    multi_view_coordinates_RCM_vision = [];
    
    s_1 = size(all_points_set_1, 1); s_2 = size(all_points_set_2, 1);
    
    if (s_1 > 50 && s_2 > 50 && abs(s_1 - s_2) < 100)

%         left_image_seg = imread([image_path, 'segmented_suture_l.jpg']); 
%         psedo_left_seg(:,:,1) = left_image_seg; psedo_left_seg(:,:,2) = left_image_seg; psedo_left_seg(:,:,3) = left_image_seg;
%         right_image_seg = imread([image_path, 'segmented_suture_r.jpg']);
%         psedo_right_seg(:,:,1) = right_image_seg; psedo_right_seg(:,:,2) = right_image_seg; psedo_right_seg(:,:,3) = right_image_seg;
% 
%         %imshow(left_image,'Border','tight'); hold on; scatter(all_points_1(:, 1), all_points_1(:, 2), 5, 'r')
%         Multi = cat(2, left_image, right_image);        
%         %Multi = cat(2, psedo_left_seg, psedo_right_seg);
%         Multi2 = cat(2, psedo_left_seg, psedo_right_seg);
% 
%         Multi_total = cat(1, Multi, Multi2);
%         Multi_total = imresize(Multi_total, scale);
%         
%         figure(2); hold off; 
%         imshow(Multi_total, 'Border','tight');
%         hold on; scatter(all_points_set_1(:, 1) * scale, all_points_set_1(:, 2) * scale, 5, 'r')
%         hold on; scatter(all_points_set_2(:, 1) * scale + size(left_image, 2) * scale, all_points_set_2(:, 2) * scale, 5, 'r')
%         
        Defined_key_points_num = min(size(all_points_set_1, 1), size(all_points_set_2, 1));
        key_point_camera_1 = suture_key_points_aligment(all_points_set_1, Defined_key_points_num);
        key_point_camera_1_converse = suture_key_points_aligment(all_points_set_1(end: -1: 1, :), Defined_key_points_num);

        key_point_camera_2 = suture_key_points_aligment(all_points_set_2, Defined_key_points_num);
        
%--------------------------------------------------------------------------
%----------------------------- ICP TEST -----------------------------------
        [TR, TT] = icp_multi_dim(key_point_camera_1,key_point_camera_2); 
        
        RTdata = TR * key_point_camera_2' + repmat(TT, 1, size(key_point_camera_2, 1));
        RTdata = RTdata';
        %{
        figure; scatter(key_point_camera_1(:, 1), key_point_camera_1(:, 2),'r'); hold on; 
        scatter(key_point_camera_2(:, 1), key_point_camera_2(:, 2), 'b'); hold on; 
        scatter(RTdata(:, 1), RTdata(:, 2), 'y');
        %}
%--------------------------------------------------------------------------

       
        
% ------------------------ 3D coordinates computations --------------------
        [xyz_tem, re_projection_error] = triangulate(key_point_camera_1, key_point_camera_2, stereoParams);
        %xyz = xyz_tem * [cos(pi) -sin(pi) 0; sin(pi) cos(pi) 0; 0 0 1];
        % <-------------dVRK simulator
        
        if (string(system_var) == 'vrep')
            xyz = xyz_tem * [cos(pi) -sin(pi) 0; sin(pi) cos(pi) 0; 0 0 1];
        else
            xyz = xyz_tem;
        end
        
        % -----
        % ------- for IROS 2020 illustration
%         expand_xyz = [];
%         
%         xyz_size = size(xyz, 1);
%         if (xyz_size > 100)
%             expand_xyz = xyz(1:39, :);
%             
%             for ii = 40 : xyz_size - 40
%                 expand_xyz = [expand_xyz; xyz(ii, :)];
%                 n = 1;
%                 append_xyz = [];
%                 while(n < 3) 
%                     aaaa = xyz(ii, :) + 10 * (rand(1,3) * 2 - 1);
%                     append_xyz = [append_xyz; aaaa];
%                     n = n + 1;
%                 end
%                 expand_xyz = [expand_xyz; append_xyz];
%                 ii = ii + 1;
%             end
%             
%         end
%         xyz = expand_xyz;
%         Defined_key_points_num = size(xyz, 1);
        % ------- for IROS 2020 illustration
        
% -------------------------------------------------------------------------
        
        End_point_gap_threshold = 1.5; % it is tunable parameter --> according to scale
        End_point_neighbour_num = 4;

        for i_begin_node = 1 : Defined_key_points_num

            dis_i =  sqrt(sum((xyz - xyz(i_begin_node, :)) .^ 2, 2));
            marker = size(find(dis_i < End_point_gap_threshold), 1);

            if (marker > End_point_neighbour_num)
                break;
            end

        end
        for j_end_node = Defined_key_points_num : -1 : 1

            dis_i =  sqrt(sum((xyz - xyz(j_end_node, :)) .^ 2, 2));
            marker = size(find(dis_i < End_point_gap_threshold), 1);

            if (marker > End_point_neighbour_num)
                break;
            end

        end

        Element_weight_1 = [];
        for ii = 1 : Defined_key_points_num
            dis_i =  sqrt(sum((xyz - xyz(ii, :)) .^ 2, 2));
            near_element(ii, 1) = size(find(dis_i < End_point_gap_threshold), 1);
            Element_weight_1(ii, 1) = 1 / near_element(ii, 1); %exp(near_element(ii, 1) -  mean(near_element));
        end

        Dis_coefficient = bsxfun(@times, Element_weight_1, Element_weight_1');

% .........................................................................
% ----------------------Give larger penalty to two elements----------------
% ----------------------whose number lists are too far from----------------
% ----------------------each other.----------------------------------------

        Row_Row = repmat(linspace(1, Defined_key_points_num, Defined_key_points_num), Defined_key_points_num, 1);
        Col_Col = repmat(linspace(1, Defined_key_points_num, Defined_key_points_num)', 1, Defined_key_points_num);
        Row_Col_diff = abs(Row_Row - Col_Col);
        Penalty_list = 1 + (exp(Row_Col_diff/20 - 2));

        
% -------------------------------------------------------------------------
% --------------------find propoer path length threshold-------------------
    
        valid_path_length_maximum_length = 8;
        L = Inf;
        while (L == Inf)
            d = pdist(xyz);
            valid_path_length_maximum_length = valid_path_length_maximum_length + 0.5;
            list = find(d > valid_path_length_maximum_length); % !!!with the increase of scale, we should decrese with value for more accurate computation!!!

            d(list) = 0;
            % Convert to Adjacency matrix
            d_Adjacency = squareform(d);

            hyper_d = d_Adjacency .* Dis_coefficient.* Penalty_list;
            % Generate Graph object and store XYZ coordinates in it
            G_3D_length = graph(hyper_d);
            G_3D_length.Nodes = array2table(xyz, 'VariableNames', {'X', 'Y', 'Z'});
            % Calculate shortest path between node 1 and end
            [P_3D_length, L] = shortestpath(G_3D_length, i_begin_node, j_end_node);
        end
        
        if (nargin > 4)
            figure(3); hold off; view(45, 45);
            H_3D_length = plot(G_3D_length, 'XData', G_3D_length.Nodes.X, 'YData', G_3D_length.Nodes.Y, 'ZData', G_3D_length.Nodes.Z);
            highlight(H_3D_length, P_3D_length, 'EdgeColor', 'r', 'LineWidth', 6);
            disp(L); grid on;
        end
% -------------------------------------------------------------------------
        updated_nodes = xyz(P_3D_length, :);
        xyzsg = dVRK_suture_thread_smoothing(updated_nodes);

        xyzsg = xyzsg * 0.001;
        length_optimize = 0;
        for i = 1 : size(xyzsg, 1) - 1
            length_optimize = norm(xyzsg(i + 1, :) - xyzsg(i, :)) + length_optimize;
        end
        display(length_optimize * 1000);
        

% -------------------------------------------------------------------------
        %[t1_RCM_vision, t2_RCM_vision, xyzco_RCM_vision] = dVRK_get_transformation_and_coordinates('Vision_sensor_left', 'J1_ECM', xyzsg);

        %[t1_RCM_leftArm, t2_RCM_leftArm, xyzco_RCM_leftArm] = dVRK_get_transformation_and_coordinates('Vision_sensor_left', 'J1_PSM1', xyzsg);
        
        %
        %{
        figure(3); hold off;
        plot3(xyzsg(:, 1), xyzsg(:, 2), xyzsg(:, 3), 'r', 'LineWidth', 7);
        xlabel('X/mm', 'fontsize', 25, 'fontweight', 'bold');
        ylabel('Y/mm', 'fontsize', 25, 'fontweight', 'bold'); 
        zlabel('Z/mm', 'fontsize', 25, 'fontweight', 'bold');
        %xlim([-0.05 0.025]); ylim([-0.02 0.04]); zlim([0.08 0.14]);
        view(-0, 0);
        grid on; box on;

        figure(4); hold off;
        plot3(xyzsg(:, 1), xyzsg(:, 2), xyzsg(:, 3), 'r', 'LineWidth', 7);
        xlabel('X/mm', 'fontsize', 25, 'fontweight', 'bold');
        ylabel('Y/mm', 'fontsize', 25, 'fontweight', 'bold'); 
        zlabel('Z/mm', 'fontsize', 25, 'fontweight', 'bold');
        %xlim([-0.05 0.025]); ylim([-0.02 0.035]); zlim([0.08 0.15]); 
        view(-90, 90);
        grid on; box on;
        %}
        
        %multi_view_coordinates_RCM_vision{multi_view_num} = xyzco_RCM_vision;
    else
        disp('Error Happened During Semgmentation')
        
    end
end