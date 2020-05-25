function [all_points_set, suture_tip1]  = DL_based_intersected_suture_thread_detection_with_clicks(Image1_origin, Segmented_image_name, Previous_tip)

    % debug initialization
    % Image1_origin = left_image; Segmented_image_name = 'segmented_suture_l.jpg';
    
    max_row = size(Image1_origin, 1);
    max_col = size(Image1_origin, 2);
     
    [edgeIm1, contours_totoal_number, detected_contours] = DL_based_contour_segmentation(Segmented_image_name, max_row, max_col);
    %edgeIm2 = DL_based_contour_segmentation('segmented_suture_r.jpg', max_row, max_col);
    biOutIm1 = imread(Segmented_image_name);
    biOutIm1 = double(biOutIm1 / max(biOutIm1(:)));
    %figure; imshow(biOutIm1);
    %montage([edgeIm1,edgeIm2]);

    %%
    %tCom = toc; disp(['Computational Time before click is: ' num2str(tCom)]);
    if (nargin < 3)
        figure; imshow(Image1_origin,'Border','tight'); hold on; title('Initial indication of suture tip'); 
        suture_tip_point_image_1 = getrect; close;
    else
        suture_tip_point_image_1 = Previous_tip;
    end
        
    %tCom = toc; disp(['Computational Time after click is: ' num2str(tCom)]);
    %tic
    % ----- define the size centered by the clicked suture tip -----
    suture_area_length = 20; suture_area_width = 20;
       
    % =========================================================================
    % in image 1 and 2, make iterative pattern
    % =========================================================================
    
    suture_local_area_image1 = round([...
        suture_tip_point_image_1(1) - suture_area_length, ...
        suture_tip_point_image_1(2) - suture_area_width,  ...
        2 * suture_area_length, 2 * suture_area_width]);
    %edgeIm1 = edgeIm1 * (-1) + 1;
    local_pattern_camera1 = imcrop(edgeIm1, suture_local_area_image1);
    % figure;imshow(local_pattern_camera1)
    %tCom = toc; disp(['Computational Time of Loop 1 is: ' num2str(tCom)]);
    %%
    %tic
    suture_tip1 = [];

    suture_tip1 = tip_detection_TMECH(local_pattern_camera1, suture_local_area_image1, edgeIm1);
    if (size(suture_tip1, 1) == 0)
        display('Tip_missing');
        suture_tip1 = Previous_tip;
        all_points_set = [];
        return;
    else
        suture_tip1 = [suture_tip1(2), suture_tip1(1)];
    end
    % figure; imshow(Image1_origin); hold on; scatter(suture_tip1(1), suture_tip1(2), 80, 'r');
    %tCom = toc; disp(['Computational Time of Loop out is: ' num2str(tCom)]);
    %%
    
    min_dis = 100000;
    selected_contour_num = [];
    for contour_single_number = 1 : contours_totoal_number
        data = double(py.array.array('d',py.numpy.nditer(detected_contours{contour_single_number}))); % d is for double, see link below on types
        data = reshape(data,[2 detected_contours{contour_single_number}.size/2])'; % Could incorporate x.shape here ...
        current_min_dis_to_tip = min(vecnorm((data - suture_tip1)'));
        if (current_min_dis_to_tip < min_dis)
            min_dis = current_min_dis_to_tip;
            selected_contour_num = contour_single_number;
        end  
    end
    contour_members1 = double(py.array.array('d',py.numpy.nditer(detected_contours{selected_contour_num})));
    contour_members1 = reshape(contour_members1,[2 detected_contours{selected_contour_num}.size/2])';
    switch_value = contour_members1(:, 2);
    contour_members1(:, 2) = contour_members1(:, 1);
    contour_members1(:, 1) = switch_value;

    %hold on; scatter(contour_members1(:, 2), contour_members1(:, 1), 3, 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');
    %pause(1); close;
    %%
    % Delete elements which are detected as contour members, but located on the
    % boundary. The criteria should include the elements that are 1 pixel away
    % from the boudary.
    
    del_num_row_min = find(contour_members1(:, 1) <= 1); del_num_row_max = find(contour_members1(:, 1) >= max_row - 1);
    del_num_col_min = find(contour_members1(:, 2) <= 1); del_num_col_max = find(contour_members1(:, 2) >= max_col - 1);
    contour_members1([del_num_row_min; del_num_row_max; del_num_col_min; del_num_col_max], :) = [];

    % Select elements with a specific interval
    clear updateKeyPointsImage1 updateKeyPoints1 patternEdgeCorner1;
    inter = 6; % indicate the intervals of key points selection along the segmented suture
    selected_k_p = contour_members1(1 : inter : end, :); Reversed_biOutIm1 = biOutIm1 * (-1) + 1;
    updateKeyPointsImage1(1, :) = [selected_k_p(1, 2) selected_k_p(1, 1)];
    Loop_num = size(selected_k_p, 1);
    %tCom = toc; disp(['Computational Time of Loop Begin is: ' num2str(tCom)]);
    
    %tic
    %edgeIm1 = gpuArray(edgeIm1);
    next_loop_edge_pattern = edgeIm1 * (-1) + 1;
    
    
    for i = 2 : Loop_num 
        if (selected_k_p(i, 1) < max_row - 1 && selected_k_p(i, 1) > 1 && ...
            selected_k_p(i, 2) < max_col - 1 && selected_k_p(i, 2) > 1 )  

            [updateKeyPoints1(i, :), patternEdgeCorner1(i, :)] = optimized_keypoints_using_twopoints_TMECH_revision([selected_k_p(i - 1, 2), selected_k_p(i - 1, 1) ], ...
                                                                 [selected_k_p(i, 2), selected_k_p(i, 1)], next_loop_edge_pattern, Reversed_biOutIm1);
            
            if (patternEdgeCorner1(i, 1) < 0)
                updateKeyPointsImage1(i, :) = ([updateKeyPoints1(i, 1), ...
                                              updateKeyPoints1(i, 2) + patternEdgeCorner1(i, 2)] - 1);
            else
                if (patternEdgeCorner1(i, 2) < 0)
                    updateKeyPointsImage1(i, :) = ([updateKeyPoints1(i, 1) + patternEdgeCorner1(i, 1) - 1, ...
                                                  updateKeyPoints1(i, 2)]);
                else
                    updateKeyPointsImage1(i, :) = ([updateKeyPoints1(i, 1) + patternEdgeCorner1(i, 1), ...
                                                  updateKeyPoints1(i, 2) + patternEdgeCorner1(i, 2)] - 1);
                end
            end
        end
        % figure(9); hold off; imshow(edgeIm1); hold on; scatter(updateKeyPointsImage1(i, 1), updateKeyPointsImage1(i, 2), 3, 'r'); pause; 
        
    end
    updateKeyPointsImage1 = [[selected_k_p(1 , 2) selected_k_p(1 , 1)] ; updateKeyPointsImage1];
    %tCom = toc; disp(['Computational Time of Loop Internal is: ' num2str(tCom)]);
    
    
    % ----- delect the number that are out of the image area              -----
    [delete_1, ~] = find(updateKeyPointsImage1(: , 1) < 1);
    [delete_2, ~] = find(updateKeyPointsImage1(: , 2) < 1);
    [delete_3, ~] = find(updateKeyPointsImage1(: , 1) > max_col);
    [delete_4, ~] = find(updateKeyPointsImage1(: , 2) > max_row);
    
    delete_set = [delete_1; delete_2; delete_3; delete_4];
    delete_set = unique(delete_set);
    updateKeyPointsImage1(delete_set, :) = [];
    %tCom = toc; disp(['Computational Time after 1st points adjustment is: ' num2str(tCom)]);
    %{
    i = 1; loop_num = size(updateKeyPointsImage1 , 1);
    while (i <= loop_num)
        if (updateKeyPointsImage1(i , 1) < 1 || updateKeyPointsImage1(i , 1) > max_col || ...
            updateKeyPointsImage1(i , 2) < 1 || updateKeyPointsImage1(i , 2) > max_row)
            updateKeyPointsImage1(i , :) = [];
            loop_num = loop_num - 1;
        end
        i = i + 1;
    end
    %}
    %figure; imshow(edgeIm1) ; hold on; scatter(updateKeyPointsImage1(: , 1), updateKeyPointsImage1(: , 2) , 5 , 'r')
    %%
    % set a tolerence parameter and remove the refined key points which are out
    % of the suture thread's body.
    %range = 4;
    %tolerance = range * range * 0.8;
    optimized_keypoints = [];
    for i = 1 : size(updateKeyPointsImage1, 1)
        %out_of_boundary(i , 1) = 0;
        %for near_row = - range : 1 : range
            %for near_col = - range : 1 : range
                if (Reversed_biOutIm1(updateKeyPointsImage1(i , 2), updateKeyPointsImage1(i , 1)) ~= 0)
                    optimized_keypoints = [optimized_keypoints ; updateKeyPointsImage1(i , :)];
                end
            %end
        %end
    
        %if (out_of_boundary(i , 1) <= tolerance)
            %optimized_keypoints = [optimized_keypoints ; updateKeyPointsImage1(i , :)];
        %end
    end
    updateKeyPointsImage1 = optimized_keypoints; updateKeyPointsImage1 = [suture_tip1; updateKeyPointsImage1];
    %figure ; imshow(Reversed_biOutIm1); hold on; scatter(updateKeyPointsImage1(: , 1) , updateKeyPointsImage1(: , 2) , 5 , 'r')
    %% 
    % -----        creterias usded to get "TRIM POINTS" set.              -----
    % ----- ============================================================= -----
    % ----- creteria one: trim the points which are too close to others.  -----
    %updateKeyPointsImage1 =  [selected_k_p(: , 2) , selected_k_p(: , 1)];
    
    %{
    TRIMMED_POINT = updateKeyPointsImage1(1 , :);

    POINT_SEQUENCE_FINAL = [];
    FINAL_ELEMENT_NUM = 0;
    % ---- important parameters ------
    % ---------------------------------------------------------------------
  
    THRE = 5;
    RADIUS = THRE * 8; % set evaluation range within all the trimmed points
    thre_out_of_zone = round(RADIUS / 3);
    thre_angle_variation = 120;
    
    for i = 2 : size(updateKeyPointsImage1, 1)

        SIG = 0;
        for ID = 1 : size(TRIMMED_POINT, 1)
            DIST = norm(updateKeyPointsImage1(i, :) - TRIMMED_POINT(ID, :));
            if (DIST < THRE)
                SIG = 1;
                break;
            end
        end

        if (SIG == 0)
            TRIMMED_POINT = [TRIMMED_POINT ; updateKeyPointsImage1(i, :)];
        end
    end

    TRIMMED_POINT = round(TRIMMED_POINT);
    % figure; imshow(Reversed_biOutIm1); hold on; %scatter(TRIMMED_POINT(: , 1) , TRIMMED_POINT(: , 2) , 10 , 'r')
    %pause; close;

    % -----------------------------------------------------------------
    for angle_variation_group = 1 : 1
        for P1_variation = 1 : 1
            for P2_variation = 1 : 1    
                P1 = 10 * P1_variation;  % out of zone area point
                P2 = 0.2 * P2_variation;    % distance
                P3 = 0.05 * angle_variation_group;

                CURRENT_POINT  = TRIMMED_POINT(1, :);
                POINT_SEQUENCE = []; 
                POINT_SEQUENCE = [POINT_SEQUENCE; CURRENT_POINT]; % initialize the points in new sequence
                LAST_STEP_COORDINATES = suture_tip1;

                LOOP_CANDIDATE = TRIMMED_POINT;
                LOOP_CANDIDATE (1, :) = []; % set the evaluation candidate. remove the first row
                i = 1;

                %{
                figure; imshow(r_biOutIm1);
                hold on; scatter(CURRENT_POINT(1) , CURRENT_POINT(2) , 'r');
                %}

                while (true)
                % for i = 1 : size(TRIMMED_POINT, 1) - 1
                % ----- Construct the two evaluation matrixes, one contains----
                % ----- the coordinates, and the other contains the row    ----
                % ----- number information.                                ----
                STRUCT_EVA_CANDIDATE = [];
                EVA_CANDIDATE = [];
                EVA_CANDIDATE_ROWNUM = [];

                % ----- Picking up the neareset neigbours from the trimmed -----
                % ----- points based on the previous calculations.         -----
                % ----- ================================================== -----
                % ----- We define "LOOP_CANDIDATE" set which is updated in -----
                % ----- each iteration, it remove the newest current point -----
                % ----- that the found points will not be duplicatly       -----
                % ----- evaluated.                                         -----
                for ID = 1 : size (LOOP_CANDIDATE, 1)
                    DIST = norm (LOOP_CANDIDATE(ID, :) - CURRENT_POINT); % distance between all other trimmed the points and the current one

                    % ----- If the point within certain range, which is    -----
                    % ----- defined by "RADIUS", this point is considered  -----
                    % ----- as the evaluation candidates.                  -----
                    if (DIST < RADIUS)
                        EVA_CANDIDATE = [EVA_CANDIDATE; LOOP_CANDIDATE(ID, :)]; % add the candidate to the evaluation set
                        EVA_CANDIDATE_ROWNUM = [EVA_CANDIDATE_ROWNUM; ID]; % record the cooresponding row number
                    end
                end

                if (size(EVA_CANDIDATE, 1) == 0)
                    display('No_candidates');
                    break;
                end

                STRUCT_EVA_CANDIDATE = [EVA_CANDIDATE, EVA_CANDIDATE_ROWNUM];

                % select the next point from the candidates
                CAN_NUM = size(EVA_CANDIDATE, 1); % pick up the number of the next point candidate

                % ----- The algorithm used to pick up the next point from the     -----
                % ----- nearest candidates obtained above .                       -----
                % ----- ========================================================  -----
                % ----- Hierarchical evaluation creterias:                        -----
                % ----- 1. Select the lines that contains the least number of     -----
                % -----    black pixels;                                          -----
                % ----- 2. Select the nearest element.                            -----

                % -----                  Creteria 1                               -----
                OUT_ZONE_NUM = zeros (CAN_NUM, 1);
                for j = 1 : CAN_NUM
                    % ----- find pixel coordinates betewwen two points.           -----
                    % ----- plot coordinate format:                               -----
                    % ----- "EVA_CANDIDATE(j , 1), EVA_CANDIDATE (j , 2)"         -----
                    % -----                  ^                      ^             -----
                    % -----                  |                      |             -----
                    % -----                  x (col)                y (row)       -----
                    w = EVA_CANDIDATE(j, 1) - CURRENT_POINT(1, 1);
                    h = EVA_CANDIDATE(j, 2) - CURRENT_POINT(1, 2);
                    SLOP = h / w;
                    % OUT_ZONE_NUM (j , 1) = 0;
                    % subpoint = [];

                    if (norm (w) >= norm (h))
                       for STEP = w/norm(w) * 1 : w/norm(w) : w

                           x_coordinate = CURRENT_POINT(1, 1) + STEP;
                           y_coordinate = CURRENT_POINT(1, 2) + round(STEP * SLOP);

                           %subpoint = [subpoint; x_coordinate , y_coordinate];

                           if (Reversed_biOutIm1(y_coordinate, x_coordinate) == 0)
                               OUT_ZONE_NUM (j) = OUT_ZONE_NUM (j) + 1;
                           end

                       end
                    end

                    if (norm (w) < norm (h))
                       for STEP = h/norm(h) * 1 : h/norm(h) : h

                           x_coordinate = CURRENT_POINT(1, 1) + round(STEP / SLOP);
                           y_coordinate = CURRENT_POINT(1, 2) + STEP;

                           %subpoint = [subpoint; x_coordinate , y_coordinate];

                           if (Reversed_biOutIm1(y_coordinate, x_coordinate) == 0)
                               OUT_ZONE_NUM (j) = OUT_ZONE_NUM (j) + 1;
                           end

                       end
                    end

                end

                STRUCT_EVA_CANDIDATE = [STRUCT_EVA_CANDIDATE, OUT_ZONE_NUM];

                % -----                  Creteria 2                               -----
                % ----- evaluate the distance between the candidate and the       -----
                % ----- the current point.                                        -----
                DISTANCE_TO_CURRENT_POINT = [];
                for II = 1 : size(STRUCT_EVA_CANDIDATE, 1)
                    DIST = round(norm( STRUCT_EVA_CANDIDATE(II, 1 : 2) - CURRENT_POINT ));

                    DISTANCE_TO_CURRENT_POINT = [DISTANCE_TO_CURRENT_POINT; DIST];
                end

                STRUCT_EVA_CANDIDATE = [STRUCT_EVA_CANDIDATE, DISTANCE_TO_CURRENT_POINT];

                % -----                  Creteria 3                               -----
                % ----- the slop variance should be considered                    -----
                if (i == 1)
                    SLOP_VARIANCE = ones(CAN_NUM, 1);
                end

                if (i > 1)
                    SLOP_VARIANCE = ones(CAN_NUM, 1);
                    for j = 1 : size(STRUCT_EVA_CANDIDATE, 1)
                        CURRENT_VECTOR = [STRUCT_EVA_CANDIDATE(j, 1 : 2) - LAST_STEP_COORDINATES, 0];
                        %CURRENT_SLOP = (STRUCT_EVA_CANDIDATE(j , 2) - LAST_STEP_COORDINATES(2)) / (STRUCT_EVA_CANDIDATE(j , 1) - LAST_STEP_COORDINATES(1));
                        %CURRENT_ANGLE = atan(CURRENT_SLOP) * 180 / pi
                        %SLOP_VARIANCE(j , 1) = abs(CURRENT_ANGLE - LAST_STEP_ANGLE);
                        SLOP_VARIANCE(j, 1) = abs(atan2d(norm(cross(CURRENT_VECTOR, LAST_VECTOR)), dot(CURRENT_VECTOR, LAST_VECTOR)));
                    end
                end

                STRUCT_EVA_CANDIDATE = [STRUCT_EVA_CANDIDATE, SLOP_VARIANCE];

                % -----        set parameters for 3 creterias                     -----
                RANKING_MATRIX = [];

                % P3 = 0.2;  % angle variance 
                for ALLCAN = 1 : size(STRUCT_EVA_CANDIDATE, 1)
                    RANK = P1 * STRUCT_EVA_CANDIDATE(ALLCAN, 4) + P2 * STRUCT_EVA_CANDIDATE(ALLCAN, 5) + exp(P3 * STRUCT_EVA_CANDIDATE(ALLCAN, 6));
                    RANKING_MATRIX = [RANKING_MATRIX; RANK];
                end

                STRUCT_EVA_CANDIDATE = [STRUCT_EVA_CANDIDATE, RANKING_MATRIX];

                MARK = find(STRUCT_EVA_CANDIDATE(:, 7) == min(STRUCT_EVA_CANDIDATE(:, 7)));
                MARK = MARK(1);

                % ----- added criteria: if the minimal number of the out-of-zone  -----
                % ----- element is larger than a threshold, it means all candidate-----
                % ----- are not qualified, and now all candidates should belong   -----
                % ----- to another region, but within the search area we set.     -----
                if (STRUCT_EVA_CANDIDATE(MARK, 4) > thre_out_of_zone)
                    display('Out_of_zone');
                    break;
                end
                
                % STRUCT_EVA_CANDIDATE(MARK, 6)
                if (STRUCT_EVA_CANDIDATE(MARK, 6) > thre_angle_variation)
                    display('Angle_variance');
                    break;
                end

                SELECT_POINT = STRUCT_EVA_CANDIDATE(MARK, 1 : 2);
                SELECT_ROW = STRUCT_EVA_CANDIDATE(MARK, 3);

                LOOP_CANDIDATE (SELECT_ROW, :)=[]; % remove the current point from the loop set using the recorded row number
                CURRENT_POINT = SELECT_POINT; 
                POINT_SEQUENCE = [POINT_SEQUENCE; CURRENT_POINT];

                %CURRENT_SLOP = (CURRENT_POINT(2) - LAST_STEP_COORDINATES(2)) / (CURRENT_POINT(1) - LAST_STEP_COORDINATES(1));
                %CURRENT_ANGLE = atan(CURRENT_SLOP) * 180 / pi;
                %LAST_STEP_ANGLE = CURRENT_ANGLE;
                LAST_VECTOR = [CURRENT_POINT - LAST_STEP_COORDINATES, 0];
                LAST_STEP_COORDINATES = CURRENT_POINT;

                %{
                scatter(CURRENT_POINT(1, 1), CURRENT_POINT(1, 2), 40 , 'r');
                hold on; pause(0.03);
                %}
                i = i + 1; 

                end

                %size(POINT_SEQUENCE, 1) pause(0.5);
                if (size(POINT_SEQUENCE, 1) > FINAL_ELEMENT_NUM)
                    FINAL_ELEMENT_NUM = size(POINT_SEQUENCE, 1);
                    POINT_SEQUENCE_FINAL = [];
                    POINT_SEQUENCE_FINAL = POINT_SEQUENCE;
                end
    % -----------------------------------------------------------------             
            end
        end
    end
    %}
    % ---------------------------------------------------------------------
    % ---------------------------------------------------------------------
    % ---------------------------------------------------------------------           
    POINT_SEQUENCE_FINAL = Ordering_curvilinear_obejcts(updateKeyPointsImage1, suture_tip1, Reversed_biOutIm1);
    %figure; imshow(Reversed_biOutIm1); hold on; scatter(POINT_SEQUENCE_FINAL(: , 1) , POINT_SEQUENCE_FINAL(: , 2) , 20 , 'r')
    
    %%
    %tic
    %POINT_SEQUENCE_FINAL = gpuArray(POINT_SEQUENCE_FINAL);
    all_points_set = map_shape_on_points(POINT_SEQUENCE_FINAL);
    %tCom = toc; disp(['Computational Time before illustration is: ' num2str(tCom)]);
    %%
       
    %all_points_set = gpuArray(all_points_set);
    all_points_set = gather(all_points_set);
    %dc=hsv(length(all_points_set));
    
    %total_point_for_show = size(all_points_set, 1) - 1;
    %line(all_points_set(:, 1), all_points_set(:, 2), 'Color', [1 0 0], 'linewidth', 2);
    
    %for i = 1 : total_point_for_show
    %    line([all_points_set(i, 1) all_points_set(i + 1 , 1)], [all_points_set(i, 2) all_points_set(i + 1, 2)] , 'Color', dc(i, :), 'linewidth', 2);
    %end
    %tCom = toc; disp(['Computational Time of illustration is: ' num2str(tCom)]);
    %figure; imshow(Image1_origin); hold on; hold on; scatter(all_points_set(:, 1) , all_points_set(:, 2) , 2 , 'r')
end