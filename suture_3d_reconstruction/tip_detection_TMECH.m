function [selected_suture_tip_image_1, selected_suture_contour_image_1, tip_of_contour_candidates_1] = tip_detection_TMECH(edge_pattern_image_1, rect_iterative_pattern_1, outline_1)
%%
    % tic
    % edge_pattern_image_1 = local_pattern_camera1; rect_iterative_pattern_1 = suture_local_area_image1; outline_1 = edgeIm1;
    
    % ---------------------------------------------------------------------
    [boundary_point_1 , tip_of_contour_candidates_1] = sutureContourWrapping_TMECH_revision(edge_pattern_image_1);

    if (size(tip_of_contour_candidates_1, 1) == 0)
        selected_suture_tip_image_1 = [];
        return;
    end
    
    tip_of_contour_candidates_in_image_1 = tip_of_contour_candidates_1 + [max(rect_iterative_pattern_1(2) - 1 , 0) , max(rect_iterative_pattern_1(1) - 1 , 0)];
    % -----------------------------------------------[row , col]-------------------------row--------------------------------------------col-------------------
    %
    RoI_contour_figure_1 = edge_pattern_image_1 * (-1) + 1;
    entire_contour_figure_1 = outline_1 *(-1) + 1;

    suture_tip_candidate_coordinates_image_1 = [];
    obejct_contour_total_length_image_1 = [];
    suture_tip_diff_from_clicks_image_1 = [];

    P1_1 = 8;
    P1_2 = 1/20;
    P1_3 = 200;

    min_1 = 10^10;
    Rating_1 = [];
    selected_suture_contour_image_1 = [];
    
    for i = 1 : size(tip_of_contour_candidates_1, 1)

        obejct_contour_image_1 = contour_sequence_mapping_2019_Dec_update(boundary_point_1(i, :), RoI_contour_figure_1);
        entire_contour_image_1 = contour_sequence_mapping_2019_Dec_update(tip_of_contour_candidates_in_image_1(i, :), entire_contour_figure_1);


        suture_tip_diff_from_clicks_image_1(i , :) = norm(tip_of_contour_candidates_1(i , :) - [size(edge_pattern_image_1, 2)/2 , size(edge_pattern_image_1, 1)/2]);
        % -------------------------------------------------------(row , col)---------------------------row--------col----------------------------


        obejct_contour_total_length_image_1(i , :) = size(obejct_contour_image_1 , 1);

        suture_two_end_distance(i , :) = norm(obejct_contour_image_1(end, :) - obejct_contour_image_1(1, :));

        %hold on; scatter(obejct_contour_image_1(: , 2), obejct_contour_image_1(:, 1), 5 , 'r');
        %hold on; scatter(tip_of_contour_candidates_in_image_1(i, 2), tip_of_contour_candidates_in_image_1(i, 1), 8 , 'b');
        %pause;

        Rating_1(i , :) = exp(suture_two_end_distance(i , :)/P1_1 - 1) + P1_2 * (suture_tip_diff_from_clicks_image_1(i , :)) + P1_3 / obejct_contour_total_length_image_1(i , :);

         if (Rating_1(i , :) < min_1)
             min_1 = Rating_1(i , :);
 
             selected_suture_contour_image_1 = [];
             selected_suture_contour_image_1 = entire_contour_image_1;
             selected_suture_tip_image_1 = tip_of_contour_candidates_in_image_1(i , :);
         end
    end
    
    %tCom = toc; disp(['Computational Time of Loop is: ' num2str(tCom)]); 
end

%{
suture_tip_num_in_candidate_image_1 = find(Rating_1 == max(Rating_1));
selected_suture_tip_image_1 = tip_of_contour_candidates_in_image_1(suture_tip_num_in_candidate_image_1(1 , 1), :);
%}

%hold on; scatter(selected_suture_contour_image_1(: , 2) , selected_suture_contour_image_1(: , 1) , 3 , 'r')
%hold on; scatter(selected_suture_tip_image_1(1, 2), selected_suture_tip_image_1(1, 1), 40 , 'MarkerFaceColor','b');