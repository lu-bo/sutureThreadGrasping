function [initialRowColumn, contour_candidates] = sutureContourWrapping_TMECH_revision(Pattern)
    %======================================================================
    % edge pattern should be calculated from the binary pattern
    %======================================================================
    %edgePattern=edgeupdate;
%%
%==========================================================================
% for test use 2018.01.11
%  clear all;
%  close all;
%  biParameter=0.47;
%  
%  cc=2;
%  str1='C:\Users\LU Bo\Desktop\PhD Studies\Semester 5\Suture Detection\Test Suture Tip\';
%  origin=imread([str1,num2str(cc),'.bmp']);
%  edgePattern=rgb2gray(origin);
%  edgePattern=imbinarize(edgePattern,biParameter);
%  edgePattern=contourSearching(edgePattern);
%  figure;
%  imshow(edgePattern);
%==========================================================================
%edgePattern=contourSearching(Pattern);
%%  
    %Pattern = Pattern * (-1) + 1;
    % Pattern = edge_pattern_image_1;
    
    edgePattern = Pattern * (-1) + 1;
        
    edgeLocation = [];
    %%
    %tic
    L1 = size(edgePattern, 1); L2 = size(edgePattern, 2);
    
    initialRowColumn = []; 
    [x, y] = find(edgePattern == 1);
    edgeLocation = [x, y];
    
    boundary_num = [find(edgeLocation(:, 1) == 1); find(edgeLocation(:, 1) == L1); find(edgeLocation(:, 2) == 1); find(edgeLocation(:, 2) == L2)];
    boundary_num = unique(boundary_num);
    initialRowColumn = edgeLocation(boundary_num, :);
    %tCom = toc; disp(['Computational Time of Loop is: ' num2str(tCom)]); 
    %%
    %{
    tic 
    row = 1;
    for i = 1 : size(edgePattern, 1)
        for j = 1 : size(edgePattern, 2)
% =========================================================================
% ----- use "edgeLocation" to store the contour pixel coordinates, and the
% ----- connecting pixels to the current one

            if (edgePattern(i, j) == 1)
               edgeLocation(row, 1) = i; 
               edgeLocation(row, 2) = j; 
%==========================================================================               
               
               %{
               for local_I = -1: 1: 1
                   for local_J = -1: 1: 1
                       
                       localRow = i + local_I;
                       localCol = j + local_J;
                       if (localRow >= 1 && localRow <= size(edgePattern, 1) && ...
                           localCol >= 1 && localCol <= size(edgePattern, 2) && ...
                           edgePattern(localRow, localCol) == 1)
                           edgeLocation(row, 3) = edgeLocation(row, 3) + 1; 
                       end
                       
                   end
               end
               %}
               row = row + 1;
%==========================================================================
            end
                
        end
    end
    tCom = toc; disp(['Computational Time of Loop is: ' num2str(tCom)]);  
    %}
%==========================================================================
% contour configuration wrapping (CCW)
%==========================================================================
% initial x or y - figure our value
%==========================================================================
%   
    %{
    initialRowColumn = [];
    for i = 1 : size(edgeLocation, 1)
        if (edgeLocation(i , 1) == 1 || edgeLocation(i , 1) == size(edgePattern, 1) || ...
            edgeLocation(i , 2) == 1 || edgeLocation(i , 2) == size(edgePattern, 2))
            
            initialRowColumn = [initialRowColumn ; edgeLocation(i, 1) , edgeLocation(i, 2)];
        
        end
    end
    %}
%%
% =========================================================================
% ----- position wrapping / most important (edgePattern)
% ----- TMECH revision improvement
    %tic
    contour_candidates = [];
    
    for i = 1 : size(initialRowColumn, 1)
        currentRowCol = contour_sequence_mapping_2019_Dec_update(initialRowColumn(i, :), edgePattern);
        %
        iteration_step = 6;
        iteration = floor(size(currentRowCol, 1) / iteration_step);
        selected_tip_candidate = [];
        
        if size(currentRowCol, 1) > iteration_step * 2
        
            num = 1;
            max_variation = 0;
            
            for i_iteration = 2 : iteration
                node_1 = iteration_step * (i_iteration - 2) + 1;
                node_2 = iteration_step * (i_iteration - 1) + 1;
                node_3 = iteration_step * (i_iteration)     + 1;
            
                if (node_3 > size(currentRowCol, 1))
                    node_3  = size(currentRowCol, 1);
                end
            
                vect_1 = currentRowCol(node_2, :) - currentRowCol(node_1, :);
                vect_2 = currentRowCol(node_3, :) - currentRowCol(node_2, :);
            
                x1 = vect_1(1 , 1); y1 = vect_1(1 , 2);
                x2 = vect_2(1 , 1); y2 = vect_2(1 , 2);
            
                variation(num , :) = abs(atan2d(x1*y2-y1*x2,x1*x2+y1*y2));
                        
                if (variation(num , :) > max_variation)
                    max_variation = variation(num , :);
                    selected_tip_candidate = currentRowCol(node_2, :);
                end
            
                num = num + 1;
            end
        end
        %hold on; scatter(currentRowCol(:, 2) , currentRowCol(:, 1), 15 , 'r');
        %%
        %contour_candidates(i , :) = [currentRowCol(floor(size(currentRowCol, 1) / 2), 1), ...
                                %currentRowCol(floor(size(currentRowCol, 1) / 2), 2)];
              
        if (size(selected_tip_candidate, 1) ~= 0)
            contour_candidates(i , :) = selected_tip_candidate(1, :);
        end
        
    end
    %tCom = toc; disp(['Computational Time of Loop is: ' num2str(tCom)]); 
    %{
    figure; imshow(edgePattern);
    hold on; scatter(contour_candidates(: , 2) , contour_candidates(: , 1) , 10 , 'r')
    %}
%%
    
    %{
    %=====================================================================
    %added part - 2018-01-11
    suture_begin = currentRowCol(1, :);
    suture_end   = currentRowCol(size(currentRowCol, 1), :);
    
    length_begin_middle =norm(suture_begin - localPosition);
    length_end_middle   =norm(suture_end   - localPosition);
    parameter1 = 1;
    parameter2 = 1;
    parameter_bias = 0.8;
    bias = round(parameter_bias * size(currentRowCol, 1) * ...
                 (parameter1 * length_begin_middle - parameter2 * length_end_middle) / ...
                 (parameter1 * length_begin_middle + parameter2 * length_end_middle));
%     renewed_tip=[currentRowCol(floor(size(currentRowCol,1)/2)+bias,1),...
%         currentRowCol(floor(size(currentRowCol,1)/2)+bias,2)];
    
%      figure;
%      imshow(edgePattern);
%      hold on;
%      scatter(localPosition(2),localPosition(1),20);
%      hold on;
%      scatter(renewed_tip(2),renewed_tip(1),10);
     
     %=====================================================================
     %added part - 2018-01-11
    suture_contour_length = 0;
    for number_counter = 2 : size(currentRowCol, 1)
        suture_contour_length(number_counter, :) = ...
        suture_contour_length(number_counter - 1, :) + norm(currentRowCol(number_counter, :) - currentRowCol(number_counter-1, :));
    end
     
    half_length = suture_contour_length(number_counter) / 2;
     
    judge_deviation = half_length * 2;
    pixel_number = [];
    for search_tip = 1 : size(currentRowCol, 1)
        current_deviation = abs(suture_contour_length(search_tip) - half_length);
        if (current_deviation < judge_deviation)
            judge_deviation = current_deviation;
            pixel_number = search_tip;
        end
    end
     
    pixel_number = pixel_number + bias;
     
    tip_location_by_length = [currentRowCol(pixel_number, 1), currentRowCol(pixel_number, 2)];

    % -----                       Outputs                             -----
    tipLocation = [tip_location_by_length(2), tip_location_by_length(1)];
    WrappingLocation = currentRowCol;
    %}
end
    
    
  