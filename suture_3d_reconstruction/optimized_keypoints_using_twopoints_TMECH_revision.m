function [locatingSutureSegment,relative_size]=optimized_keypoints_using_twopoints_TMECH_revision(last_point, current_point_to_refine, edge_image, Reversed_biOutIm1)
    % This algorithm shifts the point to the center of an edge pattern.
    % Input requirement: 1. the point "current_point_to_refine" with the
    % format: [x(col), y(row)]; 2. a edge pattern, the background is the
    % black color(0), and the edge is the white color(1). the edge should
    % be white color
    
    % define the half length of the searching area, and crop it. 
    
    % debug initialize
    % last_point = [selected_k_p(i - 1, 2), selected_k_p(i - 1, 1) ]; current_point_to_refine = [selected_k_p(i, 2), selected_k_p(i, 1) ]; edge_image = next_loop_edge_pattern;
    
    %%
    search_area_length = 20;
    search_area_width  = 20;
    rect_search_area = [current_point_to_refine(1) - search_area_length, ...
                        current_point_to_refine(2) - search_area_width , ...
                        2 * search_area_length, 2 * search_area_width];
    edge_search_area = imcrop(edge_image, rect_search_area);
    edge_search_area_out_of_zone = imcrop(Reversed_biOutIm1, rect_search_area);
    %figure(10); hold off; imshow(edge_search_area); hold on; scatter(last_point(1), last_point(2), 20, 'r'); hold on; scatter(current_point_to_refine(1), current_point_to_refine(2), 20, 'b'); 
%%
    %figure; imshow(edge_search_area); hold on;
    slop = -1 / ((current_point_to_refine(2) - last_point(2)) / ...
           (current_point_to_refine(1) - last_point(1)));

    edgeNumber1 = 0;
    contourLocation = [];
    maxThreshold = search_area_length / 1;  %Adjust value

    if (abs(slop) >= 1)
        for edgeJ1 = 1 : size(edge_search_area, 1)
            
            
            edgeI1 = round((1 / slop) * (edgeJ1 - size(edge_search_area, 1) / 2) + size(edge_search_area , 2) / 2);
            if (edgeI1 <= size(edge_search_area, 2) && edgeI1 >= 1)
                %scatter(edgeI1,edgeJ1, 1,'b'); hold on;
                
                deviation = norm(edgeJ1 - size(edge_search_area, 1) / 2) + norm(edgeI1 - size(edge_search_area, 1) / 2);
                if (edge_search_area(edgeJ1, edgeI1) == 1 && deviation < maxThreshold)
                    edgeNumber1 = edgeNumber1 + 1;
                    contourLocation(edgeNumber1, 1) = edgeI1;
                    contourLocation(edgeNumber1, 2) = edgeJ1;
                end
            end
            
            %
            edgeI11 = round((1 / slop) * (edgeJ1 - size(edge_search_area, 1) / 2) + size(edge_search_area, 2) / 2) + 1;
            if (edgeI11 <= size(edge_search_area, 2) && edgeI11 >= 1)            
                %scatter(edgeI11,edgeJ1, 1,'b'); hold on;
                
                deviation = norm(edgeJ1 - size(edge_search_area, 1) / 2) + norm(edgeI11 - size(edge_search_area, 1) / 2);
                if (edge_search_area(edgeJ1, edgeI11) == 1 && deviation < maxThreshold)
                    edgeNumber1 = edgeNumber1 + 1;
                    contourLocation(edgeNumber1, 1) = edgeI11;
                    contourLocation(edgeNumber1, 2) = edgeJ1;
                end
            end
            %{
            edgeI12 = round((1 / slop) * (edgeJ1 - size(edge_search_area, 1) / 2) + size(edge_search_area, 2) / 2) - 1;
            if (edgeI12 <= size(edge_search_area, 2) && edgeI12 >= 1)            
                scatter(edgeI12,edgeJ1, 1,'b');
                hold on;
                deviation = norm(edgeJ1 - size(edge_search_area, 1) / 2) + norm(edgeI12 - size(edge_search_area, 1) / 2);
                if (edge_search_area(edgeJ1, edgeI12) == 1 && click == 0 && deviation < maxThreshold)
                    contourLocation(edgeNumber1, 1) = edgeI12;
                    contourLocation(edgeNumber1, 2) = edgeJ1;
                    click = click + 1;
                    edgeNumber1 = edgeNumber1 + 1;
                end
            end
            %}
        
        end
    end
    

    if (abs(slop) < 1)
        for edgeI1 = 1 : size(edge_search_area, 2)
            %click = 0;
            edgeJ1 = round(slop * (edgeI1 - size(edge_search_area, 2) / 2) + size(edge_search_area, 1) / 2);
            if (edgeJ1 <= size(edge_search_area, 1) && edgeJ1 >= 1)
               % scatter(edgeI1,edgeJ1, 1,'b'); hold on;
                
                deviation = norm(edgeJ1 - size(edge_search_area, 1) / 2) + norm(edgeI1 - size(edge_search_area, 1) / 2);
                if (edge_search_area(edgeJ1, edgeI1) == 1 && deviation < maxThreshold)
                    edgeNumber1 = edgeNumber1 + 1;
                    contourLocation(edgeNumber1,1) = edgeI1;
                    contourLocation(edgeNumber1,2) = edgeJ1;
                    %click = click + 1;
                end
            end
        
            %
            edgeJ11 = round(slop * (edgeI1 - size(edge_search_area, 2) / 2) + size(edge_search_area, 1) / 2) - 1;
            if (edgeJ11 <= size(edge_search_area, 1) && edgeJ11 >= 1)
                % scatter(edgeI1,edgeJ11, 1,'b'); hold on;
                
                deviation = norm(edgeJ11 - size(edge_search_area, 1) / 2) + norm(edgeI1 - size(edge_search_area, 1) / 2);
                if (edge_search_area(edgeJ11, edgeI1) == 1 && deviation < maxThreshold)
                    edgeNumber1 = edgeNumber1 + 1;
                    contourLocation(edgeNumber1, 1) = edgeI1 ;
                    contourLocation(edgeNumber1, 2) = edgeJ11;
                    %click = click + 1;
                end
            end
            %{
            edgeJ12 = round(slop * (edgeI1 - size(edge_search_area, 2) / 2) + size(edge_search_area, 1) / 2) + 1;
            if (edgeJ12 <= size(edge_search_area, 1) && edgeJ12 >= 1)
                scatter(edgeI1,edgeJ11, 1,'b');
                hold on;
                deviation = norm(edgeJ12 - size(edge_search_area, 1) / 2) + norm(edgeI1 - size(edge_search_area, 1) / 2);
                if (edge_search_area(edgeJ12, edgeI1) == 1 && click == 0 && deviation < maxThreshold)
                    contourLocation(edgeNumber1, 1) = edgeI1 ;
                    contourLocation(edgeNumber1, 2) = edgeJ12;
                    click = click + 1;
                    edgeNumber1 = edgeNumber1 + 1;
                end
            end
            %}

        end
    end
    %figure; imshow(edge_search_area); hold on; scatter(contourLocation(:, 1), contourLocation(:, 2), 5, 'r');
    %figure; imshow(edge_image); hold on; scatter(last_point(1, 1), last_point(1, 2), 10, 'b'); hold on; scatter(current_point_to_refine(1, 1), current_point_to_refine(1, 2), 10, 'g');
    
    
    %%
    % evaluation
    dis_thre_bottomline = 3;
    if (size(contourLocation, 1) >= 2)
        %updatedContourLocation = contourLocation(1, :);
        
        %
        % ----------------revised part 2019.07.05-----------------
        % ----------------for better robustness-------------------
        %if (size(updatedContourLocation, 1) > 2)
        base_num = size(contourLocation, 1);
        list_vectors = (vecnorm((contourLocation - [search_area_width, search_area_length])'));
        picked_num_initial_1 = find(list_vectors == min(list_vectors));

        initial_size = size(picked_num_initial_1, 2);
        % in case, there exists two elements, which obtains the same
        % distance to the center.
        picked_num_1 = picked_num_initial_1(1, 1);
        picked_element_1 = contourLocation(picked_num_1, :);

        contourLocation(picked_num_initial_1, :) = [];
        
        dis_list_to_center = vecnorm((contourLocation - picked_element_1)');
        filtered_list = find(dis_list_to_center < dis_thre_bottomline);
        filtered_num = size(filtered_list, 2);
        contourLocation(filtered_list, :) = [];
               
        remaining_num = base_num - initial_size - filtered_num;
        
        if (remaining_num > 0 )
            picked_num_2 = [];

            for ii = 1 : remaining_num
                out_pixel_nums(ii, 1) = out_of_body_number(contourLocation(ii, :), picked_element_1, edge_search_area_out_of_zone); 
            end

            picked_num_initial_2 = find(out_pixel_nums == min(out_pixel_nums));
            picked_num_2 = picked_num_initial_2(1, 1);
            picked_element_2 = contourLocation(picked_num_2, :);

            updatedContourLocation = [picked_element_1; picked_element_2];
        %end
        
        
        % ----------------revised part 2019.07.05-----------------
        %
        
            locatingSutureSegment(1) = mean(updatedContourLocation(:, 1));
            locatingSutureSegment(2) = mean(updatedContourLocation(:, 2));
            relative_size = rect_search_area;
        else
            locatingSutureSegment(1) = size(edge_search_area, 2) / 2;
            locatingSutureSegment(2) = size(edge_search_area, 1) / 2;
            relative_size = rect_search_area;
        end
            
    else
        locatingSutureSegment(1) = size(edge_search_area, 2) / 2;
        locatingSutureSegment(2) = size(edge_search_area, 1) / 2;
        relative_size = rect_search_area;
    end
    
    locatingSutureSegment = round(locatingSutureSegment);
    %hold on; scatter(locatingSutureSegment(1), locatingSutureSegment(2), 3, 'r');
    
    % figure; imshow(edge_search_area); hold on; scatter(locatingSutureSegment(1), locatingSutureSegment(2),15,'b')
    %pause(1); close;
end

