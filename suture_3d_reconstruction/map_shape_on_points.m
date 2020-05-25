function overall_points_set = map_shape_on_points(POINT_SEQUENCE)
    
% ----- "POINT_SEQUENCE" = format{col(x) , row(y)}.                   -----
% ----- ============================================================= -----
% ----- NOTE: make sure the uniqueness of the output matrix, and to ensure
% ----- that all elements follow the sequence from the tip to the end, I 
% ----- search the next element between the two current key points from the
% ----- last current point. 
% ----- Be careful of the negative values, the loop value should also be
% ----- negative 1 to negative the last value.

    POINT_SEQUENCE = round(POINT_SEQUENCE);
    POINT_SEQUENCE = unique(POINT_SEQUENCE,'rows','stable');
    % ----- add the first key point -----
    overall_points_set = POINT_SEQUENCE(1 , :);
    %%
    for sequence = 2 : size(POINT_SEQUENCE , 1)
        
        current_poisition = POINT_SEQUENCE(sequence , :);
        last_position = POINT_SEQUENCE(sequence - 1 , :);
        
        
        % ----- width and height variation                            -----
        w = current_poisition(1) - last_position(1);
        h = current_poisition(2) - last_position(2);
        slop = h / w ;
        
        % ----- add all pixels between current and last points to set -----
        if (abs(slop) >= 1)
            for inner_y = h/abs(h) * 1 : h/abs(h) : h - (h/abs(h) * 1)
                add_x = last_position(1) + round (inner_y / slop);
                add_y = last_position(2) + inner_y;
                overall_points_set = [overall_points_set ; add_x , add_y];
            end
        else
            for inner_x = w/abs(w) * 1 : w/abs(w) : w - (w/abs(w) * 1)
                add_x = last_position(1) + inner_x;
                add_y = last_position(2) + round (inner_x * slop);
                overall_points_set = [overall_points_set ; add_x , add_y];
            end
        end
        
        % ----- add the current key point -----
        overall_points_set = [overall_points_set ; current_poisition];
        %{
        scatter(overall_points_set(sequence,1),overall_points_set(sequence,2),'r');
        hold on;
        pause;
        %}
    end
    %%
    
    % ----- unique all the elements in the output -----
    %overall_points_set = unique(overall_points_set , 'rows');
    
end