function currentRowCol = contour_sequence_mapping_2019_Dec_update(initialRowColumn_insideFunc, edgePattern)
    % ----- function description:
    % With the input an initial point(row , col) and the contour paterrn,
    % it can figure out the complete contour or a continuous edge.
    
    
    % initialRowColumn_insideFunc = initialRowColumn(2, :); edgePattern = edgePattern;
    
    clear currentRowCol sMatrix sRow;
    [x1, y1] = find(edgePattern == 1);
    candidate_list = [x1, y1];
    sequence = 1;
    currentRowCol(sequence, :) = initialRowColumn_insideFunc;
    Euclidean_dList = vecnorm([candidate_list - currentRowCol(1, :)]');
    current_candidate_ordering = find(Euclidean_dList == 0);
    candidate_list(current_candidate_ordering, :) = [];
    
    valida_PATH_threshold = 2^0.5;
    
    totoal_number = size(x1, 1);
    
    for i = 2 : totoal_number 
        Euclidean_dList = vecnorm([candidate_list - currentRowCol(i - 1, :)]');
        current_candidate_ordering = find(Euclidean_dList <= valida_PATH_threshold & Euclidean_dList > 0);
        
        current_candidate_number = size(current_candidate_ordering, 2);
        
        if(current_candidate_number == 0 )
           break; 
        end
        
        if(current_candidate_number ~= 0)
           
            [c_x , ] = find( Euclidean_dList(current_candidate_ordering) == min(Euclidean_dList(current_candidate_ordering)) ); 
            
                       
            number_in_candidate_list = current_candidate_ordering(c_x(1));
            
            currentRowCol(i , :) = candidate_list(number_in_candidate_list, :);
            candidate_list(number_in_candidate_list, :) = [];
                        
        end

    end
    
    
    % ---- test plotting sequence -----
    %{
    figure; imshow(edgePattern);
    for j = 1 : size(currentRowCol, 1)
        hold on; pause(0.05);
        scatter(currentRowCol(j, 2), currentRowCol(j, 1), 'r');
    end
    %}
    
    % it returns the data following the form: [row, col]
    % return currentRowCol
end