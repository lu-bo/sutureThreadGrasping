function returned_key_points = suture_key_points_aligment(input_key_points, defined_key_num)

    interval = size(input_key_points, 1) / defined_key_num;
    returned_key_points = [];
    
    for i = 1 : defined_key_num
        line_num = round(1 + (i - 1) * interval);
        returned_key_points(i, :) = input_key_points(line_num, :);
    end

end