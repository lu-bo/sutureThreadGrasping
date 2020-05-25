function OUT_ZONE_NUM = out_of_body_number(EVA_CANDIDATE, CURRENT_POINT, Reversed_biOutIm1)
    j = 1;
    OUT_ZONE_NUM = 0;
    
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