function POINT_SEQUENCE_FINAL = Ordering_curvilinear_obejcts(updateKeyPointsImage1, suture_tip1, Reversed_biOutIm1)
% -----        creterias usded to get "TRIM POINTS" set.              -----
    % ----- ============================================================= -----
    % ----- creteria one: trim the points which are too close to others.  -----
    %updateKeyPointsImage1 =  [selected_k_p(: , 2) , selected_k_p(: , 1)];
    %global Reversed_biOutIm1
    %global suture_tip1
    
    TRIMMED_POINT = updateKeyPointsImage1(1 , :);

    POINT_SEQUENCE_FINAL = [];
    FINAL_ELEMENT_NUM = 0;
    % ---- important parameters ------
    % ---------------------------------------------------------------------
  
    THRE = 3;
    RADIUS = THRE * 20; % set evaluation range within all the trimmed points
    thre_out_of_zone = round(RADIUS / 2);
    thre_angle_variation = 100;
    
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
                P1_out_scale = 1;
                P1_in = 5 * P1_variation;  % out of zone area point
                P2 = 0.8 * P2_variation;    % distance
                P3 = 0.1 * angle_variation_group;

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
                    RANK = P1_out_scale*log(P1_in * STRUCT_EVA_CANDIDATE(ALLCAN, 4) + 1.1) + ... % <---- out of zone pixel numbers
                           P2 * STRUCT_EVA_CANDIDATE(ALLCAN, 5) + ... % <---- length
                           exp(P3 * STRUCT_EVA_CANDIDATE(ALLCAN, 6)); % <---- angle variations
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

end