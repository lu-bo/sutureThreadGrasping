%% initial position setups of dVRK system and the suture 
setup_initialization;
vrep_dVRK.intial_config_setup

q_6_total = [];

%%
if ~exist('q_6_total','var') 
    q_6_total = []; 
end
rotm_21=[]; rotation_31=[]; tipOffSet_J1_PSM1=[]; eulXYZ_31=[]; orientation_cost=[]; 
Position_PSM1 = []; eulXYZ_31 = [];

%% quantitative selection/computation on the grasping point
accum_len = 0;
set_threshold = 0.13;
for node_i = 2 : size(xyzsg, 1)
    accum_len = norm(xyzsg(node_i, :) - xyzsg(node_i - 1, :)) +accum_len;
    if (accum_len > set_threshold)
        break;
    end
end

%% 

for node_i = node_i
    %% normal vector of plane formed by surrounding points   
    sf = fit([xyzsg(:, 1), xyzsg(:, 2)],xyzsg(:, 3),'poly33');
    
    bi_bound = 5*10^(-3); grid_size = 1*10^(-3);
    X_plot3=[]; Y_plot3=[]; Z_plot3=[];
    Norm_x=[]; Norm_y=[]; Norm_z=[];
    
    [X_plot3, Y_plot3] =...
    meshgrid(xyzsg(node_i, 1) - bi_bound: grid_size : xyzsg(node_i, 1) + bi_bound,...
             xyzsg(node_i, 2) - bi_bound: grid_size : xyzsg(node_i, 2) + bi_bound ...
             );
    Z_plot3 = sf(X_plot3, Y_plot3);
    [Norm_x, Norm_y, Norm_z] = surfnorm(X_plot3, Y_plot3, Z_plot3);
    P_C_X = (size(X_plot3,1) + 1)/2; P_C_Y = (size(Y_plot3,1) + 1)/2;
    nor_vec = [(Norm_x(P_C_X,P_C_Y)), mean(Norm_y(P_C_X,P_C_Y)), mean(Norm_z(P_C_X,P_C_Y))];
    
    %% initialization of the tool pose
    % Z_e axis is determined by grasping point
    vv3 = xyzsg(node_i, :) - xyzsg(node_i - 1, :); scale_vv3 = 1 / norm(vv3); 
    v3_p = scale_vv3 * vv3; v3_n = -v3_p;
    
    % set initial Y_e axis
    v2_initial = [0 0 0];
    v2_initial(2) = 0.1; v2_initial(1) = (-v3_p(2)/v3_p(1))*v2_initial(2); 
    scale_v2 = 1 / norm(v2_initial); v2_initial = scale_v2 * v2_initial;
    
    % compute X_e axis using right-hand rule
    v1_initial=cross(v2_initial, v3_p);
    % build up initial rotation matrix
    initial_R = [v1_initial' v2_initial' v3_p'];
    %initial_R=initial_R*RotX(30*pi/180);
    
    %% calculation 1(numerical solution) - end-effector orthogonal to the surface 
    A_normal=(-initial_R(1,1)*nor_vec(1)-initial_R(2,1)*nor_vec(2)-initial_R(3,1)*nor_vec(3));
    B_normal=( initial_R(1,2)*nor_vec(1)+initial_R(2,2)*nor_vec(2)+initial_R(3,2)*nor_vec(3));
    phi=atan2(B_normal,A_normal);
    theta_1=pi/2-phi;
    
    %% calculation 2(numerical solution) - maximal projection length
    cRt=initial_R;
    cPt=xyzsg(node_i, :)';
    cPt_ = [cPt(1)/cPt(3); cPt(2)/cPt(3); 1];
    Ltip=0.0051;
    ePt=[0,-Ltip,0]';
    
    [new_cRe, new_theta] = get_orientation_from_projection(cRt, Ltip, cPt);
    [theta_size, ~]=size(new_cRe);
    
    theta_2=[];
    calc_proj = 0;
    for j=1:theta_size
        
        new_cPe = new_cRe{j} * (ePt) + cPt;
        new_cPe_ = [new_cPe(1)/new_cPe(3); new_cPe(2)/new_cPe(3); 1];
        new_proj = sqrt((new_cPe_(1)-cPt_(1))^2 + (new_cPe_(2)-cPt_(2))^2);
        
        if calc_proj < new_proj
            calc_proj = new_proj;
            theta_2 = new_theta(j);
        end
    end
    
    k = 0.5;    
    if(theta_1>pi)
        theta_1 = theta_1-2*pi;
    end
    if(theta_2>pi)
        theta_2 = theta_2-2*pi;
    end
    
    if(abs(theta_1-theta_2)<pi)
        theta_e = k*theta_1+(1-k)*theta_2;
    else
        d_theta = 2*pi-abs(theta_1-theta_2);
        if (theta_1>theta_2)
            theta_e = theta_2-k*d_theta;
        else
            theta_e = theta_2+k*d_theta;
        end
    end
    
%     theta_e=new_theta(3);
    cROTt = cRt*RotZ(theta_e);
    
    
    %% compute the minimal "cost" target angle
    [~, current_ori] = vrep_pos_ori('J3_dx_TOOL1', 'J1_PSM1');
    [rotm_32, ~, CoCo] = dVRK_get_transformation_and_coordinates('Vision_sensor_left', 'J1_PSM1', xyzsg);
    rotm_num = 1;
    
    rotm_21(:, :, rotm_num) = cROTt;
    rotation_31(:, :, rotm_num) = rotm_32 * rotm_21(:, :, rotm_num);

    tipOffSet_J1_PSM1(:, rotm_num) = rotation_31(:, :, rotm_num) * ePt;
    Position_PSM1(rotm_num, :) = CoCo(node_i, :) + tipOffSet_J1_PSM1(:, rotm_num)';
    if (isempty(q_6_total))
        q_6_total = 0;
    end

    step_number = 10;
    returned_q = vrep_dVRK.vrep_dVRK_IK_multisteps(Position_PSM1(rotm_num, :), ...
                                                  rotation_31(:, :, rotm_num),...
                                                  q_6_total, step_number);
    computed_q(:, rotm_num) = returned_q;
    computed_q = real(computed_q);
    %orientation_cost(rotm_num, :) = sum(abs(eulXYZ_31(:, :, rotm_num) - current_ori));
    
    computed_q(6,:) = rem(computed_q(6,:), 2*pi);
    q_6_total = returned_q(6);
    %[row_rotm, col_rotm] = find(orientation_cost == min(orientation_cost));
    
end

