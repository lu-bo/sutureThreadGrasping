
%%
classdef vrep_dVRK
    properties
        %opening_close_angle = 20;
    end
    
    methods (Static)
        function intial_config_setup()
            vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
            vrep.simxFinish(-1); % just in case, close all opened connections
            clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
            vrep.simxSynchronous(clientID, true);
            
            
            Joint_list_1 = {'J1_PSM1', 'J2_PSM1', 'J3_PSM1', 'J1_TOOL1', 'J2_TOOL1', 'J3_dx_TOOL1', 'J3_sx_TOOL1'};
            Joint_list_2 = {'J1_PSM2', 'J2_PSM2', 'J3_PSM2', 'J1_TOOL2', 'J2_TOOL2', 'J3_dx_TOOL2', 'J3_sx_TOOL2'};
            
            % ---- extract PSM1 information and setup initial config
            [~, J1_PSM1] = vrep.simxGetObjectHandle(clientID, 'J1_PSM2', vrep.simx_opmode_blocking);
            [~, J2_PSM1] = vrep.simxGetObjectHandle(clientID, 'J2_PSM2', vrep.simx_opmode_blocking);
            [~, J3_PSM1] = vrep.simxGetObjectHandle(clientID, 'J3_PSM2', vrep.simx_opmode_blocking);

            [~, J1_PSM1_init_position] = vrep.simxGetJointPosition(clientID, J1_PSM1, vrep.simx_opmode_blocking);
            [~, J2_PSM1_init_position] = vrep.simxGetJointPosition(clientID, J2_PSM1, vrep.simx_opmode_blocking);
            [~, J3_PSM1_init_position] = vrep.simxGetJointPosition(clientID, J3_PSM1, vrep.simx_opmode_blocking);
            
            init_q_1 = [0.6981
                        -0.2618
                        0.1000   %initial_prismatic_length
                        0.0000
                        0.0000
                        0.0000];
                    
            init_q_2 = [-0.6981
                        -0.2618
                        0.1000   %initial_prismatic_length
                        0.0000
                        0.0000
                        0.0000];
                      
            Joint_Handle_1 = [];
            Joint_Handle_2 = [];
            
            for i = 1 : 6

                [~, Joint_Handle_1(i, 1)] = vrep.simxGetObjectHandle(clientID, Joint_list_1{i}, vrep.simx_opmode_blocking);
                [~, Joint_Handle_2(i, 1)] = vrep.simxGetObjectHandle(clientID, Joint_list_2{i}, vrep.simx_opmode_blocking);
                vrep.simxSetJointPosition(clientID, Joint_Handle_1(i, 1), init_q_1(i, 1), vrep.simx_opmode_blocking);
                vrep.simxSetJointPosition(clientID, Joint_Handle_2(i, 1), init_q_2(i, 1), vrep.simx_opmode_blocking);
                
            end
            vrep.simxSynchronousTrigger(clientID);
        end
        
        
        function vrep_grasper_open()
            vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
            vrep.simxFinish(-1); % just in case, close all opened connections
            clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
            Joint_list = {'J1_PSM1', 'J2_PSM1', 'J3_PSM1', 'J1_TOOL1', 'J2_TOOL1', 'J3_dx_TOOL1', 'J3_sx_TOOL1'};
            
            [~, grasper_half_1]=vrep.simxGetObjectHandle(clientID, Joint_list{6}, vrep.simx_opmode_blocking);
            [~, grasper_half_2]=vrep.simxGetObjectHandle(clientID, Joint_list{7}, vrep.simx_opmode_blocking);
            
            opening_angle = 20;
            vrep.simxSetObjectOrientation(clientID, grasper_half_1, grasper_half_1, [0, 0, opening_angle*pi/180], vrep.simx_opmode_blocking); % revolute joint
            vrep.simxSetObjectOrientation(clientID, grasper_half_2, grasper_half_2, [0, 0, opening_angle*pi/180], vrep.simx_opmode_blocking); % revolute joint

        end
        function vrep_grasper_close()
            vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
            vrep.simxFinish(-1); % just in case, close all opened connections
            clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
            Joint_list = {'J1_PSM1', 'J2_PSM1', 'J3_PSM1', 'J1_TOOL1', 'J2_TOOL1', 'J3_dx_TOOL1', 'J3_sx_TOOL1'};
            
            [~, grasper_half_1]=vrep.simxGetObjectHandle(clientID, Joint_list{6}, vrep.simx_opmode_blocking);
            [~, grasper_half_2]=vrep.simxGetObjectHandle(clientID, Joint_list{7}, vrep.simx_opmode_blocking);
            
            close_angle = -20;
            vrep.simxSetObjectOrientation(clientID, grasper_half_1, grasper_half_1, [0, 0, close_angle*pi/180], vrep.simx_opmode_blocking); % revolute joint
            vrep.simxSetObjectOrientation(clientID, grasper_half_2, grasper_half_2, [0, 0, close_angle*pi/180], vrep.simx_opmode_blocking); % revolute joint

        end

%%        
       
        function computed_q = vrep_dVRK_IK_multisteps(Target_position, Target_orientation, init_q_6, step_num)
        % Input q_6 angle, output initial angles of 6 joints and their
        % computed angles to reach the desired config
        %% initialize the pose of dVRK 

        % list of the joint:
        % 1. J1_PSM1
        % 2. J2_PSM1
        % 3. J3_PSM1
        % 4. J1_TOOL1
        % 5. J2_TOOL1
        % 6. J3_dx_TOOL1 & J2_sx_TOOL1

        %clear all; clc;
        % Set intial angle and positon to these joints
        vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
        vrep.simxFinish(-1); % just in case, close all opened connections
        clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
        vrep.simxSynchronous(clientID, true);
        
        Joint_list = {'J1_PSM1', 'J2_PSM1', 'J3_PSM1', 'J1_TOOL1', 'J2_TOOL1', 'J3_dx_TOOL1', 'J3_sx_TOOL1'};
        
        %%
        Joint_Handle = [];
        %initial_prismatic_length = 0.1;
        for i = 1 : 7
            [~, Joint_Handle(i, 1)] = vrep.simxGetObjectHandle(clientID, Joint_list{i}, vrep.simx_opmode_blocking);
        end

        %%
        [~, current_q(1,1)]=vrep.simxGetJointPosition(clientID, Joint_Handle(1), vrep.simx_opmode_blocking);
        [~, current_q(2,1)]=vrep.simxGetJointPosition(clientID, Joint_Handle(2), vrep.simx_opmode_blocking);
        [~, current_q(3,1)]=vrep.simxGetJointPosition(clientID, Joint_Handle(3), vrep.simx_opmode_blocking);
        [~, current_q(4,1)]=vrep.simxGetJointPosition(clientID, Joint_Handle(4), vrep.simx_opmode_blocking);
        [~, current_q(5,1)]=vrep.simxGetJointPosition(clientID, Joint_Handle(5), vrep.simx_opmode_blocking);
        %[~, init_q(6,1)]=vrep.simxGetJointPosition(clientID, Joint_Handle(6), vrep.simx_opmode_blocking);
        current_q(6,1) = init_q_6;
        
        %PSM_q = [       0;        0;       0;       0;        0;       0];
        PSM_q = current_q;
        PSM_M = [0.9984    0.0402    0.0402    0.0000
                -0.0402    0.9992   -0.0003   -0.0066
                -0.0402   -0.0013    0.9992    0.0000
                 0         0         0         1.0000]; 
                
        PSM_S = [0         1.0000    0         0.0000    0.9992    0.0402
                 0         0         0         1.0000    0.0000   -0.0003
                 1.0000   -0.0000    0         0.0003   -0.0402    0.9992
                 0         0.0000    0.0000   -0.0000    0.0006   -0.0066
                 0         0.0000    1.0000   -0.0000    0.0000   -0.0000
                 0         0.0000    0.0003    0.0000    0.0156    0.0003];

        %%

        % set eulerangles for the end_effector
        Steps = step_num;
        q = current_q; % should be propoerly set in order to get correct KF information.
        
        [Target_theta, Target_r] = rotation_and_axisAngle.Ro2Ax(Target_orientation);
        
        Current_EulerAngles=[]; Current_Position=[];
        Current_theta=[]; Current_r=[];
        Increase_Eulerangle=[]; Increase_translation=[];
        %Target_orientation = [2*pi/4 pi/4 1*pi/4]; % angle value --> set the desired values

        [~, Current_EulerAngles] = vrep.simxGetObjectOrientation(clientID, Joint_Handle(6), Joint_Handle(1), vrep.simx_opmode_blocking);
        Current_rotation = eulerAngle_2_rotationMatrix(Current_EulerAngles);
        [Current_theta, Current_r] = rotation_and_axisAngle.Ro2Ax(Current_rotation);
        
        Increase_theta=(Target_theta-Current_theta)/Steps;
        Increase_r=(Target_r-Current_r)/Steps;
        %Increase_Eulerangle = (Target_orientation - Current_EulerAngles) / Steps; % angle value

        % -------------------------------------------------------------------------
        %Target_position = [-0.1 0.15 0.15];

        [~, Current_Position]  = vrep.simxGetObjectPosition(clientID, Joint_Handle(6), Joint_Handle(1), vrep.simx_opmode_blocking);
        Increase_translation = (Target_position - Current_Position)/Steps;

        %%
        P_Current_theta=[];P_Current_r=[];
        for i = 2 : Steps + 1 % step number --> pay attention!
            
            P_Current_theta(i, :) = Current_theta(i - 1, :) + Increase_theta(i - 1, :);
            P_Current_r(i, :) = Current_r(i - 1, :) + Increase_r(i - 1, :);

            IK_RotationMatrix = rotation_and_axisAngle.Ax2Ro(P_Current_theta(i, :), P_Current_r(i, :));
            
            P_Current_Position(i, :) = Current_Position(i - 1, :) + Increase_translation(i - 1, :);
            IK_translation = P_Current_Position(i, :)';

            IK_transformation = [IK_RotationMatrix IK_translation; 0 0 0 1];

            % skew
            ep_esp = 1e-5;
            eo_esp = 1e-3;

            % PSM parameters modify
            PSM_joint_size = 6;
            PSM_joint_type = [       0;        0;       1;       0;        0;       0];
            PSM_a          = [       0;        0;       0;       0;   -0.009;       0];
            PSM_alpha      = [    pi/2;    -pi/2;       0;  1.5708;   1.5708;   3.1416];
            PSM_d_base     = [       0;        0; -0.3677;  0.4521;  -0.0004;       0];
            PSM_q_base     = [    -pi/2;       0;  1.6110;  1.6110;   1.5306;   3.0611];

            if (i > 2)
               PSM_q = q(:, i - 1); 
            end

            PSM = robot(PSM_joint_size, PSM_joint_type, PSM_a, PSM_alpha, PSM_d_base, PSM_q_base);

            q(:,i) = PSM.IKinSpace(PSM_S, PSM_M, IK_transformation, PSM_q, eo_esp, ep_esp);

            % drive the joints in PSM 
            vrep.simxSetJointPosition(clientID, Joint_Handle(1, 1), q(1,i), vrep.simx_opmode_blocking); % revolute joint
            vrep.simxSetJointPosition(clientID, Joint_Handle(2, 1), q(2,i), vrep.simx_opmode_blocking); % revolute joint
            vrep.simxSetJointPosition(clientID, Joint_Handle(3, 1), q(3,i), vrep.simx_opmode_blocking); %prismatic joint
            vrep.simxSetJointPosition(clientID, Joint_Handle(4, 1), q(4,i), vrep.simx_opmode_blocking); % revolute joint
            vrep.simxSetJointPosition(clientID, Joint_Handle(5, 1), q(5,i), vrep.simx_opmode_blocking); % revolute joint
            vrep.simxSetObjectOrientation(clientID, Joint_Handle(6, 1), Joint_Handle(6, 1), [0, 0,  q(6,i) - q(6,i-1)], vrep.simx_opmode_blocking); % revolute joint
            vrep.simxSetObjectOrientation(clientID, Joint_Handle(7, 1), Joint_Handle(7, 1), [0, 0,-(q(6,i) - q(6,i-1))], vrep.simx_opmode_blocking); % revolute joint
            %q_6_increase = q_6_increase + q(6,i) - q(6,i-1);
            
            % ------------- real pose && pose error between planned pose and real pose in the current loop -------------
            [~, now_EulerAngles] = vrep.simxGetObjectOrientation(clientID, Joint_Handle(6), Joint_Handle(1), vrep.simx_opmode_blocking);
            now_rotation = eulerAngle_2_rotationMatrix(now_EulerAngles);
            [Now_theta, Now_r] = rotation_and_axisAngle.Ro2Ax(now_rotation);
            
            [~, now_Position]  = vrep.simxGetObjectPosition(clientID, Joint_Handle(6), Joint_Handle(1), vrep.simx_opmode_blocking);
            
            % put current pose into the pose matrix
            Current_theta(i, :) = Now_theta;
            Current_r(i, :) = Now_r;
            %Current_EulerAngles(i, :) = now_EulerAngles;
            Current_Position(i, :) = now_Position;

            % compute step incremental values for pose
            if ((Steps - i) == 0)
                %Increase_Eulerangle(i, :) = (Target_orientation - now_EulerAngles);
                Increase_theta(i, :)=(Target_theta-Now_theta);
                Increase_r(i, :)=(Target_r-Now_r);
                
                Increase_translation(i, :)= (Target_position - now_Position);
            end
            if ((Steps - i) < 0)
                break;
            end
            if ((Steps - i) > 0)
                %Increase_Eulerangle(i, :) = (Target_orientation - now_EulerAngles) / (Steps - 1 - i);
                Increase_theta(i, :)=(Target_theta-Now_theta)/(Steps - 1 - i);
                Increase_r(i, :)=(Target_r-Now_r)/(Steps - 1 - i);
                Increase_translation(i, :)= (Target_position - now_Position) / (Steps - 1 - i);
            end
        end
        %
        computed_q = rem(q(:,end), 2*pi);
        %[~, J3_sx_TOOL1_EulerAngles] = vrep.simxGetObjectOrientation(clientID, J3_sx_TOOL1, J2_TOOL1, vrep.simx_opmode_blocking);
        %%
        %vrep.simxSetJointPosition(clientID, Joint_Handle(5, 1), 0, vrep.simx_opmode_blocking); 
        %vrep.simxSetObjectOrientation(clientID, Joint_Handle(6, 1), Joint_Handle(6, 1), [-5,0,0]*pi/180, vrep.simx_opmode_blocking);
        %vrep.simxSetObjectOrientation(clientID, Joint_Handle(7, 1), Joint_Handle(7, 1), [0,0,-pi/2], vrep.simx_opmode_blocking);
        vrep.simxSynchronousTrigger(clientID);
        end
    end
    

end
