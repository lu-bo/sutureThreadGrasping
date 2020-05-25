%%%%% robot class %%%%%
classdef robot < handle
    properties
%% basic parameters
        JOINT_SIZE = 0;
        JOINT_TYPE = []; 
        A = [];
        ALPHA = [];
        D = [];
        THETA = [];
        DRAW_SCALE = 0.1;
        
%% internal parameters
        EP_ESP = 1e-5;
        EO_ESP = 1e-3;
        ITERATE_TIMES = 10;
        
%% function pointers
        FEO = 0;
        RED = 0;
    end

    methods
        function obj = robot(joint_size, joint_type, a, alpha, d, theta)
            obj.JOINT_SIZE = joint_size+1;
            obj.JOINT_TYPE = joint_type;
            obj.A = a;
            obj.ALPHA = alpha;
            obj.D = d;
            obj.THETA = theta;
            obj.FEO = {@obj.feo_samuel};
            obj.RED = {@obj.redundant};
        end
        
        function ans = A1(obj, theta, d)
            ans = [cos(theta), -sin(theta), 0, 0;
                   sin(theta), +cos(theta), 0, 0;
                            0,           0, 1, d;
                            0,           0, 0, 1];
        end

        function ans = A2(obj, alpha, a)
            ans = [1,           0,           0, a;
                   0, +cos(alpha), -sin(alpha), 0;
                   0, +sin(alpha), +cos(alpha), 0;
                   0,           0,           0, 1];
        end

        function ans = sk(obj, a)
            ans = obk.VecToso3(a);
        end
        

%% modify DH method (Creig's book), xi-1 ⊥ zi-1, zi

        %
        % i-1         i         
        %  +----------+  Oi
        %             |         i+1
        %             +----------+  Qi+1  
        %
        function ans = MDH(obj, a, alpha, d, theta)
            ans = obj.A2(alpha, a) * obj.A1(theta, d);
        end
        
        function bTe = mfk(obj, theta)
            bTe = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
            for k = 2:obj.JOINT_SIZE
                if obj.JOINT_TYPE(k-1) == 0
                    bTe = bTe*obj.MDH(obj.A(k-1), obj.ALPHA(k-1), obj.D(k-1), obj.THETA(k-1)+theta(k-1));
                else
                    bTe = bTe*obj.MDH(obj.A(k-1), obj.ALPHA(k-1), obj.D(k-1)+theta(k-1), obj.THETA(k-1));
                end
            end
        end
        
        function T = mfk_full(obj, theta)
            T = cell(obj.JOINT_SIZE);
            T{1} = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
            for k=2:obj.JOINT_SIZE
                if obj.JOINT_TYPE(k-1) == 0
                    T{k} = T{k-1}*obj.MDH(obj.A(k-1), obj.ALPHA(k-1), obj.D(k-1), obj.THETA(k-1)+theta(k-1));
                else
                    T{k} = T{k-1}*obj.MDH(obj.A(k-1), obj.ALPHA(k-1), obj.D(k-1)+theta(k-1), obj.THETA(k-1));
                end
            end
        end

        function bJe = mdk(obj, theta)
            Te = obj.mfk(theta);
            T = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
            bJe = [];
            for k=2:obj.JOINT_SIZE
                if obj.JOINT_TYPE(k-1) == 0
                    T = T*obj.MDH(obj.A(k-1), obj.ALPHA(k-1), obj.D(k-1), obj.THETA(k-1)+theta(k-1));
                    J = [cross(T(1:3,3), Te(1:3, 4) - T(1:3, 4)); T(1:3, 3)];
                elseif obj.JOINT_TYPE(k-1) == 1
                    T = T*obj.MDH(obj.A(k-1), obj.ALPHA(k-1), obj.D(k-1)+theta(k-1), obj.THETA(k-1));
                    J = [T(1:3, 3); [0; 0; 0]];
                end
                bJe = [bJe J];
            end
        end
        
        function [ans, count, eo_norm] = mik(obj, Rt, Pt, q)
            Tc = obj.mfk(q);
            ep = Pt - Tc(1:3, 4);
            [keo, eo] = obj.FEO{1}(Tc(1:3, 1:3), Rt);
            count = 0;
            while (norm(ep) > obj.EP_ESP || norm(eo) > obj.EO_ESP) && count < obj.ITERATE_TIMES
                J = obj.mdk(q);
                dq = pinv(J) * [ep; keo] * 0.5;
                if obj.JOINT_SIZE == 8
                    dq = dq + obj.RED{1}(obj.mfk_full(q), J);
                end
                q = q + dq;

                Tc = obj.mfk(q);
                ep  = Pt - Tc(1:3, 4);
                [keo, eo] = obj.FEO{1}(Tc(1:3, 1:3), Rt);
                
                count = count + 1;
            end
            %sprintf("iterates %d times", count)
            eo_norm = norm(eo);
            ans = q;
        end

%% standard DH method (Siciliano's book), xi ⊥ zi-1, zi
        %
        % i-1         i         
        %  +----------+  Oi-1
        %             |         i+1
        %             +----------+  Qi 
        %                       
        function ans = SDH(obj, a, alpha, d, theta)
            ans = obj.A1(theta, d) * obj.A2(alpha, a);
        end
        
        function bTe = sfk(obj, theta)
            bTe = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
            for k = 2:obj.JOINT_SIZE
                if obj.JOINT_TYPE(k-1) == 0
                    bTe = bTe*obj.SDH(obj.A(k-1), obj.ALPHA(k-1), obj.D(k-1), obj.THETA(k-1)+theta(k-1));
                else
                    bTe = bTe*obj.SDH(obj.A(k-1), obj.ALPHA(k-1), obj.D(k-1)+theta(k-1), obj.THETA(k-1));
                end
            end
        end
        
        function T = sfk_full(obj, theta)
            T = cell(obj.JOINT_SIZE);
            T{1} = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
            for k=2:obj.JOINT_SIZE
                if obj.JOINT_TYPE(k-1) == 0
                    T{k} = T{k-1}*obj.SDH(obj.A(k-1), obj.ALPHA(k-1), obj.D(k-1), obj.THETA(k-1)+theta(k-1));
                else
                    T{k} = T{k-1}*obj.SDH(obj.A(k-1), obj.ALPHA(k-1), obj.D(k-1)+theta(k-1), obj.THETA(k-1));
                end
            end
        end
        
        function bJe = sdk(obj, theta)
            Te_all = obj.sfk_full(theta);
            Te = Te_all{obj.JOINT_SIZE-1};
            T = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
            bJe = [];
            for k=2:obj.JOINT_SIZE
                if obj.JOINT_TYPE(k-1) == 0
                    T = T*obj.SDH(obj.A(k-1), obj.ALPHA(k-1), obj.D(k-1), obj.THETA(k-1)+theta(k-1));
                    J = [cross(T(1:3,3), Te(1:3, 4) - T(1:3, 4)); T(1:3, 3)];
                elseif obj.JOINT_TYPE(k-1) == 1
                    T = T*obj.SDH(obj.A(k-1), obj.ALPHA(k-1), obj.D(k-1)+theta(k-1), obj.THETA(k-1));
                    J = [T(1:3, 3); [0; 0; 0]];
                end
                bJe = [bJe J];
            end
        end
         
        function [ans, count, eo_norm] = sik(obj, Rt, Pt, q)
            Tc = obj.sfk(q);
            ep = Pt - Tc(1:3, 4);
            [keo, eo] = obj.FEO{1}(Tc(1:3, 1:3), Rt);
            count = 0;
            while (norm(ep) > obj.EP_ESP || norm(eo) > obj.EO_ESP) && count < obj.ITERATE_TIME
                J = obj.sdk(q);
                dq = pinv(J) * [ep; keo] * 0.5;
                if obj.JOINT_SIZE == 8
                    dq = dq + obj.RED(obj.sfk_full(q), J);
                end
                q = q + dq;

                Tc = obj.sfk(q);
                ep  = Pt - Tc(1:3, 4);
                [keo, eo] = obj.FEO{1}(Tc(1:3, 1:3), Rt);
                
                count = count + 1;
            end
            %sprintf("iterates %d times", count)
            eo_norm = norm(eo);
            ans = q;
        end
        
%% skew theory (Modern Robotics)
        function AdT = Adjoint(obj, T)
            [R, p] = obj.TransToRp(T);
            AdT = [R, zeros(3); obj.VecToso3(p) * R, R];
        end
        
        function [omghat, theta] = AxisAng3(obj, expc3)
            theta = norm(expc3);
            omghat = expc3 / theta;
        end
        
        function [S, theta] = AxisAng6(obj, expc6)
            theta = norm(expc6(1: 3));
            if norm(theta) < 1e-6
                theta = norm(expc6(4: 6));
            end
            S = expc6 / theta;      
        end
        
        function T = FKinBody(obj, M, Blist, thetalist)
            T = M;
            for i = 1: size(thetalist)
                T = T * obj.MatrixExp6(obj.VecTose3(Blist(:, i) * thetalist(i)));
            end
        end
        
        function T = FKinSpace(obj, M, Slist, thetalist)
            T = M;
            for i = length(thetalist): -1: 1
                T = obj.MatrixExp6(obj.VecTose3(Slist(:, i) * thetalist(i))) * T;
            end
        end
        
        function Jb = JacobianBody(obj, Blist, thetalist)
            Jb = Blist;
            T = eye(4);
            for i = length(thetalist) - 1: -1: 1   
                T = T * obj.MatrixExp6(obj.VecTose3(-1 * Blist(:, i + 1) * thetalist(i + 1)));
                Jb(:, i) = obj.Adjoint(T) * Blist(:, i);
            end
        end
        
        function Js = JacobianSpace(obj, Slist, thetalist)
            Js = Slist;
            T = eye(4);
            for i = 2: length(thetalist)  
                T = T * obj.MatrixExp6(obj.VecTose3(Slist(:, i - 1) * thetalist(i - 1)));
                Js(:, i) = obj.Adjoint(T) * Slist(:, i);
            end
        end
        
        function [thetalist, success] = IKinBody(obj, Blist, M, T, thetalist0, eomg, ev)
            thetalist = thetalist0;
            i = 0;
            maxiterations = 20;
            Vb = obj.se3ToVec(obj.MatrixLog6(obj.TransInv(obj.FKinBody(M, Blist, thetalist)) * T));
            err = norm(Vb(1: 3)) > eomg || norm(Vb(4: 6)) > ev;
            while err && i < maxiterations
                thetalist = thetalist + pinv(obj.JacobianBody(Blist, thetalist)) * Vb;
                i = i + 1;
                Vb = obj.se3ToVec(obj.MatrixLog6(obj.TransInv(obj.FKinBody(M, Blist, thetalist)) * T));
                err = norm(Vb(1: 3)) > eomg || norm(Vb(4: 6)) > ev;
            end
            success = ~ err;
        end
        
        function [thetalist, success] = IKinSpace(obj, Slist, M, T, thetalist0, eomg, ev)
            thetalist = thetalist0;
            i = 0;
            maxiterations = 100;
            Tsb = obj.FKinSpace(M, Slist, thetalist);
            Vs = obj.Adjoint(Tsb) * obj.se3ToVec(obj.MatrixLog6(obj.TransInv(Tsb) * T));
            err = norm(Vs(1: 3)) > eomg || norm(Vs(4: 6)) > ev;
            while err && i < maxiterations
                thetalist = thetalist + pinv(obj.JacobianSpace(Slist, thetalist)) * Vs;
                i = i + 1;
                Tsb = obj.FKinSpace(M, Slist, thetalist);
                Vs = obj.Adjoint(Tsb) * obj.se3ToVec(obj.MatrixLog6(obj.TransInv(Tsb) * T));
                err = norm(Vs(1: 3)) > eomg || norm(Vs(4: 6)) > ev;
            end
            success = ~ err;
        end
        
        function R = MatrixExp3(obj, so3mat)
            omgtheta = obj.so3ToVec(so3mat);
            if norm(omgtheta) < 1e-6
                R = eye(3);
            else
                [omghat, theta] = obj.AxisAng3(omgtheta);
                omgmat = so3mat / theta;
                R = eye(3) + sin(theta) * omgmat + (1 - cos(theta)) * omgmat * omgmat;
            end
        end
        
        function T = MatrixExp6(obj, se3mat)
            omgtheta = obj.so3ToVec(se3mat(1: 3, 1: 3));
            if norm(omgtheta) < 1e-6
                T = [eye(3), se3mat(1: 3, 4); 0, 0, 0, 1];
            else
                [omghat, theta] = obj.AxisAng3(omgtheta);
                omgmat = se3mat(1: 3, 1: 3) / theta; 
                T = [obj.MatrixExp3(se3mat(1: 3, 1: 3)), ...
                     (eye(3) * theta + (1 - cos(theta)) * omgmat ...
                      + (theta - sin(theta)) * omgmat * omgmat) ...
                        * se3mat(1: 3, 4) / theta;
                     0, 0, 0, 1];
            end
        end
        
        function so3mat = MatrixLog3(obj, R)
            acosinput = (trace(R) - 1) / 2;
            if acosinput >= 1
                so3mat = zeros(3);
            elseif acosinput <= -1
                if norm(1 + R(3, 3)) > 1e-6
                    omg = (1 / sqrt(2 * (1 + R(3, 3)))) ...
                          * [R(1, 3); R(2, 3); 1 + R(3, 3)];
                elseif norm(1 + R(2, 2)) > 1e-6
                    omg = (1 / sqrt(2 * (1 + R(2, 2)))) ...
                          * [R(1, 2); 1 + R(2, 2); R(3, 2)];
                else
                    omg = (1 / sqrt(2 * (1 + R(1, 1)))) ...
                          * [1 + R(1, 1); R(2, 1); R(3, 1)];
                end
                so3mat = obj.VecToso3(pi * omg);
            else
                theta = acos(acosinput);
                so3mat = theta * (1 / (2 * sin(theta))) * (R - R');
            end
        end
        
        function expmat = MatrixLog6(obj, T)
            [R, p] = obj.TransToRp(T);
            omgmat = obj.MatrixLog3(R);
            if isequal(omgmat, zeros(3))
                expmat = [zeros(3), T(1: 3, 4); 0, 0, 0, 0];
            else
                theta = acos((trace(R) - 1) / 2);
                expmat = [omgmat, (eye(3) - omgmat / 2 ...
                                  + (1 / theta - cot(theta / 2) / 2) ...
                                    * omgmat * omgmat / theta) * p;
                          0, 0, 0, 0];    
            end
        end
        
        function [R, p] = TransToRp(obj, T)
            R = T(1: 3, 1: 3);
            p = T(1: 3, 4);
        end
        
        function se3mat = VecTose3(obj, V)
            se3mat = [obj.VecToso3(V(1: 3)), V(4: 6); 0, 0, 0, 0];
        end
        
        function so3mat = VecToso3(obj, omg)
            so3mat = [0, -omg(3), omg(2); omg(3), 0, -omg(1); -omg(2), omg(1), 0];
        end
        
        function adV = ad(obj, V)
            omgmat = obj.VecToso3(V(1: 3));
            adV = [omgmat, zeros(3); obj.VecToso3(V(4: 6)), omgmat];
        end
        
        function V = se3ToVec(obj, se3mat)
            V = [se3mat(3, 2); se3mat(1, 3); se3mat(2, 1); se3mat(1: 3, 4)];
        end
        
        function omg = so3ToVec(obj, so3mat)
            omg = [so3mat(3, 2); so3mat(1, 3); so3mat(2, 1)];
        end
        
        function invT = TransInv(obj, T)
            [R, p] = obj.TransToRp(T);
            invT = [R', -R' * p; 0, 0, 0, 1];
        end

%% redundant fucntions
        function ans = redundant(obj, T, J)
            R4 = T{5}(1:3, 1:3);
            R7 = T{8}(1:3, 1:3);
            a1 = -R7(3, 2)*R4(2, 2) + R7(2, 2)*R4(3, 2);
            a2 = +R7(3, 2)*R4(2, 1) - R7(2, 2)*R4(3, 1);
            b1 = +R7(3, 2)*R4(1, 2) - R7(1, 2)*R4(3, 2);
            b2 = -R7(3, 2)*R4(1, 1) + R7(1, 2)*R4(3, 1);
            c1 = -R7(2, 2)*R4(1, 2) + R7(1, 2)*R4(2, 2);
            c2 = +R7(2, 2)*R4(1, 1) - R7(1, 2)*R4(2, 1);
            roll = -atan2(2*a1*a2+2*b1*b2+2*c1*c2, a1*a1+b1*b1+c1*c1-a2*a2-b2*b2-c2*c2);
            if roll < -pi/2
                roll = - roll - pi;
            elseif roll > pi/2
                roll = pi - roll;
            end
            ans = 0.5 * (eye(7) - pinv(J)*J)*[0; 0; 0; roll; 0; 0; 0];
        end
        
        function set_fred(obj, red)
            obj.RED = red;
        end
        
%% orientation error fucntions
        function [ans, eo] = feo_crossproduct(obj, Rc, Rt)
            eo = 0.5*(cross(Rc(1:3, 1), Rt(1:3, 1)) + cross(Rc(1:3, 2), Rt(1:3, 2)) + cross(Rc(1:3, 3), Rt(1:3, 3)));
            ans = eo;
        end
        
        function [ans, eo] = feo_angle_and_axis(obj, Rc, Rt)
            eo = 0.5*(cross(Rc(1:3, 1), Rt(1:3, 1)) + cross(Rc(1:3, 2), Rt(1:3, 2)) + cross(Rc(1:3, 3), Rt(1:3, 3)));
            L = -0.5*(obj.sk(Rt(1:3, 1))*obj.sk(Rc(1:3, 1)) + obj.sk(Rt(1:3, 2))*obj.sk(Rc(1:3, 2)) + obj.sk(Rt(1:3, 3))*obj.sk(Rc(1:3, 3)));
            ans = pinv(L) * eo;
        end
        
        function [ans, eo] = feo_quaternion(obj, Rc, Rt)
            Qt = rotm2quat(Rt);
        	Qc = rotm2quat(Rc);
            Qe = quatmultiply(Qt, quatinv(Qc));
%             if (Qe(1) > 0.998)
%                 Qe = [1, 0, 0, 0];
%             end
            e1 = 0.5 * Qc(1) * [Qt(2); Qt(3); Qt(4)] - Qt(1) * [Qc(2); Qc(3); Qc(4)] - cross([Qt(2); Qt(3); Qt(4)], [Qc(2); Qc(3); Qc(4)]);
            e2 = [Qe(2); Qe(3); Qe(4)];
            eo = e1;
            if norm(e2) < norm(e1)
                eo = e2;
            end
            ans = eo;
        end

        function [ans, eo] = feo_samuel(obj, Rc, Rt)
            Re = Rc' * Rt;
            e = 0.5 * [Re(3,2)-Re(2,3); Re(1,3)-Re(3,1); Re(2,1)-Re(1,2)];
            eo = Rc * e;
            ans = eo;
        end
        
        function [ans, eo] = feo_euler(obj, Rc, Rt)
            Re = Rt * Rc';
            theta = asin(-Re(3, 1));
            if theta > pi/2
                theta = pi - theta;
            elseif theta < -pi/2
                theta = -pi - theta;
            end
            
            psi = asin(Re(3, 2) / sqrt(1 - Re(3, 1) * Re(3, 1)));
            if psi > pi/2
                psi = pi - psi;
            elseif psi < -pi/2
                psi = -pi - psi;
            end
            
            phi = asin(Re(2, 1) / sqrt(1 - Re(3, 1) * Re(3, 1)));
            if phi > pi/2
                phi = pi - phi;
            elseif phi < -pi/2
                phi = -pi - phi;
            end
            eo = [psi; theta; phi];
            ans = eo;
        end
        
        function set_feo(obj, feo, ep_esp, eo_esp, interate_times)
            obj.EP_ESP = ep_esp;
            obj.EO_ESP = eo_esp;
            obj.ITERATE_TIMES = interate_times;
            obj.FEO = feo;
        end
        
%% functions for drawing
        function DrawCoordinate(obj, name, p, varargin) % 以(x, y, z)为原点(R)为方向绘制坐标系
            R = [1 0 0; 0 1 0; 0 0 1];
            if numel(varargin) == 1
                R = varargin{1};
            end
            quiver3(p(1), p(2), p(3), R(1,1), R(2,1), R(3,1), 0.1, 'r');
            %quiver3(p(1), p(2), p(3), R(1,1), R(2,1), R(3,1), 0.1, '--');
            hold on;
            quiver3(p(1), p(2), p(3), R(1,2), R(2,2), R(3,2), 0.1, 'g');
            %quiver3(p(1), p(2), p(3), R(1,2), R(2,2), R(3,2), 0.1, '--');
            hold on;
            quiver3(p(1), p(2), p(3), R(1,3), R(2,3), R(3,3), 0.1, 'b');
            hold on;
            text(p(1), p(2), p(3), '');
            text(p(1)+R(1,1)*obj.DRAW_SCALE, p(2)+R(2,1)*obj.DRAW_SCALE, p(3)+R(3,1)*obj.DRAW_SCALE, ['x_{', name, '}']);
            text(p(1)+R(1,2)*obj.DRAW_SCALE, p(2)+R(2,2)*obj.DRAW_SCALE, p(3)+R(3,2)*obj.DRAW_SCALE, ['y_{', name, '}']);
            text(p(1)+R(1,3)*obj.DRAW_SCALE, p(2)+R(2,3)*obj.DRAW_SCALE, p(3)+R(3,3)*obj.DRAW_SCALE, ['z_{', name, '}']);
            hold on;
        end
        
        function DrawLine(obj, p, q) % 以(x, y, z)为原点(R)为方向绘制坐标系
            plot3([p(1) q(1)], [p(2) q(2)], [p(3) q(3)], 'LineWidth', 5, 'color', [0.7 0 0]);
            hold on;
        end
        
        function DrawCube(obj, P, R)
            n = 4;%设置多少个边逼近圆
            x = [ -1,  1,  1, -1, -1; -1,  1,  1, -1, -1] * 0.15 * obj.DRAW_SCALE;
            y = [  1,  1, -1, -1,  1;  1,  1, -1, -1,  1] * 0.15 * obj.DRAW_SCALE;
            z = [ -1, -1, -1, -1, -1;  1,  1,  1,  1,  1] * 0.50 * obj.DRAW_SCALE;
            for k=1:n+1
                xx1 = R(1, 1)*x(1, k)+R(1, 2)*y(1, k)+R(1, 3)*z(1, k)+P(1);
                yy1 = R(2, 1)*x(1, k)+R(2, 2)*y(1, k)+R(2, 3)*z(1, k)+P(2);
                zz1 = R(3, 1)*x(1, k)+R(3, 2)*y(1, k)+R(3, 3)*z(1, k)+P(3);
                xx2 = R(1, 1)*x(2, k)+R(1, 2)*y(2, k)+R(1, 3)*z(2, k)+P(1);
                yy2 = R(2, 1)*x(2, k)+R(2, 2)*y(2, k)+R(2, 3)*z(2, k)+P(2);
                zz2 = R(3, 1)*x(2, k)+R(3, 2)*y(2, k)+R(3, 3)*z(2, k)+P(3);
                x(1, k) =  xx1; x(2, k) =  xx2;
                y(1, k) =  zz1; y(2, k) =  zz2;
                z(1, k) =  yy1; z(2, k) =  yy2;
            end
            % 为变成实心封顶添加数据
            z2=[z(1, :); z; z(2, :)];
            x2=[x(1, :); x; x(2, :)];
            y2=[y(1, :); y; y(2, :)];
 
            z3=[z(1, :); z(1, :)];
            x3=[x(1, :); x(1, :)];
            y3=[y(1, :); y(1, :)];
    
            z4=[z(2, :); z(2, :)];
            x4=[x(2, :); x(2, :)];
            y4=[y(2, :); y(2, :)];
            
            surf(x2,z2,y2,'LineStyle','none','facealpha',0.5)
            map=jet(16);
            cl=4;%可设置16种颜色(1-16)
            colormap(map(cl,:))
            hold on
            surf(x3,z3,y3,'facealpha',0.5);
            hold on
            surf(x4,z4,y4,'facealpha',0.5);
            hold on
        end
        
        function DrawCylinder(obj, P, R)
            r = 0.015 * obj.DRAW_SCALE / 0.1;%圆柱半径
            n = 50;%设置多少个边逼近圆
            [x, y, z] = cylinder(r, n);%生成标准的100个面的圆柱数据，半径为r，高为1，底面圆心0，0；
            z=[z(1, :)-obj.DRAW_SCALE/2; z(2, :)*obj.DRAW_SCALE-obj.DRAW_SCALE/2];%圆柱高增高，变为高h
            for k=1:n+1
                xx1 = R(1, 1)*x(1, k)+R(1, 2)*y(1, k)+R(1, 3)*z(1, k)+P(1);
                yy1 = R(2, 1)*x(1, k)+R(2, 2)*y(1, k)+R(2, 3)*z(1, k)+P(2);
                zz1 = R(3, 1)*x(1, k)+R(3, 2)*y(1, k)+R(3, 3)*z(1, k)+P(3);
                xx2 = R(1, 1)*x(2, k)+R(1, 2)*y(2, k)+R(1, 3)*z(2, k)+P(1);
                yy2 = R(2, 1)*x(2, k)+R(2, 2)*y(2, k)+R(2, 3)*z(2, k)+P(2);
                zz2 = R(3, 1)*x(2, k)+R(3, 2)*y(2, k)+R(3, 3)*z(2, k)+P(3);
                x(1, k) =  xx1; x(2, k) =  xx2;
                y(1, k) =  zz1; y(2, k) =  zz2;
                z(1, k) =  yy1; z(2, k) =  yy2;
            end
            % 为变成实心封顶添加数据
            z2=[z(1, :); z; z(2, :)];
            x2=[x(1, :); x; x(2, :)];
            y2=[y(1, :); y; y(2, :)];
 
            z3=[z(1, :); z(1, :)];
            x3=[x(1, :); x(1, :)];
            y3=[y(1, :); y(1, :)];
    
            z4=[z(2, :); z(2, :)];
            x4=[x(2, :); x(2, :)];
            y4=[y(2, :); y(2, :)];
            
            surf(x2,z2,y2,'LineStyle','none','facealpha',0.5)
            map=jet(16);
            cl=4;%可设置16种颜色(1-16)
            colormap(map(cl,:))
            hold on
            surf(x3,z3,y3,'facealpha',0.5);
            hold on
            surf(x4,z4,y4,'facealpha',0.5);
            hold on
        end
        
        function [] = sdraw(obj, theta)
            R = cell(obj.JOINT_SIZE);
            P = cell(obj.JOINT_SIZE);
            T = obj.sfk_full(theta);
            P{1} = T{1}(1:3, 4);
            R{1} = T{1}(1:3, 1:3);
            for k=2:obj.JOINT_SIZE
                P{k} = T{k}(1:3, 4);
                R{k} = T{k}(1:3, 1:3);
            end

            obj.DrawCoordinate('0', P{1}, R{1});
            for k=2:obj.JOINT_SIZE
                obj.DrawLine(P{k-1}, P{k});  
                if obj.JOINT_TYPE(k-1) == 0
                    obj.DrawCylinder(P{k}, R{k});
                elseif obj.JOINT_TYPE(k-1) == 1
                    obj.DrawCube(P{k}, R{k});
                end
                % obj.DrawCoordinate('0'+k-1, P{k}, R{k});
            end
            obj.DrawCoordinate('e', P{obj.JOINT_SIZE}, R{obj.JOINT_SIZE});
            
            % 顯示參數
            %axis([-0.4 0.2 -0.3 0.3 -0.05 0.9]);
            axis equal;   % 顯示坐標軸比例
            %view(10,30); % 指定子图1的视点
            hold off
        end
        
        function [] = mdraw(obj, theta)
            R = cell(obj.JOINT_SIZE);
            P = cell(obj.JOINT_SIZE);
            T = obj.mfk_full(theta);
            P{1} = T{1}(1:3, 4);
            R{1} = T{1}(1:3, 1:3);
            for k=2:obj.JOINT_SIZE
                P{k} = T{k}(1:3, 4);
                R{k} = T{k}(1:3, 1:3);
            end

            obj.DrawCoordinate('0', P{1}, R{1});
            for k=2:obj.JOINT_SIZE
                obj.DrawLine(P{k-1}, P{k});  
                if obj.JOINT_TYPE(k-1) == 0
                    obj.DrawCylinder(P{k}, R{k});
                elseif obj.JOINT_TYPE(k-1) == 1
                    obj.DrawCube(P{k}, R{k});
                end
                obj.DrawCoordinate('0'+k-1, P{k}, R{k});
            end
            obj.DrawCoordinate('e', P{obj.JOINT_SIZE}, R{obj.JOINT_SIZE});
            
            % 顯示參數
            %axis([-0.4 0.2 -0.3 0.3 -0.05 0.9]);
            axis equal;   % 顯示坐標軸比例
            %view(10,30); % 指定子图1的视点
            hold off
        end
    end
end