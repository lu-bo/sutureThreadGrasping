classdef rotation_and_axisAngle
    properties
    
    end
    
    methods (Static)
        
        function [theta,r] = Ro2Ax(RM)
            theta=acos((RM(1,1)+RM(2,2)+RM(3,3)-1)/2);
            r=(1/(2*sin(theta)))*[RM(3,2)-RM(2,3), RM(1,3)-RM(3,1), RM(2,1)-RM(1,2)];
        end
        
        function RM = Ax2Ro(theta,r)
            RM=[r(1)*r(1)+(1-r(1)^2)*cos(theta),            r(2)*r(1)*(1-cos(theta))-r(3)*sin(theta),   r(3)*r(1)*(1-cos(theta))+r(2)*sin(theta)
                r(1)*r(2)*(1-cos(theta))+r(3)*sin(theta),   r(2)*r(2)+(1-r(2)^2)*cos(theta),            r(3)*r(2)*(1-cos(theta))-r(1)*sin(theta)
                r(1)*r(3)*(1-cos(theta))-r(2)*sin(theta),   r(2)*r(3)*(1-cos(theta))+r(1)*sin(theta),   r(3)*r(3)+(1-r(3)^2)*cos(theta)         ];
        end
        
    end
    
end