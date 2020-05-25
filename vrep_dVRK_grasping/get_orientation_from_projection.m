function [R, theta] = get_orientation_from_projection(cRt, Ltip, cPt)
    t1 = cPt(1);
    t2 = cPt(2);
    t3 = cPt(3);
    
    A1 = +cRt(1, 1) * Ltip;
    B1 = -cRt(1, 2) * Ltip;
    
    A2 = +cRt(2, 1) * Ltip;
    B2 = -cRt(2, 2) * Ltip;
    
    A3 = +cRt(3, 1) * Ltip;
    B3 = -cRt(3, 2) * Ltip;
    
    D1 = A3 * t1 - A1 * t3;
    E1 = B1 * t3 - B3 * t1;
    F1 = A3 * B1 - A1 * B3;
    %sprintf('df1*f3-f1*df3 = %f cos + %f sin + %f', D1, E1, F1)
    
    D2 = A3 * t2 - A2 * t3;
    E2 = B2 * t3 - B3 * t2;
    F2 = A3 * B2 - A2 * B3;
    %sprintf('df2*f3-f2*df3 = %f cos + %f sin + %f', D2, E2, F2)
    
    % dg = A * cos(x)^2 + B * sin(x) * cos(x) + C * sin(x)^2 + D * cos(x) + E * sin(x)
    A = - D1 * E1 - D2 * E2;
    B = + D1 * D1 - E1 * E1 + D2 * D2 - E2 * E2;
    C = + D1 * E1 + D2 * E2;
    D = - E1 * F1 - E2 * F2;
    E = + D1 * F1 + D2 * F2;
    % [A, B, C, D, E, F]
    
    % let t = tan(x/2) instead of sin(x) and cos(x),
    a = A - D;
    b = 2 * E - 2 * B;
    c = 4 * C - 2 * A;
    d = 2 * B + 2 * E;
    e = A + D;
    
    %[a, b, c, d, e]
    % then solve a * t^4 + b * t^3 + c * t^2 + d * t + e = 0
    tan = roots([a, b, c, d, e]);
    [tan_size, ~] = size(tan);
    R = cell(tan_size, 1);
    theta = zeros(tan_size, 1);
    for i = 1:tan_size
        theta(i) = atan(tan(i)) * 2;
        R{i} = cRt * [cos(theta(i)), -sin(theta(i)), 0; sin(theta(i)), cos(theta(i)), 0; 0, 0, 1];
    end
end