function [x_new, P_new] = correctorZVK(x_pred, P_pred, a_b_m, om_b_m, mag_meas, mag_NED, zvk)

    quat_pred   = x_pred(1:4)';
    v_pred      = x_pred(5:7)';
    r_pred      = x_pred(8:10)';
    bias_a_pred = x_pred(11:13)';
    bias_g_pred = x_pred(14:16)';

    A   = [quat_pred(1)^2 - quat_pred(2)^2 - quat_pred(3)^2 + quat_pred(4)^2,               2*(quat_pred(1)*quat_pred(2) + quat_pred(3)*quat_pred(4)),                 2*(quat_pred(1)*quat_pred(3) - quat_pred(2)*quat_pred(4));
                 2*(quat_pred(1)*quat_pred(2) - quat_pred(3)*quat_pred(4)),      -quat_pred(1)^2 + quat_pred(2)^2 - quat_pred(3)^2 + quat_pred(4)^2,                2*(quat_pred(2)*quat_pred(3) + quat_pred(1)*quat_pred(4)) ;
                 2*(quat_pred(1)*quat_pred(3) + quat_pred(2)*quat_pred(4)),               2*(quat_pred(2)*quat_pred(3) - quat_pred(1)*quat_pred(4)),       -quat_pred(1)^2 - quat_pred(2)^2 + quat_pred(3)^2 + quat_pred(4)^2];

    % accelerometer correciton
    a_b = a_b_m' - bias_a_pred;  

    g_est = (A * [0, 0, -9.81]');
    g_meas = a_b;
    
    M = [0         -g_est(3)    g_est(2);
         g_est(3)   0          -g_est(1);
        -g_est(2)   g_est(1)    0];

    % magnetometer correction
    mag_meas =  mag_meas/norm(mag_meas);
    mag_NED = mag_NED/norm(mag_NED);

    z       = A*mag_NED;                   %Magnetic vector in the body axis (estimated)
    z_mat   = [ 0      -z(3)   z(2);
                z(3)     0     -z(1);
               -z(2)     z(1)   0;];        %Matrix needed to obtain the derivative H


    % Measurment matrix
    H_x = [zeros(3) eye(3)   zeros(3) zeros(3) zeros(3);
           zeros(3) zeros(3) zeros(3) zeros(3) -eye(3);
           M        zeros(3) zeros(3) zeros(3) zeros(3);
           z_mat    zeros(3) zeros(3) zeros(3) zeros(3)];


    S = H_x * P_pred * H_x' + zvk.R;

    K = P_pred * H_x' * inv(S);


    error = [zeros(6,1); g_meas; mag_meas'] - [v_pred; om_b_m'-bias_g_pred; g_est; z];

    update = K * error;

    
    x_new = zeros(1,16);
    x_new(5:end) = x_pred(5:end) + update(4:end)';

    quat_pred = x_pred(1:4);

    % quat_error = [update(1:3)'/2, 1];
    % 
    % quat_error(4)*quat_pred(4) - quat_error(1:3)*quat_pred(1:3)'
    
    % x_new(1:4) = [ (quat_pred(4)*quat_error(1:3) + quat_error(4)*quat_pred(1:3) - cross(quat_error(1:3),quat_pred(1:3)) )';
    %               quat_error(4)*quat_pred(4) - quat_error(1:3)*quat_pred(1:3)' ];


    OM = [ quat_pred(4), -quat_pred(3),  quat_pred(2);
           quat_pred(3),  quat_pred(4), -quat_pred(1);
          -quat_pred(2),  quat_pred(1),  quat_pred(4);
          -quat_pred(1), -quat_pred(2), -quat_pred(3)];

    quat_star = quat_pred' + 0.5 * OM * update(1:3);

    x_new(1:4) = quat_star / norm(quat_star);


    disp(rad2deg(quat2eul(x_new(1:4))))

    
    P_new = (eye(15) - K * H_x) * P_pred;


    disp(x_new)




    % r = [ 0.5*update(1:3), sqrt(1-0.25*update(1:3)*update(1:3)') ]; % scalar last
    % 
    % u = quatProd(r',x_pred(1:4)')';
    % 
    % x_new(1:4) = u / norm(u);
    % 
    % 
    % P_new = (eye(15) - K * H_x) * P_pred;
    % 
    % 
    % 
    % function quat = quatProd( quat1, quat2 )
    % %	Calculates the Hemiltonian product between two quaternions
    % %
    % %   quat = quatProd( quat1, quat2 )
    % %
    % %   This function compute the hamiltonian product between two quaternions
    % %   written as 4-by-1 vectors with the scalar component as the fourth
    % %   element of the vector.
    % %
    % %   References:
    % %	[1] Markley, F. Landis. "Attitude error representations for Kalman filtering." 
    % %       Journal of guidance control and dynamics 26.2 (2003): 311-317.
    % 
    % qv1 = quat1(1:3);
    % qs1 = quat1(4);
    % 
    % qv2 = quat2(1:3);
    % qs2 = quat2(4); % scalar last in the hamiltonian product
    % 
    % quat = [qs1 * qv2 + qs2 * qv1' - cross( qv1, qv2 )' ;
    %               qs1 * qs2 - dot( qv1, qv2 )        ];
    % 
    % quat = quat / norm(quat);
    % 
    % end

end