function [x_n, P_n] = EKF(x_0, u, z, n, lengths, sAvail)
% inputs:   x_0 = estimated beginning state
%           u_n = inputs/gyro readings
%           z_n = measurements/camera positions
%           Q = process noise co-variance
%           R = measurement noise co-variance
%           P_0 = estimated beginning state co-variance
%           f = function handle for non-linear state transitions/linear
%           model for angles
%           b = linear mapping of gyros to state
%           h = function handle for non-linear transform of state to
%              measurement (transform angle state to x, y, z)
%           n = number of states
%
% outputs:  y_n = kalman state estimate
%           error_n = kalman state co-variance

% states = x_n = [theta1, phi1, theta2, phi2, theta3, phi3]
% inputs = u_n = [theta1_dot, phi1_dot, theta2_dot, phi2_dot, theta3_dot, phi3_dot]
% process = F = I
% input process = b = [delta_t; ...]
% state to sensor = h = (linear kinematics)
%    
    
    x_n = [];
    P_n = [];
    delta_t = 1/100;
    
    x_prev = x_0';
    F = eye(6);
    B = delta_t*eye(6);
    P_prev = eye(6);
    Q= [0.02 0.02 0.01 0.01 0.01 0.01];
%     Q = [0.02 0 0 0 0 0; 0 0.02 0 0 0 0; 0 0 0.01 0 0 0; 0 0 0 0.01 0 0;
%         0 0 0 0 0.01 0; 0 0 0 0 0 0.01];
    %R = 0.001*eye(9);
    R = 0.001*ones(1, 9);
    for i = 1 : n
      %%%%%%%%Predict%%%%%%%%%%%
      
      x_hat = F*(x_prev) + B*(u(i, :)');
      P_hat = F*P_prev*F' + diag(Q);
      
      %%%%%%%%Update%%%%%%%%%%%%
      if(sAvail(i, 1) && sAvail(i, 2) && sAvail(i, 3))
        [H, hx] = generated_H(lengths(1), lengths(2), lengths(3), x_hat(1), x_hat(2), x_hat(3), x_hat(4), x_hat(5), x_hat(6));
        z_use = z(i, :)';
        R_use = R;
      elseif(sAvail(i, 1) && sAvail(i, 2) && ~sAvail(i, 3))
              [H, hx] = generated_H12(lengths(1), lengths(2),x_hat(1), x_hat(2), x_hat(3), x_hat(4));
               z_use = z(i, 1:2)';
               R_use = R(1:6);
      elseif(sAvail(i, 1) && ~sAvail(i, 2) && sAvail(i, 3))
              [H, hx] = generated_H13(lengths(1), lengths(2), lengths(3), x_hat(1), x_hat(2), x_hat(3), x_hat(4), x_hat(5), x_hat(6));        
              z_use = [z(i, 1) z(i: 3)]';
              R_use = [R(1:3) R(7:9)];
      elseif(~sAvail(i, 1) && sAvail(i, 2) && sAvail(i, 3))
              [H, hx] = generated_H23(lengths(1), lengths(2), lengths(3), x_hat(1), x_hat(2), x_hat(3), x_hat(4), x_hat(5), x_hat(6));        
              z_use = z(i, 2:3)';
              R_use = R(4:9);
      elseif(sAvail(i, 1) && ~sAvail(i, 2) && ~sAvail(i, 3))
              [H, hx] = generated_H1(lengths(1), x_hat(1), x_hat(2));
              z_use = z(i, 1)';
              R_use = R(1:3);
      elseif(~sAvail(i, 1) && sAvail(i, 2) && ~sAvail(i, 3))
              [H, hx] = generated_H2(lengths(1), lengths(2), x_hat(1), x_hat(2), x_hat(3), x_hat(4));        
              z_use = z(i, 2)';
              R_use = R(4:6);
      elseif(~sAvail(i, 1) && ~sAvail(i, 2) && sAvail(i, 3))
              [H, hx] = generated_H3(lengths(1), lengths(2), lengths(3), x_hat(1), x_hat(2), x_hat(3), x_hat(4), x_hat(5), x_hat(6));        
              z_use = z(i, 3)';
              R_use = R(7:9);
      end
              
      y = z_use - hx;
      S = H*P_hat*H' + diag(R_use);
      K = P_hat*H'*inv(S);
      x_prev = x_hat + K*y
      x_n = [x_n x_prev];
      P_prev = (eye(6) - K*H)*P_hat;
      P_n = [P_n ;P_prev];      
    end

    return
end