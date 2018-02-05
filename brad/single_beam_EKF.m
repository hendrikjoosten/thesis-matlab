function [x_n, P_n] = single_beam_EKF(x_0, u, z, n, lengths, sAvail)
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

% states = x_n = [theta1, phi1]
% inputs = u_n = [theta1_dot, phi1_dot]
% process = F = I
% input process = b = [delta_t; ...]
% state to sensor = h = (linear kinematics)
%    
    
    x_n = [];
    P_n = [];
    delta_t = 1/100;
    update = 0;
    x_prev = x_0';
    F = eye(2);
    B = delta_t*eye(2);
    P_prev = eye(2);
    Q= [0.02 0.02];
%     Q = [0.02 0 0 0 0 0; 0 0.02 0 0 0 0; 0 0 0.01 0 0 0; 0 0 0 0.01 0 0;
%         0 0 0 0 0.01 0; 0 0 0 0 0 0.01];
    %R = 0.001*eye(9);
    R = 0.001*ones(1, 3);
    for i = 1 : n
      %%%%%%%%Predict%%%%%%%%%%%
      
      x_hat = F*(x_prev) + B*(u(i, :)');
      P_hat = F*P_prev*F' + diag(Q);
      
      %%%%%%%%Update%%%%%%%%%%%%
      if(sAvail(i) == 1)
        [H, hx] = generated_H1(lengths(1), x_hat(1), x_hat(2));
        hx = [-10; 60; 45];
        z_use = z(i)';
        R_use = R;
        update = 1;         
      end
      if(update)
          y = z_use - hx;
          S = H*P_hat*H' + diag(R_use);
          K = P_hat*H'*inv(S);
          x_prev = x_hat + K*y
          x_n = [x_n x_prev];
          P_prev = (eye(6) - K*H)*P_hat;
          P_n = [P_n ;P_prev];      
      else
          x_prev = x_hat;
          x_n = [x_n x_prev];
          P_prev = P_hat;
          P_n = [P_n ;P_prev];  
      end
    end

    return
end