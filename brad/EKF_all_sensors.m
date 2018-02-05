function [x_n, P_n] = EKF_all_sensors(x_0, u, z, n, lengths, sAvail)
    x_n = [];
    P_n = [];
    delta_t = 1/100;
    
    x_prev = x_0';
    F = eye(6);
    B = delta_t*eye(6);
    P_prev = eye(6);
    Q= [0.02 0.02 0.01 0.01 0.01 0.01];
    %Q = [0.02 0 0 0 0 0; 0 0.02 0 0 0 0; 0 0 0.01 0 0 0; 0 0 0 0.01 0 0;
    %    0 0 0 0 0.01 0; 0 0 0 0 0 0.01];
    %R = 0.001*eye(9);
    R = 0.001*ones(1, 9);
    for i = 1 : n
      %%%%%%%%Predict%%%%%%%%%%%
      
      x_hat = F*(x_prev) + B*(u(i, :)');
      P_hat = F*P_prev*F' + diag(Q);
      
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