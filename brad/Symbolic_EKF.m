clc;clear

syms th1 th2 th3 ph1 ph2 ph3 dth1 dth2 dth3 dph1 dph2 dph3 x1 y1 z1 x2 y2 z2 x3 y3 z3 l1 l2 l3

    
    delta_t = 1/100;
    x_prev = [th1 ph1 th2 ph2 th3 ph3].';
    u = [dth1 dph1 dth2 dph2 dth3 dph3].';
    F = eye(6);
    B = delta_t*eye(6);
    P_prev = eye(6);
    Q = [0.02 0 0 0 0 0; 0 0.02 0 0 0 0; 0 0 0.01 0 0 0; 0 0 0 0.01 0 0;
        0 0 0 0 0.01 0; 0 0 0 0 0 0.01];
    R = 0.001*eye(9);
    z = [x1; y1; z1; x2; y2; z2; x3; y3; z3];
    
    %%%%%%%%%%PREDICT%%%%%%%%%%%%%
    x_hat = F*x_prev + B*u;
    P_hat = F*P_prev*F' + Q;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    hx = [l1*cos(x_hat(2))*cos(x_hat(1));
        l1*cos(x_hat(1))*sin(x_hat(2));
        -l1*sin(x_hat(1));
        l1*cos(x_hat(2))*cos(x_hat(2)) + l2*cos(x_hat(4))*cos(x_hat(3));
        l1*cos(th1)*sin(ph1) + l2*cos(th2)*sin(ph2);
        - l1*sin(x_hat(1)) - l2*sin(x_hat(3));
        l1*cos(x_hat(2))*cos(x_hat(1)) + l2*cos(x_hat(4))*cos(x_hat(3)) + l3*cos(x_hat(6))*cos(x_hat(5));
        l1*cos(x_hat(1))*sin(x_hat(2)) + l2*cos(x_hat(3))*sin(x_hat(4)) + l3*cos(x_hat(5))*sin(x_hat(6));
        - l1*sin(x_hat(1)) - l2*sin(x_hat(3)) - l3*sin(x_hat(5))];
    
    H = jacobian(hx, x_prev);
    
    %%%%%%%%%%MEASURE%%%%%%%%%%%%
    y = z - hx;
    S = H*P_hat*H' + R;
    K = P_hat*H'*inv(S);
    X = x_hat + K*y;
    P = (eye(6) - K*H)*P_hat;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    x_prev = X;
    P_prev = P;
    
%     for i = 1 : 2
%       %%%%%%%%Predict%%%%%%%%%%%
%       x_est = f*x_prev + b*u_n(i); %alternatively f(x_prev)
%       P = f*P_prev*f' + Q;
%       
%       %%%%%%%%Update%%%%%%%%%%%%
%       
%       H = jacobian(h, x_est);
%       S = H*P*H' + R;
%       K = P*H'*inv(S);
%       y = z(i) - h(x_est);
%       x_n(i) = x_est + K*y;
%       error_n(i) = (eye(3) - K*H)*P;
%       x_prev = x_n(i);
%       P_prev = error_n(i);
%     end
