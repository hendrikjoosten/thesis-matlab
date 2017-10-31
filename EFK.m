
%%=================================================================
%%admin
%%=================================================================

clear all
close all
clc

%%=================================================================
%%Pre Process and import the data
%%=================================================================

preprocess;

%%=================================================================
%%Initial values of states and cov
%%=================================================================

create_equations;
create_PQR;

states = zeroes(42,42);

%%=================================================================
%%FILTER
%%=================================================================

N = 1800;




for i=1:1:N
    %%=================================================================
    %%Predict
    %%=================================================================
    
    H = [];
    h = [];
    R = [];

    %%=================================================================
    %%Update
    %%=================================================================
    
    if z01Avail(i) == 1
    
    end
    if z02Avail(i) == 1
    
    end
    if z03Avail(i) == 1
    
    end
    if z04Avail(i) == 1
    
    end
    if z05Avail(i) == 1
    
    end
    if z06Avail(i) == 1
    
    end
    if z07Avail(i) == 1
    
    end
    if z08Avail(i) == 1
    
    end
    if z09Avail(i) == 1
    
    end
    if z10Avail(i) == 1
    
    end
    if z11Avail(i) == 1
    
    end
    if z12Avail(i) == 1
    
    end
    if z13Avail(i) == 1
    
    end
    if z14Avail(i) == 1
    
    end
    if z15Avail(i) == 1
    
    end
    if z16Avail(i) == 1
    
    end
    if z17Avail(i) == 1
    
    end
    if z18Avail(i) == 1
    
    end
    if z19Avail(i) == 1
    
    end
    
    % Kalman Gain
    %K=(covP*H')/(H*covP*H'+diag(R));
    % Correction
    %states=states+K*(zk-h);
    % New covariance
    %covP=(I-K*H)*covP;
    % Store covariance for smoother
    %Pcov(:,:,i) = covP;
    %xEst(:,i)=states;

end


%%=================================================================
%%Measure
%%=================================================================




