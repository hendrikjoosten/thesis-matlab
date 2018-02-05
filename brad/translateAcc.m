function [transAcc] = translateAcc(Acc)
    transAcc = [];
    transAcc = [transAcc, (Acc(:,1).*-9.8 - 0.1)];
    transAcc = [transAcc, (Acc(:,2).*9.8 -0.6)];
    transAcc = [transAcc, (Acc(:,3).*-9.8)];
    return
end