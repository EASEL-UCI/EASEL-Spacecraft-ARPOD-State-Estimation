function [na] = testOpt()
    %define function to minimize
    f = @(x) x(1)*x(4)*( x(1)+x(2)+x(3) ) + x(3);
    
    %constraints
    function [c,ceq] = nlcon(x)
        c = 25.0 - x(1)*x(2)*x(3)*x(4);
        ceq = sum(x.^2) - 40;
    end
    
    x0 = [1;5;5;1];
    
    A = [];
    b = [];
    Aeq = [];
    beq = [];

    lb = 1.0*ones(4);
    lb = lb(:,1);
    ub = 5.0*ones(4);
    ub = ub(:,1);
    nonlincon = @nlcon;

    %fmincon(objective, guess, LinearInequalityA, LinearIneqaulityb,
    %LinearAeq, LinearBeq, lowerbound, upperbound, nonlcon); 
    disp(['Initial Value: ' num2str(f(x0))]);
    x = fmincon(f, x0, A, b, Aeq, beq, lb, ub, nonlincon);
    disp(['X1 Value: ' num2str(x(1))]);
    disp(['X2 Value: ' num2str(x(2))]);
    disp(['X3 Value: ' num2str(x(3))]);
    disp(['X4 Value: ' num2str(x(4))]);
    disp(['Final Value: ' num2str(f(x)) ]);
end