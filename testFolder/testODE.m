function final = testODE()
    %{
        Shows how to solve a 2nd order ODE using ode45.
        Normally, ode45 works for first-order ODE, but we can do it for
        second order by re-formulating our problem.

        y'' - y' + 3y = t is 2nd-order ODE. We want y(t)
        y(0) = 1
        y'(0) = -2

        right now f takes in t,y and outputs [ydot; ydotdot]
        using ode45 we first-order integrate [ydot; ydotdot] -> [y; ydot]

        basically reformulate diffeq to be in terms of ydot and ydotdot
    %}
    f = @(t,y) [y(2); t+y(2)-3*y(1)];
    t0 = 0;
    y0 = f(0,[1;-2]);

    T = 4;
    [ts,ys] = ode45(f, t0:0.5:T, y0);
    %ts is the timesteps created
    %ys(:,1) is the y that is integrated
    %ys(:,2) is the ydot that is integrated
    plot(ts,ys(:,1))
    title('Solution y(t) of IVP')
    xlabel('t'); grid on
    return
end

