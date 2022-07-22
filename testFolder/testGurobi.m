function testGurobi()
    % max x + y + 2z
    % s.t.
    %   x + 2y + 3z <= 4
    %   x + y >= 1
    %   x,y,z binary
    names = {'x';'y';'z'};

    model.A = sparse([1,2,3;1,1,0]);
    model.obj = [1,1,2];
    model.rhs = [4;1];
    model.sense = '<>';
    model.vtype = 'B';
    model.modelsense = 'max';
    model.varnames = names;

    %gurobi_write(model, 'testGurobi.lp');
    params.outflag = 0;
    result = gurobi(model, params);
    disp(result);

    for v = 1:length(names)
        fprintf('%s %d\n', names{v}, result.x(v));
    end
    fprintf('Obj: %e\n', result.objval);
end