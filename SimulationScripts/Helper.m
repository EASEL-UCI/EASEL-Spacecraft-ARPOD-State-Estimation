function helper_str = Helper(Metric, Phase1,Phase2,Phase3, fun, yolo_bool)

    numbers_str = "";
    if yolo_bool == true
        numbers_str = "& - & - & - & - & - & - & - & - & - ";
    else
        ekf_e = Metric{1}(Phase1{1});
        pf_e = Metric{2}(Phase1{2});
        mhe_e = Metric{3}(Phase1{3});
        num = fun(ekf_e);
        if num == 0
            numbers_str = numbers_str + " & " + 0;
        else
            numbers_str = numbers_str + " & " + round(num/10.^(floor(log(num)/log(10))),1)+"e"+floor(log(num)/log(10));
        end
        
        num = fun(pf_e);
        if num == 0
            numbers_str = numbers_str + " & " + 0;
        else
            numbers_str = numbers_str + " & " + round(num/10.^(floor(log(num)/log(10))),1)+"e"+floor(log(num)/log(10));
        end

        num = fun(mhe_e);
        if num == 0
            numbers_str = numbers_str + " & " + 0;
        else
            numbers_str = numbers_str + " & " + round(num/10.^(floor(log(num)/log(10))),1)+"e"+floor(log(num)/log(10));
        end

        ekf_e = Metric{1}(Phase2{1});
        pf_e = Metric{2}(Phase2{2});
        mhe_e = Metric{3}(Phase2{3});

        num = fun(ekf_e);
        if num == 0
            numbers_str = numbers_str + " & " + 0;
        else
            numbers_str = numbers_str + " & " + round(num/10.^(floor(log(num)/log(10))),1)+"e"+floor(log(num)/log(10));
        end
        num = fun(pf_e);
        if num == 0
            numbers_str = numbers_str + " & " + 0;
        else
            numbers_str = numbers_str + " & " + round(num/10.^(floor(log(num)/log(10))),1)+"e"+floor(log(num)/log(10));
        end
        num = fun(mhe_e);
        if num == 0
            numbers_str = numbers_str + " & " + 0;
        else
            numbers_str = numbers_str + " & " + round(num/10.^(floor(log(num)/log(10))),1)+"e"+floor(log(num)/log(10));
        end

        ekf_e = Metric{1}(Phase3{1});
        pf_e = Metric{2}(Phase3{2});
        mhe_e = Metric{3}(Phase3{3});

        num = fun(ekf_e);
        if num == 0
            numbers_str = numbers_str + " & " + 0;
        else
            numbers_str = numbers_str + " & " + round(num/10.^(floor(log(num)/log(10))),1)+"e"+floor(log(num)/log(10));
        end
        num = fun(pf_e);
        if num == 0
            numbers_str = numbers_str + " & " + 0;
        else
            numbers_str = numbers_str + " & " + round(num/10.^(floor(log(num)/log(10))),1)+"e"+floor(log(num)/log(10));
        end
        num = fun(mhe_e);
        if num == 0
            numbers_str = numbers_str + " & " + 0;
        else
            numbers_str = numbers_str + " & " + round(num/10.^(floor(log(num)/log(10))),1)+"e"+floor(log(num)/log(10));
        end
    end

    ekf_e = Metric{1};
    pf_e = Metric{2};
    mhe_e = Metric{3};

    num = fun(ekf_e);
    if num == 0
        numbers_str = numbers_str + " & " + 0;
    else
        numbers_str = numbers_str + " & " + round(num/10.^(floor(log(num)/log(10))),1)+"e"+floor(log(num)/log(10));
    end
    num = fun(pf_e);
    if num == 0
        numbers_str = numbers_str + " & " + 0;
    else
        numbers_str = numbers_str + " & " + round(num/10.^(floor(log(num)/log(10))),1)+"e"+floor(log(num)/log(10));
    end
    num = fun  (mhe_e);
    if num == 0
        numbers_str = numbers_str + " & " + 0;
    else
        numbers_str = numbers_str + " & " + round(num/10.^(floor(log(num)/log(10))),1)+"e"+floor(log(num)/log(10));
    end

    numbers_str = numbers_str + "\\" + "\\" + "\n";

    helper_str = numbers_str;
end