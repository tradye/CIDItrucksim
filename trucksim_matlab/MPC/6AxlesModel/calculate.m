function num = calculate()
    persistent a;
    global t_sim;
    if isempty(a)
        a = 0;
    end
    a = a+1;
    num = a;
    if (a > t_sim*100)
        a = 0;
    end
end