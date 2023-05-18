function cost = terminalcosts1v1(t, x, kT)
    global x1_w;
    global x4_w;
    global x1_c;
    global x4_c;
    cost = x1_w*(x(1) - x1_c)^2 + x4_w*(x(4)-x4_c)^2;
end