function cbf = defineCbf(~, params, symbolic_state)
    x = symbolic_state;
    p_x = x(1); p_y = x(2); theta = x(3);

    v = params.v;
    d = params.d;

    xo1 = params.xo(1);
    yo1 = params.yo(1);
    distance1 = (p_x - xo1)^2 + (p_y - yo1)^2 - (d + params.r_rob)^2;
    derivDistance1 = 2*(p_x-xo1)*v*cos(theta) + 2*(p_y-yo1)*v*sin(theta);

    xo2 = params.xo(2);
    yo2 = params.yo(2);
    distance2 = (p_x - xo2)^2 + (p_y - yo2)^2 - (d + params.r_rob)^2;
    derivDistance2 = 2*(p_x-xo2)*v*cos(theta) + 2*(p_y-yo2)*v*sin(theta);
    cbf = [derivDistance1 + params.cbf_gamma0 * distance1;
           derivDistance2 + params.cbf_gamma0 * distance2]; 
end