function cbf = defineCbf(~, params, symbolic_state)
    x = symbolic_state;
    p_x = x(1); p_y = x(2); theta = x(3);

    v = params.v;
    xo1 = params.xo1;
    yo1 = params.yo1;
    xo2 = params.xo2;
    yo2 = params.yo2;
    d = params.d;

    distance1 = (p_x - xo1)^2 + (p_y - yo1)^2 - d^2;
    derivDistance1 = 2*(p_x - xo1)*v*cos(theta) + 2*(p_y - yo1)*v*sin(theta);

    distance2 = (p_x - xo2)^2 + (p_y - yo2)^2 - d^2;
    derivDistance2 = 2*(p_x - xo2)*v*cos(theta) + 2*(p_y - yo2)*v*sin(theta);

    cbf = [derivDistance1 + params.cbf_gamma0 * distance1;
           derivDistance2 + params.cbf_gamma0 * distance2];
end