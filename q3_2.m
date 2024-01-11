% define geometric parameters
syms px py pz theta_1 theta_2 d3 

d2 = 2

equation1 = px == d2*sin(theta_1) + d3*cos(theta_1)*sin(theta_2);
equation2 = py == -d2*cos(theta_1) + d3*sin(theta_1)*sin(theta_2);
equation3 = pz == -d3*cos(theta_2);

solution = solve([equation1, equation2, equation3], [theta_1, theta_2, d3]);

simplify(solution.theta_1)
simplify(solution.theta_2)
simplify(solution.d3)