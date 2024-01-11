% define geometric parameters 
syms L_1 L_2 theta_1 theta_2 XE YE 

% specify values for the link lengths of the robot
L1 = 1;
L2 = 0.5;

% specify values for the position of the end effector
XE = 1.5;
YE = 0;

% define X and Y coordinates of the end-effector 
XE_RHS = L_1*cos(theta_1) + L_2*cos(theta_1+theta_2)
YE_RHS = L_1*sin(theta_1) + L_2*sin(theta_1+theta_2)

% Find Inverse Kinematics 

% define the foward kinematics equation 
XE_EQ = XE == XE_RHS;
YE_EQ = YE == YE_RHS;

% solve for theta1 and theta2
S = solve([XE_EQ YE_EQ], [theta_1 theta_2])

% show the pair of solutions for theta1 and theta2
simplify(S.theta_1)
simplify(S.theta_2)

% convert the solutions into Matlab functions you can use later 
TH1_MLF{1} = ([L_1 L_2 XE YE]);
TH1_MLF{2} = ([L_1 L_2 XE YE]);
TH2_MLF{1} = ([L_1 L_2 XE YE]);
TH2_MLF{2} = ([L_1 L_2 XE YE]);

% substitute the values

simplify(S.theta_1);
S.theta_1 = subs(S.theta_1, L_1, 1);
S.theta_1 = subs(S.theta_1, L_2, 0.5);

simplify(S.theta_2);
S.theta_2 = subs(S.theta_2, L_1, 1);
S.theta_2 = subs(S.theta_2, L_2, 0.5);

simplify(S.theta_1);
S.theta_1 = subs(S.theta_1, XE, 1.5);
S.theta_1 = subs(S.theta_1, YE, 0);

simplify(S.theta_2);
S.theta_2 = subs(S.theta_2, XE, 1.5);
S.theta_2 = subs(S.theta_2, YE, 0);

% print answers 
disp('Theta 1');
S.theta_1
disp('Theta 2');
S.theta_2