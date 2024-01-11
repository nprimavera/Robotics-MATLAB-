% define geometric parameters 
syms L_1 L_2 L_3 theta_1 theta_2 theta_3 XE YE PHI

% specify values for the link lengths of the robot
L1 = sym('x');
L2 = sym('y');
L3 = sym('z');

% define X, Y and PHI coordinates of the end-effector 
XE_RHS = L_1*cos(theta_1) + L_2*cos(theta_1 + theta_2) + L_3*cos(theta_1 + theta_2 + theta_3)
YE_RHS = L_1*sin(theta_1) + L_2*sin(theta_1 + theta_2) + L_3*sin(theta_1 + theta_2 + theta_3)
PHI_RHS = theta_1 + theta_2 + theta_3

% Find Inverse Kinematics 

% define the foward kinematics equation 
XE_EQ = XE == XE_RHS;
YE_EQ = YE == YE_RHS;
PHI_EQ = PHI == PHI_RHS;

% solve for theta1 and theta2
S = solve([XE_EQ YE_EQ PHI_EQ], [theta_1 theta_2 theta_3])

% show the pair of solutions for theta1 and theta2
simplify(S.theta_1)
simplify(S.theta_2)
simplify(S.theta_3)

% convert the solutions into Matlab functions you can use later 
TH1_MLF{1} = matlabFunction(S.theta_1(1),'Vars',[L_1 L_2 L_3 XE YE PHI]);
TH1_MLF{2} = matlabFunction(S.theta_1(2),'Vars',[L_1 L_2 L_3 XE YE PHI]);

TH2_MLF{1} = matlabFunction(S.theta_2(1),'Vars',[L_1 L_2 L_3 XE YE PHI]);
TH2_MLF{2} = matlabFunction(S.theta_2(2),'Vars',[L_1 L_2 L_3 XE YE PHI]);

TH3_MLF{1} = matlabFunction(S.theta_3(1),'Vars',[L_1 L_2 L_3 XE YE PHI]);
TH3_MLF{2} = matlabFunction(S.theta_3(2),'Vars',[L_1 L_2 L_3 XE YE PHI]);

TH1_MLF{1} (1, 1, 1, 1, 1, 0);
TH1_MLF{2} (1, 1, 1, 1, 1, 0);

TH2_MLF{1} (1, 1, 1, 1, 1, 0);
TH2_MLF{2} (1, 1, 1, 1, 1, 0);

TH3_MLF{1} (1, 1, 1, 1, 1, 0);
TH3_MLF{2} (1, 1, 1, 1, 1, 0);
