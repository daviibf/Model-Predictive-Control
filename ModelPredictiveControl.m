%% LINEARIZING STATE SPACE MODEL %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Setting the variables with the same values of the first Homework
% conditions.
s = 0.0154 ;        % Section Area
sp = 5*10^(-5);     % Pipe Section Area
mi = 0.5;           % Flow Constant
mi2 = 0.675;        % Flow Constant
qmax = 1.2*10^(-4); % Max. Flow Rate
lmax = 0.6;         % Max. High
g = 9.80665;        % Gravity

% The constant values of Input Flow Rate to Find the Operating Points
q1 = (1/3)*qmax;
q2 = (2/7)*qmax;

%% Instantiating the EDOs that Describe the System

syms L1 L2 L3

f1 = (1/s)*(q1 - mi*sp*sign(L1 - L3)*sqrt(2*g)*sqrt(abs(L1 - L3)));
f2 = (1/s)*(q2 + mi*sp*sign(L3 - L2)*sqrt(2*g)*sqrt(abs(L3 - L2)) - mi2*sp*sqrt(2*g)*sqrt(L2) );
f3 = (1/s)*(mi*sp*sign(L1 - L3)*sqrt(2*g)*sqrt(abs(L1 - L3)) - mi*sp*sign(L3 - L2)*sqrt(2*g)*sqrt(abs(L3 - L2)) );

%% Calculating the Jacobian and Finding Matrix A

Ajacobiano = jacobian([f1 f2 f3],[L1 L2 L3]);

L1 = 0.508003919943708 ;
L2 = 0.246913405476143 ;
L3 = 0.377653414328398 ;

% Substituition into the Jacobian Matrix
AA = subs(Ajacobiano);

% Matrix A
A = double(AA);


%% Finding B Matrix for fixed Height Values.

% Steady Highs
L1 = 0.508003919943708 ; 
L2 = 0.246913405476143 ; 
L3 = 0.377653414328398 ;

syms q1 q2

f1 = (1/s)*(q1 - mi*sp*sign(L1 - L3)*sqrt(2*g)*sqrt(abs(L1 - L3)));
f2 = (1/s)*(q2 + mi*sp*sign(L3 - L2)*sqrt(2*g)*sqrt(abs(L3 - L2)) - mi2*sp*sqrt(2*g)*sqrt(L2) );
f3 = (1/s)*(mi*sp*sign(L1 - L3)*sqrt(2*g)*sqrt(abs(L1 - L3)) - mi*sp*sign(L3 - L2)*sqrt(2*g)*sqrt(abs(L3 - L2)) );

Bjacobiano = jacobian([f1 f2 f3],[q1 q2]);

% Substituition into the Jacobian Matrix
q1 = (1/3)*qmax;
q2 = (2/7)*qmax;

BB = subs(Bjacobiano);

%Matrix B
B = double(BB);

%% Setting Matrix C and D of the State Space Model

C = [1 0 0 ; 0 1 0 ; 0 0 1];

D = zeros(3,2);

%% MODEL PREDICTIVE CONTROL %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

modelPlant = 'MPC_Disturb';
load_system(modelPlant)
% open_system([modelPlant '/Three Tank Nonlinear System'],'force')

mdlMPC = 'MPC_Disturb';
open_system(mdlMPC)

%% Defining the State Space, and Setting the Parameters of MPC

% State Space 
sys = ss(A,B,C,D);

% Parameters of MPC
sys.InputName = {'q1','q2'};
sys.OutputName = {'L1','L2','L3'};
sys.StateName = {'L1','L2','L3'};
sys.InputGroup.MV = 2;
sys.OutputGroup.MO = [1 2];
sys.OutputGroup.UO = 3;

% Sample Time
Ts = 0.1; % 112.7 > Ts > 56.35

% Creating the MPC Object to import into Simulink
MpcMdl = mpc(sys, Ts);

% Defining Optimal Prediction Horizon and Control Horizon
MpcMdl.PredictionHorizon = 50; % 198 > T_PH > 99
MpcMdl.ControlHorizon = 10; %  0.2*T_PH = 39.36 > CH > 19.8 = 0.1*T_PH

%% Specify Constraints for MV and MV Rate
MpcMdl.MV(1).Min = 0;
MpcMdl.MV(1).Max = 0.00012;
MpcMdl.MV(2).Min = 0;
MpcMdl.MV(2).Max = 0.00012;

%% Specify Constraints for OV
MpcMdl.OV(1).Min = 0;
MpcMdl.OV(1).Max = 0.6;
MpcMdl.OV(2).Min = 0;
MpcMdl.OV(2).Max = 0.6;
MpcMdl.OV(3).Min = 0;
MpcMdl.OV(3).Max = 0.6;

%% Specify Weights
MpcMdl.Weights.MV = [1 1];
MpcMdl.Weights.MVRate = [100 100];
MpcMdl.Weights.OV = [1 1 1];
MpcMdl.Weights.ECR = 100000;

%% Specify Simulation Options
options = mpcsimopt();
options.RefLookAhead = 'off';
options.MDLookAhead = 'off';
options.Constraints = 'on';
options.OpenLoop = 'off';

%% Setpoint Tracking Parameters 
set_param("MPC_Disturb/Setpoint Tracking", 'Time', '0');
set_param("MPC_Disturb/Setpoint Tracking", 'Before', '0');
set_param("MPC_Disturb/Setpoint Tracking", 'After', '0.4');

%% Run Simulation
SimulT = 325 ; % Simulation Time
sim('MPC_Disturb', SimulT, []);

% Get the Simulation Time
tp = SimulMPC.Time ;

% Setpoint Tracking Plot with Noise
figure
plot(tp, SimulMPC.Data(:,1),'LineWidth',2)
hold on
plot(tp ,SimulMPC.Data(:,2),'LineWidth',2)
hold on
plot(tp, SimulMPC.Data(:,3),'LineWidth',2)
hold on
plot(tp, SimulMPC.Data(:,4),'--','LineWidth',2)
hold on
legend({'L1 with Noise','L2 with Noise','L3 with Noise','Set Point'},'Location','southeast','Orientation','Vertival')
hold off

MPC_Stepinfo = stepinfo(SimulMPC.Data(1:end,1:3),SimulMPC.Time, 0.4) ;