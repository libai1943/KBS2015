param BasicParameters{i in {1..11}};
param SixBoundaryValues{i in {1..6}};

param lw == BasicParameters[1];
param lf == BasicParameters[2];
param lr == BasicParameters[3];
param lb == BasicParameters[4];
param vmax == BasicParameters[5];
param vmin == BasicParameters[6];
param amax == BasicParameters[7];
param phymax == BasicParameters[8];
param wmax == BasicParameters[9];
param nfe == BasicParameters[10];
param nobs == BasicParameters[11];

var tf >= 0.1;
var hi = tf / (nfe - 1);
param obstacle_vertices{i in {1..nobs}, j in {1..4}, k in {1..2}};


var x{i in {1..nfe}};
var y{i in {1..nfe}};
var theta{i in {1..nfe}};
var v{i in {1..nfe}};
var a{i in {1..nfe}};
var phy{i in {1..nfe}};
var w{i in {1..nfe}};
var AX{i in {1..nfe}};
var AY{i in {1..nfe}};
var BX{i in {1..nfe}};
var BY{i in {1..nfe}};
var CX{i in {1..nfe}};
var CY{i in {1..nfe}};
var DX{i in {1..nfe}};
var DY{i in {1..nfe}};


minimize obj: 
tf;

s.t. Bound_terminal_time:
tf <= 50; 

s.t. DIFF_dxdt {i in {2..nfe}}:
x[i] = x[i-1] + hi * v[i-1] * cos(theta[i-1]);
s.t. DIFF_dydt {i in {2..nfe}}:
y[i] = y[i-1] + hi * v[i-1] * sin(theta[i-1]);
s.t. DIFF_dthetadt {i in {2..nfe}}:
theta[i] = theta[i-1] + hi * tan(phy[i-1]) * v[i-1] / lw;
s.t. DIFF_dvdt {i in {2..nfe}}:
v[i] = v[i-1] + hi * a[i-1];
s.t. DIFF_dphydt {i in {2..nfe}}:
phy[i] = phy[i-1] + hi * w[i-1];

s.t. RELATIONSHIP_AX {i in {1..nfe}}:
AX[i] = x[i] + (lf+lw) * cos(theta[i]) - lb*0.5 * sin(theta[i]);
s.t. RELATIONSHIP_AY {i in {1..nfe}}:
AY[i] = y[i] + (lf+lw) * sin(theta[i]) + lb*0.5 * cos(theta[i]);
s.t. RELATIONSHIP_BX {i in {1..nfe}}:
BX[i] = x[i] + (lf+lw) * cos(theta[i]) + lb*0.5 * sin(theta[i]);
s.t. RELATIONSHIP_BY {i in {1..nfe}}:
BY[i] = y[i] + (lf+lw) * sin(theta[i]) - lb*0.5 * cos(theta[i]);
s.t. RELATIONSHIP_CX {i in {1..nfe}}:
CX[i] = x[i] - lr * cos(theta[i]) + lb*0.5 * sin(theta[i]);
s.t. RELATIONSHIP_DX {i in {1..nfe}}:
DX[i] = x[i] - lr * cos(theta[i]) - lb*0.5 * sin(theta[i]);
s.t. RELATIONSHIP_CY {i in {1..nfe}}:
CY[i] = y[i] - lr * sin(theta[i]) - lb*0.5 * cos(theta[i]);
s.t. RELATIONSHIP_DY {i in {1..nfe}}:
DY[i] = y[i] - lr * sin(theta[i]) + lb*0.5 * cos(theta[i]);


########## 始终存在的限制，防止碰撞 ########## 
s.t. eq_PPoutsideABCD {ii in {1..nobs}, jj in {1..4}, i in {1..nfe}}:
abs((AX[i] - obstacle_vertices[ii,jj,1])*(BY[i] - obstacle_vertices[ii,jj,2]) - (AY[i] - obstacle_vertices[ii,jj,2])*(BX[i] - obstacle_vertices[ii,jj,1])) * 0.5 + abs((BX[i] - obstacle_vertices[ii,jj,1])*(CY[i] - obstacle_vertices[ii,jj,2]) - (BY[i] - obstacle_vertices[ii,jj,2])*(CX[i] - obstacle_vertices[ii,jj,1])) * 0.5 + abs((CX[i] - obstacle_vertices[ii,jj,1])*(DY[i] - obstacle_vertices[ii,jj,2]) - (CY[i] - obstacle_vertices[ii,jj,2])*(DX[i] - obstacle_vertices[ii,jj,1])) * 0.5 + abs((DX[i] - obstacle_vertices[ii,jj,1])*(AY[i] - obstacle_vertices[ii,jj,2]) - (DY[i] - obstacle_vertices[ii,jj,2])*(AX[i] - obstacle_vertices[ii,jj,1])) * 0.5 >= (lf+lr+lw)*lb + 0.01;

s.t. eq_AoutsidePRECTANGLE {ii in {1..nobs}, i in {1..nfe}}:
abs((obstacle_vertices[ii,1,1] - AX[i])*( obstacle_vertices[ii,2,2] - AY[i]) - (obstacle_vertices[ii,1,2] - AY[i])*(obstacle_vertices[ii,2,1] - AX[i])) * 0.5 + abs((obstacle_vertices[ii,2,1] - AX[i])*(obstacle_vertices[ii,3,2] - AY[i]) - (obstacle_vertices[ii,2,2] - AY[i])*( obstacle_vertices[ii,3,1] - AX[i])) * 0.5 + abs((obstacle_vertices[ii,3,1] - AX[i])*( obstacle_vertices[ii,4,2] - AY[i]) - (obstacle_vertices[ii,3,2] - AY[i])*( obstacle_vertices[ii,4,1] - AX[i])) * 0.5 + abs((obstacle_vertices[ii,4,1] - AX[i])*( obstacle_vertices[ii,1,2] - AY[i]) - (obstacle_vertices[ii,4,2] - AY[i])*( obstacle_vertices[ii,1,1] - AX[i])) * 0.5 >= sqrt((obstacle_vertices[ii,1,1] - obstacle_vertices[ii,2,1])^2 + (obstacle_vertices[ii,1,2] - obstacle_vertices[ii,2,2])^2) * sqrt((obstacle_vertices[ii,1,1] - obstacle_vertices[ii,4,1])^2 + (obstacle_vertices[ii,1,2] - obstacle_vertices[ii,4,2])^2) + 0.01;

s.t. eq_BoutsidePRECTANGLE {ii in {1..nobs}, i in {1..nfe}}:
abs((obstacle_vertices[ii,1,1] - BX[i])*( obstacle_vertices[ii,2,2] - BY[i]) - (obstacle_vertices[ii,1,2] - BY[i])*(obstacle_vertices[ii,2,1] - BX[i])) * 0.5 + abs((obstacle_vertices[ii,2,1] - BX[i])*(obstacle_vertices[ii,3,2] - BY[i]) - (obstacle_vertices[ii,2,2] - BY[i])*( obstacle_vertices[ii,3,1] - BX[i])) * 0.5 + abs((obstacle_vertices[ii,3,1] - BX[i])*( obstacle_vertices[ii,4,2] - BY[i]) - (obstacle_vertices[ii,3,2] - BY[i])*( obstacle_vertices[ii,4,1] - BX[i])) * 0.5 + abs((obstacle_vertices[ii,4,1] - BX[i])*( obstacle_vertices[ii,1,2] - BY[i]) - (obstacle_vertices[ii,4,2] - BY[i])*( obstacle_vertices[ii,1,1] - BX[i])) * 0.5 >= sqrt((obstacle_vertices[ii,1,1] - obstacle_vertices[ii,2,1])^2 + (obstacle_vertices[ii,1,2] - obstacle_vertices[ii,2,2])^2) * sqrt((obstacle_vertices[ii,1,1] - obstacle_vertices[ii,4,1])^2 + (obstacle_vertices[ii,1,2] - obstacle_vertices[ii,4,2])^2) + 0.01;

s.t. eq_CoutsidePRECTANGLE {ii in {1..nobs}, i in {1..nfe}}:
abs((obstacle_vertices[ii,1,1] - CX[i])*( obstacle_vertices[ii,2,2] - CY[i]) - (obstacle_vertices[ii,1,2] - CY[i])*(obstacle_vertices[ii,2,1] - CX[i])) * 0.5 + abs((obstacle_vertices[ii,2,1] - CX[i])*(obstacle_vertices[ii,3,2] - CY[i]) - (obstacle_vertices[ii,2,2] - CY[i])*( obstacle_vertices[ii,3,1] - CX[i])) * 0.5 + abs((obstacle_vertices[ii,3,1] - CX[i])*( obstacle_vertices[ii,4,2] - CY[i]) - (obstacle_vertices[ii,3,2] - CY[i])*( obstacle_vertices[ii,4,1] - CX[i])) * 0.5 + abs((obstacle_vertices[ii,4,1] - CX[i])*( obstacle_vertices[ii,1,2] - CY[i]) - (obstacle_vertices[ii,4,2] - CY[i])*( obstacle_vertices[ii,1,1] - CX[i])) * 0.5 >= sqrt((obstacle_vertices[ii,1,1] - obstacle_vertices[ii,2,1])^2 + (obstacle_vertices[ii,1,2] - obstacle_vertices[ii,2,2])^2) * sqrt((obstacle_vertices[ii,1,1] - obstacle_vertices[ii,4,1])^2 + (obstacle_vertices[ii,1,2] - obstacle_vertices[ii,4,2])^2) + 0.01;

s.t. eq_DoutsidePRECTANGLE {ii in {1..nobs}, i in {1..nfe}}:
abs((obstacle_vertices[ii,1,1] - DX[i])*( obstacle_vertices[ii,2,2] - DY[i]) - (obstacle_vertices[ii,1,2] - DY[i])*(obstacle_vertices[ii,2,1] - DX[i])) * 0.5 + abs((obstacle_vertices[ii,2,1] - DX[i])*(obstacle_vertices[ii,3,2] - DY[i]) - (obstacle_vertices[ii,2,2] - DY[i])*( obstacle_vertices[ii,3,1] - DX[i])) * 0.5 + abs((obstacle_vertices[ii,3,1] - DX[i])*( obstacle_vertices[ii,4,2] - DY[i]) - (obstacle_vertices[ii,3,2] - DY[i])*( obstacle_vertices[ii,4,1] - DX[i])) * 0.5 + abs((obstacle_vertices[ii,4,1] - DX[i])*( obstacle_vertices[ii,1,2] - DY[i]) - (obstacle_vertices[ii,4,2] - DY[i])*( obstacle_vertices[ii,1,1] - DX[i])) * 0.5 >= sqrt((obstacle_vertices[ii,1,1] - obstacle_vertices[ii,2,1])^2 + (obstacle_vertices[ii,1,2] - obstacle_vertices[ii,2,2])^2) * sqrt((obstacle_vertices[ii,1,1] - obstacle_vertices[ii,4,1])^2 + (obstacle_vertices[ii,1,2] - obstacle_vertices[ii,4,2])^2) + 0.01;






s.t. EQ_init_x :
x[1] = SixBoundaryValues[1];
s.t. EQ_init_y :
y[1] = SixBoundaryValues[2];
s.t. EQ_init_theta :
theta[1] = SixBoundaryValues[3];
s.t. EQ_init_v :
v[1] = 0;
s.t. EQ_init_a :
a[1] = 0;
s.t. EQ_init_phy :
phy[1] = 0;
s.t. EQ_init_w :
w[1] = 0;

s.t. EQ_end_x :
x[nfe] = SixBoundaryValues[4];
s.t. EQ_end_y :
y[nfe] = SixBoundaryValues[5];
s.t. EQ_end_theta :
theta[nfe] = SixBoundaryValues[6];
s.t. EQ_end_v :
v[nfe] = 0;
s.t. EQ_end_phy :
phy[nfe] = 0;

s.t. Bounds_phy {i in {1..nfe}}:
-phymax <= phy[i] <= phymax;
s.t. Bounds_v {i in {1..nfe}}:
vmin <= v[i] <= vmax;
s.t. Bounds_w {i in {1..nfe}}:
-wmax <= w[i] <= wmax;
s.t. Bounds_a {i in {1..nfe}}:
-amax <= a[i] <= amax;

s.t. EQ_end_a :
a[nfe] = 0;
s.t. EQ_end_w :
w[nfe] = 0;

data;
param: BasicParameters := include AmplInputs/BasicParameters.txt;
param: SixBoundaryValues := include AmplInputs/BoundaryValues.txt;
param: obstacle_vertices := include AmplInputs/ObstacleVertices.txt;