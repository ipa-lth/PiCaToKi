# PiCaToKi
Python-Calculation Tool Kit for control system and automation  
this repository will include a set of functions for control system analysis using Linear Matrix Inqualities (LMIs)  
The set of functions will cover stability analysis, observer design, controller desing, simulation examples, amog other
# Requirements
Python 2.7X  
PICOS   
install solvers mosek, sdpda, cvxopt, ...  
# Authors
F. R. López-Estrada  
G. Valencia-palomo  
S. Gómez-Peñate  

# List of functions  
  
lin_lyap(A): Linear Lyapunov stability test  
lin_obsv(A, C, solv) : Linear Luenberger observer  
lin_fbcon(A,B,solv): Linear feedback controoller U=-K*x  

# Examples
  
The state-space matrices for an aircraft model are [1]
A=np.array([[-0.01357, -32.2, -46.3, 0],[ 0.00012,0,1.2140, 0],[-0.0001212,0,-1.2140, 1], [0.00057, 0, -9.1, -0.6696]])  
B= np.array([ [-0.4330],[0.1394],[-0.1394],[-0.1577]]);  
C=[[0,0,0,1],[1,0,0,0]]  

lin_fbcon(A,B,"mosek")

#stability test 
lin_lyap(A)
#wiil display and found a matrix P=P.T>0, 
  
---------------------  
	optimization problem  (SDP):  
	14 variables, 0 affine constraints, 20 vars in 2 SD cones  
  
	Q 	: (1, 4), continuous  
	P 	: (4, 4), symmetric  
  
	maximize trace( P )  
	such that  
  	P*A.T -Q.T*B.T + A*P -B*Q ≼ |0|  
  	P ≽ |0|  
  
---------------------  

optimal matrix P:  
[[ 2.29907528 -0.11994877  0.09865076  0.50102559]  
 [-0.11994877  0.04704208 -0.02642021 -0.11385623]  
 [ 0.09865076 -0.02642021  0.0256783   0.05212997]    
 [ 0.50102559 -0.11385623  0.05212997  0.33067704]]  
   
 #Luenberger observer gain with mosek solver  
 lin_obsv(A, C, "mosek")  
 #Linear feedback controller with sdpa solver    
 lin_fbcon(A,B,"sdpa")  
 
 
 
 
 #[1] Z. Gajic and M. Lelic, Modern Control Systems Engineering, London, U.K.: Prentice Hall, 1996.
