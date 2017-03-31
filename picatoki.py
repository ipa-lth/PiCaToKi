import cvxopt as cvx
import picos as pic
import numpy as np
from numpy import linalg as LA



##LYAPUNOV STABILITY
def lin_lyap(A):
    # matrix A in nxn
    n=np.shape(A)
    print n
    F = pic.Problem()
    A=pic.new_param("A",A)

    P = F.add_variable('P',n,'symmetric')
    #objetive maximize the trace of P
    # if the objective is just to find a solution, then comment  the next line
    F.set_objective('max','I'|P)       #('I' | Z.real) works as well
    F.add_constraint(A.H*P+P*A<<0 )
    F.add_constraint(P>>0)
    print F

    F.solve(solver='mosek',verbose = 0)

    #print 'fidelity: F(P,Q) = {0:.4f}'.format(F.obj_value())

    print 'optimal matrix P:'
    P= np.array(P.value)
    A=np.array(A.value)
    print P
    print "\n the Eigenvalues of P are:"
    print LA.eigvals(P)
    print "\nSolution test"
    print LA.eigvals(np.transpose(A)*P+P*A)
    return P


## LINEAR LUENBERGER OBSERVER
def lin_obsv(A, C, solv):
    ## A, C are knwon matrices of the LTI system
    #solv: choose the solver for example mosek, sdpa, cvxopt
    # The solvers need to be installed separatelly
    # http://picos.zib.de/intro.html#solvers

    #size of matrices A and B
    [n,n] =np.shape(A)
    [p,n] =np.shape(C)
    #starting the optimization problem
    F = pic.Problem()

    #Add parameters and variables
    A=pic.new_param('A',A)

    C=pic.new_param('C',C)
    P = F.add_variable('P',(n,n),'symmetric')
    Q = F.add_variable('Q',(n,p))

    F.add_constraint(A.T*P-C.T*Q.T+P*A-Q*C<<0 )

    F.add_constraint(P>>0)
    #optimzation objective
    #by default the objetive is to maximize the trace of P
    #comment the next line if the objetive is just to find any solution
    F.set_objective('max','I'|P)
    print(F)
    #solving the LMI, with selected solver
    F.solve(solver=solv, verbose = 0)

    P= np.matrix(P.value)
    Q= np.matrix(Q.value)

    print('optimal matrix P:')
    print(P)
    print('matrix Q:')
    print(Q)
    print("\n the Eigenvalues of P are:")
    print(LA.eigvals(P))
    print("\nSolution test")
    #print(LA.eigvals(A.T*P+P*A))

    L=P.I*Q
    print('The gain matrix L is:')
    print(L)

    return L


## LINEAR FEEDBACK CONTROLLER

def lin_fbcon(A,B,solv):
    ## A, B are knwon matrices of the LTI system
    #solv: choose the solver for example mosek, sdpa, cvxopt
    # The solvers need to be installed separatelly
    # http://picos.zib.de/intro.html#solvers

    #size of matrices A and B
    [n,n] =np.shape(A)
    [n,m] =np.shape(B)
    #starting the optimization problem
    F = pic.Problem()

    #Add parameters and variables
    A = pic.new_param('A',A)

    B = pic.new_param('B',B)

    P = F.add_variable('P',(n,n),'symmetric')
    Q = F.add_variable('Q',(m,n))

    F.add_constraint(P*A.T-Q.T*B.T+ A*P-B*Q<<0 )

    F.add_constraint(P>>0)
    #optimzation objective
    #by default the objetive is to maximize the trace of P
    #comment the next line if the objetive is just to find any solution
    F.set_objective('max','I'|P)
    print(F)
    #solving the LMI, with selected solver
    F.solve(solver=solv, verbose = 0)

    P= np.matrix(P.value)
    Q= np.matrix(Q.value)

    print('optimal matrix P:')
    print(P)
    print('matrix Q:')
    print(Q)
    print("\n the Eigenvalues of P are:")
    print(LA.eigvals(P))
    print("\nSolution test")
    #print(LA.eigvals(A.T*P+P*A))

    K=Q*P.I
    print('The gain matrix K is:')
    print(K)

    return K
