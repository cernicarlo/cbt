"""
MPC AUV stonefish
version : 0.1
author : sumer
"""

import cvxpy as cp
import numpy as np
import torch
from dataclasses import dataclass
from collections.abc import Iterable
from fossens.fossen19 import AUVFossen
from optctrl.utils import chronometer, quaternion_error

@dataclass
class default:
    Np : int = 20 # prediction horizon
    Nc : int = 15 # control horizon
    xmin : float = -np.inf
    ymin : float = -np.inf
    zmin : float = -np.inf
    phimin : float = -np.inf
    thetamin : float = -np.inf
    psimin : float = -np.inf
    qxmin: float = -np.inf
    qymin: float = -np.inf
    qzmin: float = -np.inf
    qwmin: float = -np.inf
    umin : float = -10
    vmin : float = -10
    wmin : float = -10
    pmin : float = -1.0
    qmin : float = -1.0
    rmin : float = -1.0
    xmax : float = np.inf
    ymax : float = np.inf
    zmax : float = np.inf
    phimax : float = np.inf
    thetamax : float = np.inf
    psimax : float = np.inf
    qxmax: float = np.inf
    qymax: float = np.inf
    qzmax: float = np.inf
    qwmax: float = np.inf
    umax : float = 10
    vmax : float = 10
    wmax : float = 10
    pmax : float = 1.0
    qmax : float = 1.0
    rmax : float = 1.0
    Fxmin : float = -100
    Fymin : float = -100
    Fzmin : float = -100
    Kmin : float = -10
    Mmin : float = -10
    Nmin : float = -10
    Fxmax : float = 100
    Fymax : float = 100
    Fzmax : float = 100
    Kmax : float = 10
    Mmax : float = 10
    Nmax : float = 10

class MPC():
    def __init__(self, 
                 model=None,
                 parameters=None,
                 ):
        self.param = [] if parameters == None else parameters
        assert isinstance(model, AUVFossen), "Invalid model type."
        self.model = model
        self._init_param()
        self._init_problem()
        self.cnt_solve = 0
        
        self.epsilon_A = 1e-3
        self.epsilon_base = 1e-4
        self.illcond_tresh = 100
        
        self.check_illcond = False
        

    def _init_param(self):
        
        self.Np = self.param["Np"] if "Np" in self.param else default.Np
        self.Nc = self.param["Nc"] if "Nc" in self.param else default.Nc
        self.xmin = self._get_xmin(defpar=False) if "xmin" in self.param else self._get_xmin(defpar=True)
        self.umin = self._get_umin(defpar=False) if "umin" in self.param else self._get_umin(defpar=True)
        self.xmax = self._get_xmax(defpar=False) if "xmin" in self.param else self._get_xmax(defpar=True)
        self.umax = self._get_umax(defpar=False) if "umin" in self.param else self._get_umax(defpar=True)

    def _get_xmin(self, defpar=True) -> np.ndarray:
        
        if defpar:
            xmin = np.array((
                [
                default.xmin, default.ymin, default.zmin, 
                default.qxmin, default.qymin, default.qzmin, default.qwmin,
                default.umin, default.vmin, default.wmin,
                default.pmin, default.qmin, default.rmin
                ]
            ))
        else:
            xmin = np.array((
                [
                self.param["xmin"], self.param["ymin"], self.param["zmin"], 
                self.param["qxmin"], self.param["qymin"], self.param["qzmin"], self.param["qwmin"],
                self.param["umin"], self.param["vmin"], self.param["wmin"],
                self.param["pmin"], self.param["qmin"], self.param["rmin"]
                ]
            ))              

        return xmin
    
    def _get_xmax(self, defpar=True) -> np.ndarray: 

        if defpar:
            xmax = np.array((
                [
                default.xmax, default.ymax, default.zmax,
                default.qxmax, default.qymax, default.qzmax, default.qwmax,
                default.umax, default.vmax, default.wmax,
                default.pmax, default.qmax, default.rmax
                ]
            ))

        else:          
            xmax = np.array((
                [
                self.param["xmax"], self.param["ymax"], self.param["zmax"],
                self.param["qxmax"], self.param["qymax"], self.param["qzmax"], self.param["qwmax"],
                self.param["umax"], self.param["vmax"], self.param["wmax"],
                self.param["pmax"], self.param["qmax"], self.param["rmax"]
                ]
            ))
             

        return xmax
    
    def _get_umin(self, defpar=True) -> np.ndarray:

        if defpar:
            umin = np.array((
                [default.Fxmin, default.Fymin, default.Fzmin,
                 default.Kmin, default.Mmin, default.Nmin]
            ))
        else:
            umin = np.array((
                [self.param["Fxmin"], self.param["Fymin"], self.param["Fzmin"],
                 self.param["Kmin"], self.param["Mmin"], self.param["Nmin"]]
            ))

        return umin
    
    def _get_umax(self, defpar=True) -> np.ndarray:

        if defpar:
            umax = np.array((
                [default.Fxmax, default.Fymax, default.Fzmax,
                 default.Kmax, default.Mmax, default.Nmax]
            ))
        else:
            umax = np.array((
                [self.param["Fxmax"], self.param["Fymax"], self.param["Fzmax"],
                 self.param["Kmax"], self.param["Mmax"], self.param["Nmax"]]
            ))

        return umax
    
    def _is_pd(self, matrix: np.ndarray) -> bool:    
        return True if np.all(np.linalg.eigvals(matrix) > 0) else False

    def _get_err(self, state: cp.Parameter, ref: cp.Parameter) -> cp.Parameter:
        x1, y1, z1, w1 = ref[3], ref[4], ref[5], ref[6]
        x2, y2, z2, w2 = -state[3], -state[4], -state[5], state[6] # conjugate state quat
        
        qx_err = w2 * x1 + x2 * w1 + y2 * z1 - z2 * y1
        qy_err = w2 * y1 - x2 * z1 + y2 * w1 + z2 * x1
        qz_err = w2 * z1 + x2 * y1 - y2 * x1 + z2 * w1
        qw_err = w2 * w1 - x2 * x1 - y2 * y1 - z2 * z1

        err_rest = ref - state
        
        return cp.hstack([err_rest[0:3], qx_err, qy_err, qz_err, qw_err, err_rest[7:13]])
        
    def _init_problem(self):
        
        x_i = torch.tensor([0,0,0,0,0,0,1,
                            0,0,0,0,0,0], dtype=torch.float64)
        u_i = torch.tensor([0,0,0,0,0,0], dtype=torch.float64)
        
        A, B = torch.autograd.functional.jacobian(self.model, inputs=(x_i, u_i))
        
        self.A = A.numpy()
        self.B = B.numpy()
        
        self.I = torch.eye(self.A.shape[0])
        
        [self.nx, self.nu] = tuple(self.B.shape)
        
        self.x = cp.Variable((self.nx, self.Np + 1))
        self.u = cp.Variable((self.nu, self.Nc)) 

        self.xi = cp.Parameter(self.nx)
        self.xr = cp.Parameter((self.nx, self.Np + 1))
        
        self.err = cp.Parameter((self.nx, self.Np + 1))
        
        self.Al = cp.Parameter((self.nx, self.nx), name="A")
        self.Bl = cp.Parameter((self.nx, self.nu), name="B")
        
        self.Al.value = self.A
        self.Bl.value = self.B

        # If controller performance is not good, try
        # - increasing Q values
        # - decreasing R values
        self.Q = np.diag([1,1,5,
                          10,10,10,10,
                          0.01,0.01,0.01,
                          10,10,10]) * 1e1
        
        self.R = np.eye(self.nu) * 1e-1

        self.Qt = self.Q * 1e3

        assert self.xmin.shape[0] == self.nx, f"xmin and nx shapes don't match, expected {self.xmin.shape}, got {self.nx}"
        assert self.xmax.shape[0] == self.nx, f"x_max and nx shapes don't match, expected {self.xmax.shape}, got {self.nx}"
        assert self.umin.shape[0] == self.nu, f"u_min and nu shapes don't match, expected {self.umin.shape}, got {self.nu}"
        assert self.umax.shape[0] == self.nu, f"u_max and nu shapes don't match, expected {self.umax.shape}, got {self.nu}"
        self.update_problem(self.Q, self.R, self.Qt)
    
    # @chronometer
    def update_problem(self, Q, R, Qt):

        assert self._is_pd(Q), "Q not positive definite."
        assert self._is_pd(R), "R not positive definite."
        assert self._is_pd(Qt), "Qt not positive definite."
        self.Q, self.R, self.Qt = Q, R, Qt

        self.objective = 0 
        self.constraints = [self.x[:,0] == self.xi] 

        assert self.Np >= self.Nc, f"Prediction horizon Np:{self.Np} should be greater than or \
                                    equal to control horizon Nc:{self.Nc}."
                                                            
        for i in range(self.Np): 
            self.objective += cp.quad_form((self._get_err(state=self.x[:,i], ref=self.xr[:,i])), self.Q)
            if i < self.Nc:
                self.objective += cp.quad_form(self.u[:,i], self.R)
                self.constraints += [self.x[:,i+1] == self.Al @ self.x[:,i] + self.Bl @ self.u[:,i]]
                for j in range(self.nx):
                    self.constraints += [self.xmin[j] <= self.x[j,i], self.x[j,i] <= self.xmax[j]]
                for j in range(self.nu):
                    self.constraints += [self.umin[j] <= self.u[j,i], self.u[j,i] <= self.umax[j]]   
            elif i >= self.Nc:
                self.constraints += [self.x[:,i+1] == self.Al @ self.x[:,i] + self.Bl @ self.u[:,self.Nc - 1]]
                for j in range(self.nx):
                    self.constraints += [self.xmin[j] <= self.x[j,i], self.x[j,i] <= self.xmax[j]]

        self.objective += cp.quad_form(self.x[:,self.Np] - self.xr[:,self.Np], self.Qt) 
        self.problem = cp.Problem(cp.Minimize(self.objective), self.constraints)

    def set_reference(self, ref):
        assert isinstance(ref, Iterable), f"ref is not iterable ref:{ref}" 
        assert len(ref) == self.nx, f"Incorrect ref shape, expected {self.nx}, got {len(ref)}"
        self.xr.value = np.tile(ref, (self.Np + 1, 1)).T

    def set_sinus_reference(self, ref):
        pass
    
    def set_helix_reference(self, ref):
        pass
    
    @chronometer
    def solve_problem(self, x0) -> np.ndarray:
        if not hasattr(MPC, '_u'):
            MPC._u = [0,0,0,0,0,0]
          
        if self.check_illcond:
            _, S, _ = torch.linalg.svd(A)
            condn = S.max() / S.min()
            if condn > self.illcond_tresh:
                self.epsilon_A = self.epsilon_base * (condn / self.illcond_tresh)
                print(f"Ill conditioning detected, cond num:{condn} with A: \n {A}")
            else: 
                self.epsilon_A = self.epsilon_base
            
        A, B = torch.autograd.functional.jacobian(self.model, inputs=(
            torch.tensor(x0, dtype=torch.float64), 
            torch.tensor(self._u, dtype=torch.float64)))
        A_regularized = A + self.epsilon_A * self.I
        self.Al.value = A_regularized.numpy()
        self.Bl.value = B.numpy()   
        self.xi.value = x0
        try:
            self.problem.solve()
            if (self.problem.status == "optimal"):
                self._u = self.u.value[:,0]
                return self.u.value[:,0]
        except Exception as e:        
            print(f"Exception due to non-optimal solution: {str(e)}. Zero applied!")
        print(f"Solution not optimal! Current status is {self.problem.status}. Zero applied!")
        return np.array([0.0,0.0,0.0,0.0,0.0,0.0])
            