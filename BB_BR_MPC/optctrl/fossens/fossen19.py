"""
Underwater vehicle dynamics
version : 0.1.9 
- adjusted batch shapes for own use
- input-output states as single tensor
"""

import torch
import numpy as np

from typing import Tuple

def diag(tensor):
    diag_matrix = tensor.unsqueeze(1) * torch.eye(len(tensor), device=tensor.device)
    return diag_matrix


def diag_embed(tensor):
    return torch.stack([diag(s_) for s_ in tensor]) if tensor.dim() > 1 else diag(tensor)

'''
Dynamics object
'''
class AUVFossen(torch.nn.Module):
    def __init__(self, config=None, device: str='cpu', grad: bool=False):
        super(AUVFossen, self).__init__()
        self.config = config
        self.dtype = torch.float64
        self.device = device
        self.requires_grad = grad
        self.nx = 13
        self.nu = 6
        self.register_buffer("gravity", torch.tensor(9.81, dtype=self.dtype, device=self.device, requires_grad=self.requires_grad))
        # masks/pads
        self.register_buffer("z", torch.tensor([0., 0., 1.], dtype=self.dtype, device=self.device, requires_grad=self.requires_grad))
        self.register_buffer("pad3x3", torch.zeros(3, 3, dtype=self.dtype, device=self.device, requires_grad=self.requires_grad))
        self.register_buffer("pad4x3", torch.zeros(4, 3, dtype=self.dtype, device=self.device, requires_grad=self.requires_grad))

        ## Skew matrix masks
        self.register_buffer("A", torch.tensor([[0., 0., 0.], [0., 0., 1.], [0., -1., 0.]], dtype=self.dtype, device=self.device, requires_grad=self.requires_grad))
        self.register_buffer("B", torch.tensor([[0., 0., -1.], [0., 0., 0.], [1., 0., 0.]], dtype=self.dtype, device=self.device, requires_grad=self.requires_grad))
        self.register_buffer("C", torch.tensor([[0., 1., 0.], [-1., 0., 0.], [0., 0., 0.]], dtype=self.dtype, device=self.device, requires_grad=self.requires_grad))

        self.init_param()

    '''
    initializes model & simulation parameters
    '''
    def init_param(self):
        dt = 0.05
        if "dt" in self.config:
            dt = self.config["dt"]
            assert isinstance(dt, float), "timestep must be float"
        self.dt = dt

        rk = 2
        if "rk" in self.config:
            rk = self.config["rk"]
            assert rk == 2 or rk == 4, f"invalid rk mode, expected 2 or 4, got {rk}"
        self.rk = rk

        mass = 100.
        if "mass" in self.config:
            mass = self.config["mass"]

        self.mass = torch.nn.Parameter(
            torch.tensor(mass, dtype=self.dtype, device=self.device),
            requires_grad=self.requires_grad)

        volume = 1.
        if "volume" in self.config:
            volume = self.config["volume"]

        self.volume = torch.nn.Parameter(
            torch.tensor(volume, dtype=self.dtype, device=self.device),
            requires_grad=self.requires_grad)

        cog = [0., 0., 0.]
        if "cog" in self.config:
            cog = self.config["cog"]

        self.cog = torch.nn.Parameter(
            torch.tensor([cog], dtype=self.dtype, device=self.device),
            requires_grad=self.requires_grad)

        cob = [0., 0., 0.2]
        if "cob" in self.config:
            cob = self.config["cob"]

        self.cob = torch.nn.Parameter(
            torch.tensor([cob], dtype=self.dtype, device=self.device),
            requires_grad=self.requires_grad)


        inertialKey = ["ixx", "iyy", "izz", "ixy", "ixz", "iyz"]
        inertialArg = dict(ixx=0.7301, iyy=0.7705, izz=0.8024, ixy=0.002048, ixz=0.003906, iyz=0.001800)
        if "inertial" in self.config:
            inertialArg = self.config["inertial"]
            for key in inertialKey:
                if key not in inertialArg:
                    raise AssertionError('Invalid moments of inertia')
        self.inertial = self.get_inertial(inertialArg)

        addedMass = np.zeros((6, 6))
        if "Ma" in self.config:
            addedMass = np.array(self.config["Ma"])
            assert (addedMass.shape == (6, 6)), "Invalid add mass matrix"
            
        self.addedMass = torch.tensor(addedMass, dtype=self.dtype, device=self.device, requires_grad=self.requires_grad)
        

        massEye = self.mass * torch.eye(3, dtype=self.dtype, device=self.device, requires_grad=self.requires_grad)
        massLower = self.mass * self.skew_sym(self.cog[..., None])[0]

        upper = torch.concat([massEye, -massLower], dim=1)
        lower = torch.concat([massLower, self.inertial], dim=1)

        self.rbMass = torch.concat([upper, lower], dim=0)
        self.mTot = self.rbMass + self.addedMass
        # print("self.mTot \n", self.mTot)
        self.invMtot = torch.nn.Parameter(
            torch.linalg.inv(self.mTot),
            requires_grad=self.requires_grad
        )
        self.mTot = torch.nn.Parameter(
            self.mTot,
            requires_grad=self.requires_grad
        )


        linDamp = [-70., -70., -700., -300., -300., -100.]
        if "linear_damping" in self.config:
            linDamp = self.config["linear_damping"]

        self.linDamp = torch.nn.Parameter(
                diag_embed(
                    torch.tensor(linDamp, dtype=self.dtype, device=self.device),
                    ),
            requires_grad=self.requires_grad)

        linDampFow = [0., 0., 0., 0., 0., 0.]
        if "linear_damping_forward" in self.config:
            linDampFow = self.config["linear_damping_forward"]

        self.linDampFow = torch.nn.Parameter(
                diag_embed(
                    torch.tensor(linDampFow, dtype=self.dtype, device=self.device)),
            requires_grad=self.requires_grad)
        


        quadDam = [-740., -990., -1800., -670., -770., -520.]
        if "quad_damping" in self.config:
            quadDam = self.config["quad_damping"]

        self.quadDamp = torch.nn.Parameter(
                diag_embed(
                    torch.tensor(quadDam, dtype=self.dtype, device=self.device)),
            requires_grad=self.requires_grad)
        # print("self.quadDamp \n", self.quadDamp)
    
        if "density" in self.config:
            self.register_buffer("density", torch.tensor(self.config["density"], dtype=self.dtype, device=self.device, requires_grad=self.requires_grad))
        else:
            self.register_buffer("density", torch.tensor(1000., dtype=self.dtype, device=self.device, requires_grad=self.requires_grad))

    '''
        Forward the state pose and velocity to the next one using the action u. It is assumed that
        steps is = 1.

        input:
        ------
            - x, torch.Tensor, the pose of the vehicle using quaternion representation.
                Shape [steps, 7]
            - v, torch.Tensor, the velocity vector of the vehicle.
                Shape [steps, 6]
            - u, torch.Tensor, the action applied on the vehicle.
                Shape [steps, 6]
            - rk, runage-Kuttah integration steps. (Default = 2)

        output:
        -------
            - x_next, torch.Tensor, the next pose of the vehicle.
                Shape [1, 7]
            - v_next, torch.Tensor, the next velocity of the vehicle.
    '''
    def forward(self, x: torch.Tensor, u: torch.Tensor):
        v = x[7:13]
        x = x[0:7]
        x_k1, v_k1 = self.x_dot(x, v, u)
        x_tmp = x_k1*self.dt
        v_tmp = v_k1*self.dt

        if self.rk == 2:
            x_k2, v_k2 = self.x_dot(x + x_tmp, v + v_tmp, u)
            x_tmp = (self.dt/2.)*(x_k1 + x_k2)
            v_tmp = (self.dt/2.)*(v_k1 + v_k2)
        elif self.rk == 4:
            x_k2, v_k2 = self.x_dot(x + x_tmp, v + v_tmp, u)
            x_tmp = x_k2*self.dt
            v_tmp = v_k2*self.dt
            x_k3, v_k3 = self.x_dot(x + x_tmp, v + v_tmp, u)
            x_tmp = x_k3*self.dt
            v_tmp = v_k3*self.dt
            x_k4, v_k4 = self.x_dot(x + x_tmp, v + v_tmp, u)
            x_tmp = self.dt * (x_k1/6 + x_k2/3 + x_k3/3 + x_k4/6)
            v_tmp = self.dt * (v_k1/6 + v_k2/3 + v_k3/3 + v_k4/6)

        x_return = self.norm_quat(x+x_tmp)
        y_return = v+v_tmp
        return torch.cat((x_return, y_return))

    '''
        Computes x_dot and v_dot that can be used after for integration. Steps should always be equal to
        1 in this model.

        input:
        ------
            - x, the pose of the vehicle, shape [7].
            - v, the velocity of the vehicle, shape [6].
            - u, the action applied on the vehicle, shape [6]

        output:
        -------
            - x_dot, the time derivative of the pose, shape [7]
            - v_dot, the time derivative of the velocity, aka the acceleration. shape [6]
    '''
    def x_dot(self, x: torch.Tensor, v: torch.Tensor, u: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        rotBtoI, tBtoI = self.body2inertial(x)
        jac = self.jacobian(rotBtoI, tBtoI) # shape is [k, 7, 6]

        xDot = torch.matmul(jac, v)
        vDot = self.acc(v, u, rotBtoI)

        return xDot, vDot

    '''
        Normalizes a batch of states using quaternions representation.

        input:
        ------
            - quatState: torch.Tensor. The state with a quaternions. It is
            assumed that the quaternion is [qx, qy, qz, qw] and are elements 3:7.
                shape [7]
    '''
    def norm_quat(self, quatState: torch.Tensor) -> torch.Tensor:
        quat = quatState[3:7].clone()
        norm = torch.linalg.norm(quat, dim=-1)
        quat = quat/norm
        quatState[3:7] = quat.clone()
        return quatState

    '''
        Computes the transformation allowing to transform a state from body to
        inertial frame.

        input:
        ------
            - pose: torch.Tensor. The se3 state with a quaternions representation. It is
            assumed that the quaternion is [qx, qy, qz, qw] and are elements 3:7.
                shape [7]
        
        output:
        -------
            - rotBtoI. The rotation matrix that transforms a position vector from Body to Inertia
                shape [3, 3]
            - tBtoI. Transformation changing a quaternion from Body to Inertial frame.
                shape [4, 3]
    '''
    def body2inertial(self, pose: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        quat = pose[3:7]
        x = quat[0]
        y = quat[1]
        z = quat[2]
        w = quat[3]
        r1 = torch.tensor([
            1 - 2 * (y**2 + z**2),
            2 * (x * y - z * w),
            2 * (x * z + y * w)], dtype=self.dtype, requires_grad=self.requires_grad, device=self.device)

        r2 = torch.tensor([
            2 * (x * y + z * w),
            1 - 2 * (x**2 + z**2),
            2 * (y * z - x * w)], dtype=self.dtype, requires_grad=self.requires_grad, device=self.device)

        r3 = torch.tensor([
            2 * (x * z - y * w),
            2 * (y * z + x * w),
            1 - 2 * (x**2 + y**2)], dtype=self.dtype, requires_grad=self.requires_grad, device=self.device)

        rotBtoI = torch.stack([r1, r2, r3], dim=0)

        rwt = torch.tensor([-x, -y, -z], dtype=self.dtype, requires_grad=self.requires_grad, device=self.device)
        rxt = torch.tensor([w, -z, y], dtype=self.dtype, requires_grad=self.requires_grad, device=self.device)
        ryt = torch.tensor([z, w, -x], dtype=self.dtype, requires_grad=self.requires_grad, device=self.device)
        rzt = torch.tensor([-y, x, w], dtype=self.dtype, requires_grad=self.requires_grad, device=self.device)

        tBtoI = 0.5 * torch.stack([rxt, ryt, rzt, rwt], dim=0)
        return rotBtoI, tBtoI

    '''
        Computes the jacobian that transforms a Pose from the body frame to the inertial frame.

        inputs:
        -------
            - rotBtoI, the rotation matrix from body to inertial. Shape [3, 3]
            - tBtoI, the transformation matrix for quaternions, shape [4, 3]

        output:
        -------
            - Jacobian, the jacobian to go from body to inertial.
                Shape [7, 6]
    '''
    def jacobian(self, rotBtoI: torch.Tensor, tBtoI: torch.Tensor) -> torch.Tensor:
        jacR1 = torch.concat([rotBtoI, self.pad3x3], dim=-1)
        jacR2 = torch.concat([self.pad4x3, tBtoI], dim=-1)

        return torch.concat([jacR1, jacR2], dim=-2)

    '''
        Computes the acceleration vector from the current velocity, the action and
        the vehicle orientation.

        input:
        ------
            - v, torch.Tensor, the velocity vector. Shape [6]
            - u, torch.Tensor, the action applied to the vehicle. Shape [6]
            - rotBtoI, torch.tensor, the rotation matrix. Shape [3, 3]

        output:
        -------
            - acc, torch.Tensor, the acceleration vector. Shape [6]
    '''
    def acc(self, v: torch.Tensor, u: torch.Tensor, rotBtoI: torch.Tensor) -> torch.Tensor:
        Dv = torch.matmul(self.damping(v), v)
        Cv = torch.matmul(self.coriolis(v), v)
        g = self.restoring(rotBtoI)
        rhs = u - Cv - Dv - g
        acc = torch.matmul(self.invMtot, rhs)
        return acc

    '''
        Computes restoring forces given the rotation matrix body to inertia.

        input:
        ------
            - rotBtoI, torch.Tensor, the rotation matrix from body to inertia.
                Shape [3, 3]

        output:
        -------
            - restoring forces, torch.Tensor, the restoring forces vector.
                Shape [6]
    '''
    def restoring(self, rotBtoI: torch.Tensor) -> torch.Tensor:
        fng = -self.mass * self.gravity * self.z
        fnb = self.volume * self.density * self.gravity * self.z
        rotItoB = torch.transpose(rotBtoI, -1, -2)

        fbg = torch.matmul(rotItoB, fng)
        fbb = torch.matmul(rotItoB, fnb)
        mbg = torch.linalg.cross(self.cog[0], fbg)
        mbb = torch.linalg.cross(self.cob[0], fbb)

        return -torch.concat([fbg+fbb, mbg+mbb], dim=-1)

    '''
        Computes the damping matrix given the current velocity vector.

        input:
        ------
            - v, torch.Tensor, the velocity vector.
                Shape [6]

        output:
        -------
            - D(v), torch.Tensor, the damping matrix.
                Shape [6, 6]
    '''
    def damping(self, v: torch.Tensor) -> torch.Tensor:
        D = -self.linDamp - (v * self.linDampFow)
        tmp = -torch.mul(self.quadDamp, torch.abs(v))
        return D + tmp

    '''
        Computes the coriolis matrix given the current velocity vector.

        input:
        ------
            - v, torch.Tensor, the velocity vector.
                Shape [6]

        output:
        -------
            - C(v), torch.Tensor, the coriolis matrix.
                Shape [6, 6]
    '''
    def coriolis(self, v: torch.Tensor) -> torch.Tensor:
        skewCori = torch.matmul(self.mTot[0:3, 0:3].clone(), v[0:3]) + \
                   torch.matmul(self.mTot[0:3, 3:6].clone(), v[3:6])
        s12 = - self.skew_sym(skewCori[..., None])

        skewCoriDiag = torch.matmul(self.mTot[3:6, 0:3].clone(), v[0:3]) + \
                       torch.matmul(self.mTot[3:6, 3:6].clone(), v[3:6])
        s22 = - self.skew_sym(skewCoriDiag[..., None])
        
        r1 = torch.concat([self.pad3x3, s12], dim=-1)
        r2 = torch.concat([s12, s22], dim=-1)
        return torch.concat([r1, r2], dim=-2)

    '''
    skew method for cross
    '''
    def skew_sym(self, vec: torch.Tensor) -> torch.Tensor:
        c1 = torch.matmul(self.A, vec)
        c2 = torch.matmul(self.B, vec)
        c3 = torch.matmul(self.C, vec)
        S = torch.concat([c1, c2, c3], dim=-1)

        return S

    '''
    prints model info
    '''
    def print_info(self):
        """Print the vehicle's parameters."""
        print("="*5, " Model Info ", "="*5)
        print('Mass: {} kg, Trainable: {}\n'.format(self.mass.detach().cpu().numpy(),
                                                    self.mass.requires_grad))
        print('Volume: {} m^3, Trainable: {}\n'.format(self.volume.detach().cpu().numpy(),
                                                       self.volume.requires_grad))
        print('M:\n{}\nTrainable: {}'.format(self.mTot.detach().cpu().numpy(),
                                             self.mTot.requires_grad))
        print('Linear damping:\n{}\nTrainable: {}\n'.format(self.linDamp.detach().cpu().numpy(),
                                                            self.linDamp.requires_grad))
        print('Quad. damping:\n{}\nTrainable: {}\n'.format(self.quadDamp.detach().cpu().numpy(),
                                                           self.quadDamp.requires_grad))
        print('Center of gravity:\n{}\nTrainable: {}\n'.format(self.cog.detach().cpu().numpy(),
                                                               self.cog.requires_grad))
        print('Center of buoyancy:\n{}\nTrainable: {}\n'.format(self.cob.detach().cpu().numpy(),
                                                                self.cob.requires_grad))

    '''
    given inertia dictionary, returns inertia matrix J
    '''
    def get_inertial(self, dict):
        # buid the inertial matrix
        ixx = dict['ixx']
        ixy = dict['ixy']
        ixz = dict['ixz']
        iyy = dict['iyy']
        iyz = dict['iyz']
        izz = dict['izz']

        row0 = torch.tensor([[ixx], [ixy], [ixz]], dtype=self.dtype, device=self.device, requires_grad=self.requires_grad)
        row1 = torch.tensor([[ixy], [iyy], [iyz]], dtype=self.dtype, device=self.device, requires_grad=self.requires_grad)
        row2 = torch.tensor([[ixz], [iyz], [izz]], dtype=self.dtype, device=self.device, requires_grad=self.requires_grad)

        inertial = torch.concat([row0, row1, row2], dim=-1)

        return inertial
