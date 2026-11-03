from GFLocalization import *
from EKF import *
from DR_3DOFDifferentialDrive import *
from DifferentialDriveSimulatedRobot import *

class EKF_3DOFDifferentialDriveInputDisplacement(GFLocalization, DR_3DOFDifferentialDrive, EKF):
    """
    This class implements an EKF localization filter for a 3 DOF Diffenteial Drive using an input displacement motion model incorporating
    yaw measurements from the compass sensor.
    It inherits from :class:`GFLocalization.GFLocalization` to implement a localization filter, from the :class:`DR_3DOFDifferentialDrive.DR_3DOFDifferentialDrive` class and, finally, it inherits from
    :class:`EKF.EKF` to use the EKF Gaussian filter implementation for the localization.
    """
    def __init__(self, kSteps, robot, *args):
        """
        Constructor. Creates the list of  :class:`IndexStruct.IndexStruct` instances which is required for the automated plotting of the results.
        Then it defines the inital stawe vecto mean and covariance matrix and initializes the ancestor classes.

        :param kSteps: number of iterations of the localization loop
        :param robot: simulated robot object
        :param args: arguments to be passed to the base class constructor
        """

        self.dt = 0.1  # dt is the sampling time at which we iterate the KF
        x0 = np.zeros((3, 1))  # initial state x0=[x y z psi u v w r]^T
        P0 = np.zeros((3, 3))  # initial covariance

        # this is required for plotting
        index = [IndexStruct("x", 0, None), IndexStruct("y", 1, None), IndexStruct("z", 2, 0), IndexStruct("yaw", 3, 1)]

        self.t_1 = 0
        self.t = 0
        self.Dt = self.t - self.t_1
        super().__init__(index, kSteps, robot, x0, P0, *args)

    def f(self, xk_1, uk):
        # TODO: Ok, falta revisar si codigo mejorable menos chat pero cuadra
        metres_per_pulse = 2 * np.pi * self.wheelRadius / self.robot.pulse_x_wheelTurns
        d_left, d_right = (uk * metres_per_pulse).flatten()
        distance = (d_left + d_right) / 2
        turn = (d_right - d_left) / self.wheelBase
        return Pose3D(xk_1).oplus(Pose3D(np.array([[distance], [0.0], [turn]])))

    def Jfx(self, xk_1):
        # TODO: REVISAR. p55??
        # Sensibilidad de la predicción frente a la pose anterior.
        scale = 2 * np.pi * self.wheelRadius / self.robot.pulse_x_wheelTurns
        distance = scale * (self.uk[0, 0] + self.uk[1, 0]) / 2
        yaw = xk_1[2, 0]
        return np.array([[1, 0, -distance * np.sin(yaw)],
                         [0, 1,  distance * np.cos(yaw)],
                         [0, 0, 1]])

    def Jfw(self, xk_1):
        # TODO: REVISAR. p55??
        # Sensibilidad frente al ruido de cada encoder, medido en pulsos.
        scale = 2 * np.pi * self.wheelRadius / self.robot.pulse_x_wheelTurns
        yaw = xk_1[2, 0]
        return scale * np.array([[np.cos(yaw) / 2, np.cos(yaw) / 2],
                                 [np.sin(yaw) / 2, np.sin(yaw) / 2],
                                 [-1 / self.wheelBase, 1 / self.wheelBase]])

    def h(self, xk):
        # La brújula observa solo la orientación.
        return xk[2:3, :]

    def GetInput(self):
        # Qk es la covarianza del ruido de los pulsos izquierdo y derecho.
        return self.robot.ReadEncoders()

    def GetMeasurements(self):
        # El contador del robot ya se ha incrementado en fs(); el primer paso es k=1.
        if self.robot.k % self.robot.yaw_reading_frequency != 0:
            return None, None, None, None
        yaw, variance = self.robot.ReadCompass()
        zk = np.array([[yaw]])
        Rk = np.array([[variance]])
        Hk = np.array([[0.0, 0.0, 1.0]])
        Vk = np.ones((1, 1))
        return zk, Rk, Hk, Vk


if __name__ == '__main__':

    M = [CartesianFeature(np.array([[-40, 5]]).T),
           CartesianFeature(np.array([[-5, 40]]).T),
           CartesianFeature(np.array([[-5, 25]]).T),
           CartesianFeature(np.array([[-3, 50]]).T),
           CartesianFeature(np.array([[-20, 3]]).T),
           CartesianFeature(np.array([[40,-40]]).T)]  # feature map. Position of 2 point features in the world frame.

    xs0 = np.zeros((6,1))  # initial simulated robot pose

    robot = DifferentialDriveSimulatedRobot(xs0, M)  # instantiate the simulated robot object
    kSteps = 5000

    xs0 = np.zeros((6, 1))  # initial simulated robot pose
    index = [IndexStruct("x", 0, None), IndexStruct("y", 1, None), IndexStruct("yaw", 2, 1)]

    x0 = np.zeros((3, 1))
    P0 = np.zeros((3, 3))

    dd_robot = EKF_3DOFDifferentialDriveInputDisplacement(kSteps,robot)  # initialize robot and KF
    dd_robot.LocalizationLoop(x0, P0, np.array([[0.5, 0.03]]).T)

    exit(0)