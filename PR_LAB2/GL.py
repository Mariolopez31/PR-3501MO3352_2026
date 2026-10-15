from HF import HF
from Histogram import *


class GL(HF):
    """
    Grid Localization base class. Inherits from a HF. Implements the grid localization algorithm using a discrete Bayes Filter.
    """

    def __init__(self, p0, index, kSteps, robot, x0, *args):
        """
        Constructor of the :class:`GL` class. Initializes the Dead reckoning localization algorithm as well as the histogram filter algorithm.

        :param dx_max: maximum x displacement in meters
        :param dy_max: maximum y displacement in meters
        :param range_dx: range of x displacements in meters
        :param range_dy: range of y displacements in meters
        :param p0: initial probability histogram
        :param index: index struture containing plotting information
        :param kSteps: number of time steps to simulate the robot motion
        :param robot: robot object
        :param x0: initial robot pose
        :param args: additional arguments
        """
        super().__init__(p0, index, kSteps, robot, x0, *args)

        self.robot = robot

    def GetMeasurements(self):
        """
        Read the measurements from the robot. To be overriden by the child class.
        """
        pass

    def StateTransitionProbability_4_uk(self,uk):
        """
        Returns the state transition probability matrix for the given control input *uk*. It is used in the :meth:`Predict` method
        of the :class:`HF` class, to compute the predicted probability histogram.
        This is a pure virtual method that must be implemented by the derived class.

        :param uk: control input. In localization, this is commonly the robot displacement. For example, in the case of a differential drive robot, this is the robot displacement in the robot frame commonly computed through the odometry.
        :return: *Puk* state transition probability matrix for a given uk
        """
        pass

    def StateTransitionProbability(self):
        """
        Computes the complete state transition probability matrix.
        This is a pure virtual method that must be implemented by the derived class.

        :return: state transition probability matrix :math:`P_k=p{x_k|x_{k-1},uk}`
        """

        pass


    def MeasurementProbability(self, zk):
        """
        Computes the measurement probability histogram given the robot pose :math:`\eta_k` and the measurement :math:`z_k`.
        Method to be overriden by the child class.

        :param zk: vector of measurements
        :return: Measurement probability histogram :math:`p_z=p(z_k | \eta_k)`

        """

        pass

    def GetInput(self,usk):
        """
        Gets the number of cells the robot has displaced along its DOFs in the world N-Frame.
        Method to be overriden by the child class.

        :param usk: control input of the robot simulation. Required because it might be necessarey to call the :meth:SimulatedRobot.fs` method iterative until the robot displace at least one cell.
        :return: uk: vector containing the number of cells the robot has displaced in all the axis of the world N-Frame
        """

        pass

    def uk2cell(self, uk):
        """"
        Converts the number of cells the robot has displaced along its DOFs in the world N-Frame to an index that can be
        used to acces the state transition probability matrix.
        This is a pure virtual method that must be implemented by the derived class.

        :param uk: vector containing the number of cells the robot has displaced in all the axis of the world N-Frame
        :returns: index: index that can be used to access the state transition probability matrix
        """
        pass

    def LocalizationLoop(self, p0, usk):
        """
        Given an initial position histogram :math:`p_0` and the input to the :class:`DifferentialDrive.SimulatedRobot`
        this method calls iteratively :meth:`GL.Localize` for k steps, solving the robot localization problem.

        :param p0: initial robot pose
        :param usk: control input of the robot simulation

        """

        # TODO: To be implemented by the student

        pass

    def Localize(self, pxk_1, uk, zk):
        """
        Solves a localization iteration calling, successively to the :meth:`HF.Prediction` first, followed by the :meth:`HF.Update`.

        :param pxk_1: histogram of the previous robot position
        :param uk: robot displacement in number of cells in the world N-Frame
        :param zk: vector containing the measurements of the robot position in the world N-Frame
        :return: pk: histogram of the robot position after the prediction and the update steps
        """

        # TODO: To be implemented by the student

        pass




