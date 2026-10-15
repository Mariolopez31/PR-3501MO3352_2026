from DifferentialDriveSimulatedRobot import *
from DR_3DOFDifferentialDrive import *
from GLDifferentialDrive import *
import numpy as np
from IndexStruct import *

np.set_printoptions(suppress=True)  # print matrices without scientific notation
plt.ion()  # enable interactive drawing mode

# feature map. Position of 2 point features in the world frame.
M2D = [np.array([[7, -6]]).T,
       np.array([[0, 9]]).T,
       np.array([[-7, -6]]).T]

kSteps = 5000  # number of simulation steps
index = [IndexStruct("x", 0, None), IndexStruct("y", 1, None),
         IndexStruct("yaw", 2, 1)]  # index of the state vector used for plotting

num_bins_x, num_bins_y, x_size, y_size = 21, 21, 10, 10
nCells = num_bins_x * num_bins_y
dx_max, dy_max = 1, 1
x_range, y_range = np.linspace(-x_size, x_size, num_bins_x), np.linspace(-y_size, y_size, num_bins_y)
cell_size = (x_range[-1] - x_range[0]) / num_bins_x
range_dx = range_dy = [-cell_size, 0, cell_size]

xs0 = np.array([[0, 0, 0, cell_size, 0, 3]]).T

robot = DifferentialDriveSimulatedRobot(xs0, M2D)  # instantiate the simulated robot object

x0 = Pose3D(np.array([[0, 0, 0]]).T)
p0 = Histogram2D(num_bins_x, num_bins_y, x_range, y_range)
# p0.element[0,0] = 1
p0.histogram_1d = np.ones(nCells) * 1 / (nCells ** 2)

grl = GL_3DOFDifferentialDrive(dx_max, dy_max, range_dx, range_dy, p0, index, kSteps, robot, x0)

uk = np.array([[4, 1]]).T

grl.LocalizationLoop(p0, uk)
exit(0)