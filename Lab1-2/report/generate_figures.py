import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
LAB = os.path.join(HERE, "..", "PR_LAB1")
FIG = os.path.join(HERE, "figures")
sys.path.insert(0, LAB)
os.makedirs(FIG, exist_ok=True)
os.chdir(LAB)  # VehicleIcon loads 'DifferentialDrive.png' from the cwd

from DifferentialDriveSimulatedRobot import DifferentialDriveSimulatedRobot  # noqa: E402
from DR_3DOFDifferentialDrive import DR_3DOFDifferentialDrive  # noqa: E402
from IndexStruct import IndexStruct  # noqa: E402
from Pose3D import Pose3D  # noqa: E402

SEED = 0                 # fixed seed used for every figure run
MC_SEEDS = range(30)     # seeds used for the error table
U_D, R_D = 0.5, 0.03
SIGMAS = [0, 11, 22, 44]
GAINS = [0.25, 0.5, 1.0, 1.5]
ORANGE, BLUE = "tab:orange", "tab:blue"


class GainRobot(DifferentialDriveSimulatedRobot):
    """Same fs() as the lab class, with the velocity gain K = k*I as a parameter
    (the lab code hard-codes K = I). Plotting is skipped."""

    k_gain = 1.0

    def fs(self, xsk_1, usk):
        eta_prev = Pose3D(xsk_1[0:3])
        nu_prev = xsk_1[3:6]
        nu_desired = np.array([[usk[0, 0]], [0.0], [usk[1, 0]]])
        w = np.random.multivariate_normal(np.zeros(3), self.Qsk).reshape(3, 1)
        eta_new = eta_prev.oplus(Pose3D(nu_prev * self.dt + 0.5 * w * self.dt**2))
        K = self.k_gain * np.eye(3)
        nu_new = nu_prev + K @ (nu_desired - nu_prev) + w * self.dt
        self.xsk = np.vstack((eta_new, nu_new))
        self.xsk_1 = xsk_1
        self.k += 1
        return self.xsk


def run(seed, sigma_e=22.0, k_gain=None):
    """Run the figure-eight; return true and estimated poses and the index
    where the first turn ends."""
    np.random.seed(seed)
    M2D = [np.array([[-40, 5]]).T]  # map only matters for plotting
    cls = DifferentialDriveSimulatedRobot if k_gain is None else GainRobot
    robot = cls(np.zeros((6, 1)), M2D)
    if k_gain is not None:
        robot.k_gain = k_gain
    robot.Re = np.diag([sigma_e**2, sigma_e**2])
    k_steps = round(2 * np.pi / (R_D * robot.dt))
    index = [IndexStruct("x", 0, None), IndexStruct("y", 1, None), IndexStruct("yaw", 2, 1)]
    x0 = Pose3D(np.zeros((3, 1)))
    dr = DR_3DOFDifferentialDrive(index, k_steps, robot, x0)
    robot.visualizationInterval = 10**9  # no animation

    true, est = [np.zeros(3)], [np.zeros(3)]
    xk_1 = x0
    for usk in (np.array([[U_D, R_D]]).T, np.array([[U_D, -R_D]]).T):
        xsk_1 = robot.xsk_1
        for _ in range(k_steps):
            xsk = robot.fs(xsk_1, usk)
            uk = dr.GetInput()
            xk_1 = dr.Localize(xk_1, uk)
            xsk_1 = xsk
            true.append(np.asarray(xsk)[0:3, 0].copy())
            est.append(np.asarray(xk_1)[:, 0].copy())
    plt.close("all")
    return np.array(true), np.array(est), k_steps


def final_errors(T, E, n1):
    return (np.linalg.norm(T[n1, :2] - E[n1, :2]),
            np.linalg.norm(T[-1, :2] - E[-1, :2]))


def comparison_figure(runs, titles, fname):
    fig, axs = plt.subplots(2, 4, figsize=(16, 7.5))
    for j, ((T, E, n1), title) in enumerate(zip(runs, titles)):
        for i, last in enumerate((n1 + 1, len(T))):
            ax = axs[i, j]
            ax.plot(T[:last, 0], T[:last, 1], c=ORANGE, lw=1.8, label="Simulated trajectory")
            ax.plot(E[:last, 0], E[:last, 1], c=BLUE, lw=1.4, label="Dead reckoning")
            ax.set_aspect("equal", adjustable="datalim")
            ax.grid(alpha=0.3)
            ax.set_xlabel("x [m]")
            ax.set_ylabel("y [m]")
        axs[0, j].set_title(title, fontsize=15)
    axs[0, 0].set_ylabel("First turn\ny [m]")
    axs[1, 0].set_ylabel("Complete figure-eight\ny [m]")
    axs[0, 0].legend(loc="center", fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, fname), dpi=150)
    plt.close(fig)


def main():
    # Sanity check: the GainRobot copy of fs() with k=1 equals the lab fs().
    T1, E1, _ = run(SEED, 22.0)
    T2, E2, _ = run(SEED, 22.0, k_gain=1.0)
    assert np.allclose(T1, T2) and np.allclose(E1, E2), "GainRobot.fs differs from lab fs"

    # Figure: simulated trajectory (Lab 1)
    T, _, _ = run(SEED, 22.0)
    fig, ax = plt.subplots(figsize=(6.5, 5.6))
    ax.plot(T[:, 0], T[:, 1], c=ORANGE, lw=1.8, label="Simulated trajectory")
    ax.scatter(*T[0, :2], c="green", zorder=3, label="Start")
    ax.scatter(*T[-1, :2], c="red", marker="x", zorder=3, label="End")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(alpha=0.3)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "simulated_robot_trajectory.png"), dpi=150)
    plt.close(fig)

    # Figure: velocity gain comparison (nominal encoder noise)
    comparison_figure([run(SEED, 22.0, k_gain=k) for k in GAINS],
                      [f"$k={k:g}$" for k in GAINS], "gain_trajectory_comparison.png")

    # Figure: encoder-noise comparison (K = I, lab fs)
    comparison_figure([run(SEED, float(s)) for s in SIGMAS],
                      [rf"$\sigma_e={s}$" for s in SIGMAS], "encoder_trajectory_comparison.png")

    # Table: final position error over several seeds
    print(f"Final position error over {len(MC_SEEDS)} seeds (mean +- std) [m]")
    print("sigma_e | after first turn | after figure-eight | figure run (seed %d)" % SEED)
    for s in SIGMAS:
        errs = np.array([final_errors(*run(seed, float(s))) for seed in MC_SEEDS])
        shown = final_errors(*run(SEED, float(s)))
        print(f"{s:7d} | {errs[:, 0].mean():6.2f} +- {errs[:, 0].std(ddof=1):5.2f} | "
              f"{errs[:, 1].mean():6.2f} +- {errs[:, 1].std(ddof=1):5.2f} | "
              f"{shown[0]:.2f}, {shown[1]:.2f}")


if __name__ == "__main__":
    main()
