# Kinematics and mechanical advantage of a linear-actuator parallel-jaw gripper.
# 1-DOF toggle linkage: stroke x sets link angle theta, which sets jaw opening y.
# Geometry in mm; L1 came off the STEP file, the rest off the drawing.

import numpy as np
import matplotlib.pyplot as plt

L, L1, L2, K, h = 315.94, 110.0, 250.94, 89.43, 25.0
HORIZ_CONST = L - L2          # 65.00 mm
VERT_CONST = K - h            # 64.43 mm

# Jaws bottom out at a 63 mm gap: the actuator hits its retract stop before the
# linkage would, so the closed limit is y = 31.5 mm, not 0.
MIN_GAP = 63.0
MIN_HALF_OPEN = MIN_GAP / 2


def theta_from_x(x):
    return np.arccos(np.clip((HORIZ_CONST - x) / L1, -1.0, 1.0))

def y_from_theta(theta):
    return L1 * np.sin(theta) - VERT_CONST

def theta_from_y(y):
    return np.arcsin(np.clip((y + VERT_CONST) / L1, -1.0, 1.0))

# MA = |tan theta| from virtual work. The two links split the thrust, so per jaw
# F = (F_act / 2) tan theta, and each link carries F_act / (2 cos theta).
def mechanical_advantage(theta):
    return np.abs(np.tan(theta))

def jaw_force(theta, f_act):
    return 0.5 * f_act * np.abs(np.tan(theta))

def link_force(theta, f_act):
    return f_act / (2 * np.cos(theta))


# Working window: closed at the 63 mm stop, open just short of the 90 deg toggle
# (cap at 85 so tan stays finite).
theta_closed = theta_from_y(MIN_HALF_OPEN)
theta_open = np.radians(85)
x_closed = HORIZ_CONST - L1 * np.cos(theta_closed)
x_open = HORIZ_CONST - L1 * np.cos(theta_open)
y_max = y_from_theta(np.radians(90))


def print_summary(f_act=200.0):
    print(f"closed  {MIN_GAP:.0f} mm gap   theta {np.degrees(theta_closed):.1f} deg")
    print(f"open    {2*y_max:.0f} mm gap   toggle at 90 deg")
    print(f"parts   63 to {2*y_max:.0f} mm    stroke {x_open - x_closed:.0f} mm\n")
    print(f"part   theta     MA   F_jaw  F_link   (F_act = {f_act:.0f} N)")
    for part in (63, 70, 80, 90):
        th = theta_from_y(part / 2)
        print(f"{part:>3}mm  {np.degrees(th):5.1f}  {mechanical_advantage(th):5.2f}"
              f"  {jaw_force(th, f_act):5.0f}  {link_force(th, f_act):5.0f}")


def make_plots(f_act=200.0, prefix="gripper"):
    x = np.linspace(x_closed, x_open, 600)
    theta = theta_from_x(x)
    opening = 2 * y_from_theta(theta)

    plt.rcParams.update({"figure.dpi": 130, "font.size": 11,
                         "axes.grid": True, "grid.alpha": 0.3})
    blue = "#1f5f8b"

    fig, ax = plt.subplots(figsize=(7, 4.3))
    ax.plot(x, opening, blue, lw=2.2)
    ax.set(xlabel="actuator stroke x (mm)", ylabel="jaw opening 2y (mm)",
           title="Stroke vs. jaw opening")
    fig.tight_layout(); fig.savefig(f"{prefix}_1_stroke_vs_opening.png")

    fig, ax = plt.subplots(figsize=(7, 4.3))
    ax.plot(x, np.degrees(theta), blue, lw=2.2)
    ax.axhline(90, color="grey", ls=":", lw=1.2)
    ax.set(xlabel="actuator stroke x (mm)", ylabel="link angle theta (deg)",
           title="Stroke vs. link angle")
    fig.tight_layout(); fig.savefig(f"{prefix}_2_stroke_vs_angle.png")

    fig, ax = plt.subplots(figsize=(7, 4.3))
    ax.plot(x, mechanical_advantage(theta), blue, lw=2.2)
    ax.axhline(mechanical_advantage(theta_closed), color="#c44", ls="--", lw=1.2,
               label=f"MA = {mechanical_advantage(theta_closed):.1f} at the closed stop")
    ax.legend()
    ax.set(xlabel="actuator stroke x (mm)", ylabel="MA = |tan theta|",
           title="MA stays in the strong half of the toggle")
    fig.tight_layout(); fig.savefig(f"{prefix}_3_mechanical_advantage.png")

    fig, ax = plt.subplots(figsize=(7, 4.3))
    ax.plot(opening, jaw_force(theta, f_act), blue, lw=2.2)
    for part in (63, 70, 80):
        th = theta_from_y(part / 2)
        f = jaw_force(th, f_act)
        ax.plot(part, f, "o", color="#c44")
        ax.annotate(f"{part} mm: {f:.0f} N", (part, f), (part - 1, f + 60), fontsize=9)
    ax.set(xlabel="jaw opening 2y (mm)",
           ylabel=f"clamp force per jaw (N) at {f_act:.0f} N",
           title="Clamp force across the usable part range")
    fig.tight_layout(); fig.savefig(f"{prefix}_4_clamp_force.png")

    plt.close("all")


if __name__ == "__main__":
    print_summary()
    make_plots()
