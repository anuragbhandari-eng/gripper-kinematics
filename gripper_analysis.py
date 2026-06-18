"""
gripper_analysis.py
-------------------
Kinematic and mechanical-advantage analysis of a linear-actuator-driven,
parallel-jaw gripper (symmetric toggle linkage).

The mechanism: a linear actuator pushes a central slider along the horizontal
centerline (input, stroke = x). Two symmetric drive links connect the slider to
two jaws. The jaws ride on vertical guide rods, so they only TRANSLATE up and
down (no pivot) to open and close. Output is the half-opening y, the distance
from the centerline to one jaw's gripping surface.

1 degree of freedom: x sets the link angle theta, which sets the opening y.

Geometry (mm), read off the drawing and confirmed against the STEP:
    L  = 315.94   datum N -> gripping surface (overall horizontal span)
    L1 = 110.0    drive link, pin B -> pin C       (STEP: 110.0 mm exactly)
    L2 = 250.94   jaw horizontal arm, pin C -> corner D
    K  = 89.43    jaw vertical drop, corner D -> gripping surface E
    h  = 25.0     centerline -> slider pin B offset

CLOSED LIMIT: the actuator sets it, not the linkage. When the rod fully retracts
it bottoms against the cylinder, so the slider stops at a 63 mm jaw gap (y = 31.5
mm), NOT at y = 0. The gripper handles parts 63 - 91 mm wide; the linkage could
close further, but the actuator runs out of retract travel first.

Author: Anurag Bhandari
"""

import numpy as np
import matplotlib.pyplot as plt

# ----------------------------------------------------------------------
# 1. Geometry, loop constants, and the physical closed limit
# ----------------------------------------------------------------------
L  = 315.94   # mm
L1 = 110.0    # mm  (drive link)
L2 = 250.94   # mm
K  = 89.43    # mm
h  = 25.0     # mm

HORIZ_CONST = L - L2      # = 65.00 mm   -> horizontal equation constant
VERT_CONST  = K - h       # = 64.43 mm   -> vertical equation constant

MIN_GAP       = 63.0              # mm, closed gap (actuator fully retracted)
MIN_HALF_OPEN = MIN_GAP / 2.0     # = 31.5 mm, the real closed limit for y


# ----------------------------------------------------------------------
# 2. Kinematic model (re-derived and sign-checked from the geometry)
# ----------------------------------------------------------------------
# Horizontal loop:  jaw moves only vertically, so pin C's x is fixed:
#       cos(theta) = ( (L - L2) - x ) / L1
# Vertical loop:    y = h + L1*sin(theta) - K = L1*sin(theta) - (K - h)

def theta_from_x(x):
    """Link angle theta (rad) for actuator stroke x (mm)."""
    return np.arccos(np.clip((HORIZ_CONST - x) / L1, -1.0, 1.0))

def y_from_theta(theta):
    """Jaw half-opening y (mm) for link angle theta (rad)."""
    return L1 * np.sin(theta) - VERT_CONST

def theta_from_y(y):
    """Link angle theta (rad) needed to reach half-opening y (mm)."""
    return np.arcsin(np.clip((y + VERT_CONST) / L1, -1.0, 1.0))


# ----------------------------------------------------------------------
# 3. Mechanical advantage and clamp force
# ----------------------------------------------------------------------
# Virtual work on the loop -> MA = |dx/dy| = |tan(theta)|  (a toggle).
# Free-body of slider + one jaw, two links sharing the thrust:
#       F_link = F_a / (2 cos theta)      (axial, sizes the pins)
#       F_jaw  = (F_a / 2) * tan(theta)   (per-jaw clamp force)

def mechanical_advantage(theta):
    return np.abs(np.tan(theta))

def jaw_force(theta, f_actuator):
    return 0.5 * f_actuator * np.abs(np.tan(theta))

def link_force(theta, f_actuator):
    return f_actuator / (2.0 * np.cos(theta))


# ----------------------------------------------------------------------
# 4. Real operating window (closed gap -> toggle)
# ----------------------------------------------------------------------
THETA_CLOSED = theta_from_y(MIN_HALF_OPEN)   # ~60.7 deg at the 63 mm gap
THETA_OPEN   = np.radians(85.0)              # stop short of 90 (tan blows up)

X_CLOSED = HORIZ_CONST - L1 * np.cos(THETA_CLOSED)
X_OPEN   = HORIZ_CONST - L1 * np.cos(THETA_OPEN)
Y_MAX    = y_from_theta(np.radians(90.0))    # absolute max half-opening at toggle


def print_summary(f_actuator=200.0):
    print("=" * 66)
    print("GRIPPER OPERATING SUMMARY  (real window: jaws bottom out at 63 mm)")
    print("=" * 66)
    print(f"Drive link L1                 : {L1:.2f} mm")
    print(f"Closed limit (jaws bottom out): gap {MIN_GAP:.1f} mm  "
          f"(y={MIN_HALF_OPEN:.1f}, theta={np.degrees(THETA_CLOSED):.2f} deg)")
    print(f"Toggle (theta = 90 deg)       : gap {2*Y_MAX:.2f} mm  (y={Y_MAX:.2f})")
    print(f"=> Usable part range          : {MIN_GAP:.0f} - {2*Y_MAX:.0f} mm wide")
    print(f"Useful stroke (closed->open)  : {X_OPEN - X_CLOSED:.2f} mm")
    print(f"\nClamp force, actuator thrust = {f_actuator:.0f} N")
    print(f"{'part (mm)':>10}{'theta':>9}{'MA':>8}{'F_jaw (N)':>11}{'F_link (N)':>12}")
    for part in (63, 70, 80, 90):
        if part / 2.0 > Y_MAX:
            print(f"{part:>10}   wider than the jaws open")
            continue
        th = theta_from_y(part / 2.0)
        print(f"{part:>10}{np.degrees(th):>9.2f}{mechanical_advantage(th):>8.3f}"
              f"{jaw_force(th, f_actuator):>11.1f}{link_force(th, f_actuator):>12.1f}")
    print("=" * 66)


# ----------------------------------------------------------------------
# 5. Plots (swept over the real operating window only)
# ----------------------------------------------------------------------
def make_plots(f_actuator=200.0, save_prefix="gripper"):
    x = np.linspace(X_CLOSED, X_OPEN, 600)
    theta = theta_from_x(x)
    y = y_from_theta(theta)
    ma = mechanical_advantage(theta)
    opening = 2.0 * y
    f_jaw = jaw_force(theta, f_actuator)

    plt.rcParams.update({"figure.dpi": 130, "font.size": 11,
                         "axes.grid": True, "grid.alpha": 0.3})

    # Plot 1: stroke vs opening
    fig, ax = plt.subplots(figsize=(7, 4.3))
    ax.plot(x, opening, color="#1f5f8b", lw=2.2)
    ax.set_xlabel("actuator stroke  x  (mm)")
    ax.set_ylabel("jaw opening (gap)  2y  (mm)")
    ax.set_title("Stroke vs. jaw opening (operating window)")
    ax.annotate(f"closed\n{MIN_GAP:.0f} mm", xy=(X_CLOSED, MIN_GAP),
                xytext=(X_CLOSED + 6, MIN_GAP + 4),
                arrowprops=dict(arrowstyle="->", color="grey"))
    ax.annotate(f"max open\n{2*Y_MAX:.0f} mm", xy=(X_OPEN, opening[-1]),
                xytext=(X_OPEN - 20, opening[-1] - 6),
                arrowprops=dict(arrowstyle="->", color="grey"))
    fig.tight_layout(); fig.savefig(f"{save_prefix}_1_stroke_vs_opening.png")

    # Plot 2: stroke vs link angle
    fig, ax = plt.subplots(figsize=(7, 4.3))
    ax.plot(x, np.degrees(theta), color="#1f5f8b", lw=2.2)
    ax.set_xlabel("actuator stroke  x  (mm)")
    ax.set_ylabel("link angle  \u03b8  (deg)")
    ax.set_title("Stroke vs. link angle (never drops below ~61\u00b0)")
    ax.axhline(90, color="grey", ls=":", lw=1.2, label="90\u00b0 toggle")
    ax.axhline(np.degrees(THETA_CLOSED), color="#c44", ls="--", lw=1.2,
               label=f"{np.degrees(THETA_CLOSED):.0f}\u00b0 closed limit")
    ax.legend(loc="lower right")
    fig.tight_layout(); fig.savefig(f"{save_prefix}_2_stroke_vs_angle.png")

    # Plot 3: stroke vs mechanical advantage
    fig, ax = plt.subplots(figsize=(7, 4.3))
    ax.plot(x, ma, color="#1f5f8b", lw=2.2)
    ax.set_xlabel("actuator stroke  x  (mm)")
    ax.set_ylabel("mechanical advantage  MA = |tan \u03b8|")
    ax.set_title("The gripper lives in its power zone (MA never below ~1.8)")
    ax.axhline(mechanical_advantage(THETA_CLOSED), color="#c44", ls="--", lw=1.2,
               label=f"MA = {mechanical_advantage(THETA_CLOSED):.2f} at the closed limit")
    ax.legend(loc="upper left")
    fig.tight_layout(); fig.savefig(f"{save_prefix}_3_mechanical_advantage.png")

    # Plot 4: clamp force vs jaw opening
    fig, ax = plt.subplots(figsize=(7, 4.3))
    ax.plot(opening, f_jaw, color="#1f5f8b", lw=2.2)
    ax.set_xlabel("jaw opening (gap)  2y  (mm)")
    ax.set_ylabel(f"clamp force per jaw  (N)\nat {f_actuator:.0f} N actuator")
    ax.set_title("Clamp force across the usable part range (63 - 91 mm)")
    for part in (63, 70, 80):
        th = theta_from_y(part / 2.0)
        ax.plot(part, jaw_force(th, f_actuator), "o", color="#c44")
        ax.annotate(f"{part} mm\n{jaw_force(th, f_actuator):.0f} N",
                    xy=(part, jaw_force(th, f_actuator)),
                    xytext=(part - 2, jaw_force(th, f_actuator) + 60), fontsize=9)
    fig.tight_layout(); fig.savefig(f"{save_prefix}_4_clamp_force.png")

    plt.close("all")
    print("Saved 4 plots with prefix:", save_prefix)


if __name__ == "__main__":
    print_summary(f_actuator=200.0)
    make_plots(f_actuator=200.0, save_prefix="gripper")
