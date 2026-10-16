"""
curb_height_model.py
====================

Forward kinematic estimator for a curb-climbing wheelchair.

GIVEN  the four joint encoders and the IMU
        ai   IMU tilt angle of the base                       [deg]
        atf  encoder tick reading, front caster arm           [tick]
        atd  encoder tick reading, driving wheel arm          [tick]
        atr  encoder tick reading, rear caster arm (upper)    [tick]
        xtd  encoder tick reading, carriage                   [tick]

RETURN
        ab_estimate   wheelchair base tilt angle              [deg]
        hc_estimate   curb height = height of the bottom of the front
                      caster wheel above the ground plane     [inch]

together with the intermediate joint angles, the second (non-physical)
solution branch, and a set of consistency diagnostics.

On the RAMMP prototype the inputs are the raw values that
MEBot_control_node keeps for every TELEMETRY line.  Section 6 has the full
mapping, the load-cell signals and the thresholds the curb sequences use.

        input  MEBot_control_node attribute     TELEMETRY  keyframe
        ai     imu_pitch (axis to be confirmed)     37        -
        atf    FC_pos                                4        1
        atd    ML_pos | MR_pos                     5 | 6     2 | 3
        atr    RC_pos                                3        0
        xtd    ML_carriage_pos | MR_carriage_pos   7 | 8     4 | 5
        hc     compare with neu_navigation CurbInfo.height  [metre]

The drive-wheel arm and the carriage exist once per side, so the 2-D model
is evaluated once per side (estimate_left_right).

--------------------------------------------------------------------------
GEOMETRY AND SIGN CONVENTIONS
--------------------------------------------------------------------------
All lengths are in inches, all angles in degrees (converted internally).

    Rf  radius of the front caster wheel
    Rd  radius of the driving wheel
    Rr  radius of the rear caster wheel
    Lf  front caster arm, front-wheel centre  ->  front hinge centre
    Ld  driving wheel arm, driving-wheel centre -> arm-swing hinge centre
    Lr  rear caster arm, rear-wheel centre -> rear hinge centre
    Lb  wheelchair base length, rear hinge -> front hinge
    yd  height of the carriage hinge above the base reference line
    yr  height of the rear hinge below the base reference line
    xd  carriage position measured backward from the front hinge, so the
        rear hinge -> carriage distance is Lb - xd (see section 6)

Positive joint angles:
    af  front caster arm rotated so that the front wheel rises relative
        to the base (the lifting direction used when mounting a curb)
    ad  driving wheel arm rotated so that the driving wheel moves down
        and forward from the carriage
    ar  rear caster arm rotated so that the rear hinge rises above the
        rear wheel
    ab  base pitched nose-up relative to the ground plane
    ac  curb face angle relative to the ground plane (0 = vertical face)

The ground plane is the line through the rear caster contact point and
the driving wheel contact point: those are the two wheels that stay on
the lower surface while the front caster is lifted onto the curb.

--------------------------------------------------------------------------
THE TWO EQUATIONS
--------------------------------------------------------------------------
1) Ground-contact closure.  Walking the loop
   rear-wheel centre -> rear hinge -> carriage -> driving-wheel centre
   and requiring that both wheels touch the same ground plane gives

        A * sin(ab) + B * cos(ab) = C

        A = Lr*cos(ar) + (Lb - xd) + Ld*cos(ad)     longitudinal offset
        B = Lr*sin(ar) + (yd - yr) - Ld*sin(ad)     vertical offset
        C = Rd - Rr                                 radius difference

   Equivalently, after expanding the products,

        Lr*sin(ab+ar) + Ld*sin(ab-ad)
            + (Lb-xd)*sin(ab) + (yd-yr)*cos(ab) = Rd - Rr

   which is the form used by closure_residual() below.  A*sin+B*cos=C has
   two solutions:

        ab_1 = asin(C/R) - atan2(B, A)              upright branch
        ab_2 = pi - asin(C/R) - atan2(B, A)         flipped-over branch
        R    = sqrt(A^2 + B^2)

   Only ab_1 is physically reachable; ab_2 lands near 160-180 deg, i.e.
   the chair rolled over its front wheels.  Both are returned so the
   caller can see the branch that was rejected.

2) Front caster height.  Walking the loop
   rear-wheel centre -> rear hinge -> front hinge -> front-wheel centre,
   projecting onto the ground-plane normal, and subtracting the wheel
   radii gives the curb height

        hc = Lr*sin(ab+ar) + Lf*sin(ab+af)
             + Lb*sin(ab) - yr*cos(ab) + Rr - Rf*cos(ac)

   hc is independent of ad and xd; those only enter through ab.  The
   sensitivity dhc/dab = (Lr+Lb+Lf)*pi/180 is about 0.7 inch per degree,
   so a 1 deg error in the base tilt shows up as roughly 0.7 inch of
   curb-height error.  Calibrating ab well matters more than anything
   else in this model.

--------------------------------------------------------------------------
UNITS OF THE CARRIAGE ENCODER
--------------------------------------------------------------------------
The carriage bench calibration is expressed in centimetres
(y[cm] = 0.002765 * tick + 1.931), while the geometry is in inches, so
Calibration.carriage_unit_in_inches converts between them.  Set it to 1.0
if you re-calibrate the carriage directly in inches.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace, asdict
from enum import IntEnum
from typing import List, Optional, Sequence, Tuple, Union

__all__ = [
    "Geometry",
    "Calibration",
    "JointState",
    "Estimate",
    "ticks_to_joints",
    "base_tilt",
    "curb_height",
    "closure_residual",
    "estimate",
    "estimate_batch",
    "fit_calibration",
    "DEFAULT_GEOMETRY",
    "DEFAULT_CALIBRATION",
    "BENCH_CALIBRATION",
    # section 6: mapping to the RAMMP prototype software
    "KeyframeIndex",
    "TelemetryIndex",
    "SignalSource",
    "MODEL_INPUT_SOURCES",
    "METRES_PER_INCH",
    "inch_to_m",
    "m_to_inch",
    "GuardCondition",
    "LoadCell",
    "LOAD_CELLS",
    "GUARDABLE_KEYFRAME_INDICES",
    "LOADCELL_IIR_ALPHA",
    "TELEMETRY_RATE_HZ",
    "FIRMWARE_LOOP_PERIOD_S",
    "FC_LOAD_ASCEND_CONTACT",
    "FC_LOAD_DESCEND_OFF_CURB",
    "LOADCELL_READING_FALLS_WITH_LOAD",
    "PROBE_GUARD_CONDITION",
    "PROBE_GUARD_THRESHOLD",
    "PROBE_RAISED_POSE",
    "PROBE_FC_TARGET",
    "PROBE_FC_DURATION_MS",
    "DESCEND_FC_GUARD_THRESHOLD",
    "ASCEND_MAX_CURB_HEIGHT_IN",
    "DESCEND_MAX_DROP_IN",
    "SideEstimates",
    "estimate_left_right",
    "estimate_from_telemetry",
]

DEG = math.pi / 180.0


# ===========================================================================
# 1.  Configuration objects
# ===========================================================================


@dataclass(frozen=True)
class Geometry:
    """Fixed mechanical dimensions of the chair, in inches.

    The defaults are the values from the 'Known' block of the original
    spreadsheet (diameters halved into radii).  Use `Geometry(...)` to build
    a new set from scratch, or `geom.set(Lb=25.0)` to change individual
    members; instances are immutable so a configuration cannot be mutated
    by accident halfway through a batch.
    """

    Rf: float = 6.000 / 2  # front caster wheel radius
    Rd: float = 14.094 / 2  # driving wheel radius
    Rr: float = 7.964 / 2  # rear caster wheel radius
    Lf: float = 8.265  # front caster arm length
    Ld: float = 12.562  # driving wheel arm length
    Lr: float = 8.265  # rear caster arm length
    Lb: float = 24.000  # base length, rear hinge to front hinge
    yd: float = 5.115  # carriage hinge height above base line
    yr: float = 1.000  # rear hinge height below base line

    # ---- derived quantities -------------------------------------------
    @property
    def C(self) -> float:
        """Right-hand side of the closure equation: the radius difference
        between the driving wheel and the rear caster."""
        return self.Rd - self.Rr

    @property
    def hc_per_deg_of_tilt(self) -> float:
        """Approximate dhc/dab in inch per degree, for error budgeting."""
        return (self.Lr + self.Lb + self.Lf) * DEG

    def set(self, **kwargs) -> "Geometry":
        """Return a copy with the named dimensions replaced.

        >>> DEFAULT_GEOMETRY.set(Lb=25.0, Rf=3.25).Lb
        25.0
        """
        unknown = set(kwargs) - set(asdict(self))
        if unknown:
            raise KeyError(f"unknown geometry parameter(s): {sorted(unknown)}")
        return replace(self, **kwargs)


@dataclass(frozen=True)
class Calibration:
    """Encoder tick -> joint coordinate conversions.

    Each joint uses an affine law

        angle[deg]    = slope * tick + offset
        carriage[cm]  = slope * tick + offset        (then converted to inch)

    The slope is a mechanical scale factor (gear ratio, encoder counts per
    revolution) and should be stable; the offset is the home-pose zero and
    moves every time an encoder is re-indexed, so it is the member that
    normally needs re-fitting.

    Defaults are the least-squares fit to the six measured poses (base-tilt
    residual RMS 0.21 deg, curb-height residual RMS 0.26 inch).  See
    BENCH_CALIBRATION for the raw bench-rig numbers.
    """

    af_slope: float = -0.052841  # deg per tick, front caster
    af_offset: float = -5.2636  # deg at tick 0
    ad_slope: float = 0.090012  # deg per tick, driving wheel arm
    ad_offset: float = 9.8295  # deg at tick 0
    ar_slope: float = 0.070871  # deg per tick, rear caster
    ar_offset: float = 13.5169  # deg at tick 0
    xd_slope: float = 0.002765  # cm per tick, carriage
    xd_offset: float = 40.0310  # cm at tick 0 (1.931 bench + 38.100 home)
    carriage_unit_in_inches: float = 1.0 / 2.54  # cm -> inch

    # IMU mapping: the IMU in the test data reads about +-180 deg when the
    # base is level, so the raw value is wrapped about `imu_level_reading`
    # and may need a sign flip to match the base-tilt sign convention.
    imu_level_reading: float = 180.0
    imu_sign: float = -1.0

    def set(self, **kwargs) -> "Calibration":
        """Return a copy with the named coefficients replaced."""
        unknown = set(kwargs) - set(asdict(self))
        if unknown:
            raise KeyError(f"unknown calibration parameter(s): {sorted(unknown)}")
        return replace(self, **kwargs)

    def as_equations(self) -> str:
        """Human-readable summary of the four conversion laws."""
        return (
            f"af = {self.af_slope:+.6f} * atf {self.af_offset:+.4f}   [deg]\n"
            f"ad = {self.ad_slope:+.6f} * atd {self.ad_offset:+.4f}   [deg]\n"
            f"ar = {self.ar_slope:+.6f} * atr {self.ar_offset:+.4f}   [deg]\n"
            f"xd = {self.xd_slope:+.6f} * xtd {self.xd_offset:+.4f}   [cm]"
        )


# Raw bench-rig calibration, kept for reference and for cross-checks.
# NOTE: the driving-wheel bench table is measurably non-linear (a quadratic
# fits it to 0.1 deg where the straight line leaves 0.3-0.46 deg), and its
# local slope over ticks 148-405 runs 0.081-0.122 deg/tick, so the
# table-wide 0.07478 should not be extrapolated past the table's range.
BENCH_CALIBRATION = Calibration(
    af_slope=-0.0506286,
    af_offset=45.46,
    ad_slope=0.0747800,
    ad_offset=-1.797,
    ar_slope=0.0587216,
    ar_offset=-1.351,
    xd_slope=0.0027650,
    xd_offset=1.931,
)

DEFAULT_GEOMETRY = Geometry()
DEFAULT_CALIBRATION = Calibration()


# ===========================================================================
# 2.  Result containers
# ===========================================================================


@dataclass
class JointState:
    """Physical joint coordinates after the tick conversion."""

    af: float  # front caster arm angle            [deg]
    ad: float  # driving wheel arm angle           [deg]
    ar: float  # rear caster arm angle             [deg]
    xd: float  # carriage position back from the front hinge [inch]


@dataclass
class Estimate:
    """Everything the forward model produces for one pose."""

    ab_estimate: float  # base tilt, upright branch              [deg]
    hc_estimate: float  # curb height, upright branch            [inch]

    joints: JointState  # converted encoder readings
    A: float  # closure-equation coefficients
    B: float
    C: float

    ab_branch2: float  # flipped-over solution, for inspection  [deg]
    hc_branch2: float  # curb height on that branch            [inch]

    imu_tilt: Optional[float] = None  # wrapped, sign-corrected IMU reading [deg]
    slope_estimate: Optional[float] = None
    # slope_estimate = imu_tilt - ab_estimate, and carries two readings of the
    # same number.  The IMU measures the base tilt in the world frame while
    # `ab` is the base tilt relative to the local ground plane, so on sloped
    # ground the difference is the ground slope.  On ground known to be flat
    # the slope is zero by construction, and the same number becomes a
    # consistency check on the calibration: it should come back near zero.

    reachable: bool = True  # False if |C/R| > 1, i.e. no real solution
    closure_residual: float = 0.0  # should be ~0; a sanity check      [inch]

    def __str__(self) -> str:
        s = (
            f"ab_estimate = {self.ab_estimate:8.3f} deg\n"
            f"hc_estimate = {self.hc_estimate:8.3f} in\n"
            f"  joints: af={self.joints.af:7.2f}  ad={self.joints.ad:7.2f}  "
            f"ar={self.joints.ar:7.2f} deg   xd={self.joints.xd:6.2f} in\n"
            f"  rejected branch: ab={self.ab_branch2:7.2f} deg, "
            f"hc={self.hc_branch2:6.2f} in"
        )
        if self.imu_tilt is not None:
            s += (
                f"\n  IMU: tilt={self.imu_tilt:6.2f} deg -> ground slope "
                f"(or, on flat ground, calibration error) "
                f"{self.slope_estimate:+.2f} deg"
            )
        if not self.reachable:
            s += "\n  WARNING: pose is not geometrically reachable (|C/R| > 1)"
        return s


# ===========================================================================
# 3.  Forward model, step by step
# ===========================================================================


def ticks_to_joints(
    atf: float,
    atd: float,
    atr: float,
    xtd: float,
    cal: Calibration = DEFAULT_CALIBRATION,
) -> JointState:
    """Convert the four raw encoder readings into joint coordinates.

    Returns angles in degrees and the carriage position in inches, measured
    backward from the front hinge, which is what the kinematics below
    expect (they use Lb - xd as the rear hinge -> carriage distance).
    """
    return JointState(
        af=cal.af_slope * atf + cal.af_offset,
        ad=cal.ad_slope * atd + cal.ad_offset,
        ar=cal.ar_slope * atr + cal.ar_offset,
        xd=(cal.xd_slope * xtd + cal.xd_offset) * cal.carriage_unit_in_inches,
    )


def closure_coefficients(
    joints: JointState, geom: Geometry = DEFAULT_GEOMETRY
) -> Tuple[float, float, float]:
    """Coefficients (A, B, C) of  A*sin(ab) + B*cos(ab) = C.

    A is the longitudinal distance from the rear caster centre to the
    driving wheel centre, measured along the base; B is the corresponding
    perpendicular distance; C is the difference of the two wheel radii.
    """
    A = (
        geom.Lr * math.cos(joints.ar * DEG)
        + (geom.Lb - joints.xd)
        + geom.Ld * math.cos(joints.ad * DEG)
    )
    B = (
        geom.Lr * math.sin(joints.ar * DEG)
        + (geom.yd - geom.yr)
        - geom.Ld * math.sin(joints.ad * DEG)
    )
    return A, B, geom.C


def base_tilt(
    joints: JointState, geom: Geometry = DEFAULT_GEOMETRY
) -> Tuple[float, float, bool]:
    """Solve the closure equation for the base tilt angle.

    Returns (ab_upright, ab_flipped, reachable), both angles in degrees.
    `reachable` is False when |C/R| > 1: the commanded joint combination
    cannot put both lower wheels on one plane, which in practice means a
    bad encoder reading or a wrong calibration offset.
    """
    A, B, C = closure_coefficients(joints, geom)
    R = math.hypot(A, B)
    ratio = C / R if R > 0.0 else 2.0
    reachable = abs(ratio) <= 1.0
    ratio = max(-1.0, min(1.0, ratio))  # clamp so asin stays defined

    phase = math.atan2(B, A)  # atan2, not atan(B/A): it is
    # correct for any sign of A,
    # and agrees with the sheet's
    # ATAN(B/A) whenever A > 0
    ab1 = (math.asin(ratio) - phase) / DEG  # upright branch
    ab2 = (math.pi - math.asin(ratio) - phase) / DEG  # flipped-over branch
    return ab1, ab2, reachable


def curb_height(
    ab_deg: float,
    joints: JointState,
    geom: Geometry = DEFAULT_GEOMETRY,
    ac_deg: float = 0.0,
) -> float:
    """Height of the bottom of the front caster above the ground plane.

    `ab_deg` is the base tilt, `ac_deg` the curb face angle relative to the
    ground plane (0 for a vertical face, which is the usual case).
    """
    ab = ab_deg * DEG
    return (
        geom.Lr * math.sin(ab + joints.ar * DEG)
        + geom.Lf * math.sin(ab + joints.af * DEG)
        + geom.Lb * math.sin(ab)
        - geom.yr * math.cos(ab)
        + geom.Rr
        - geom.Rf * math.cos(ac_deg * DEG)
    )


def closure_residual(
    ab_deg: float, joints: JointState, geom: Geometry = DEFAULT_GEOMETRY
) -> float:
    """Left minus right side of the closure equation, in inches.

    Zero when `ab_deg` is a true solution.  Two uses:

    * as a self-check on the solver (should come back at 1e-12);
    * as the residual to minimise when fitting a calibration against a
      *measured* base tilt, which avoids having to pick a branch.  Divide
      by about 0.36-0.51 inch/deg to read it as a tilt error.
    """
    ab = ab_deg * DEG
    return (
        geom.Lr * math.sin(ab + joints.ar * DEG)
        + geom.Ld * math.sin(ab - joints.ad * DEG)
        + (geom.Lb - joints.xd) * math.sin(ab)
        + (geom.yd - geom.yr) * math.cos(ab)
        - geom.C
    )


def _wrap180(angle_deg: float) -> float:
    """Wrap an angle into (-180, +180]."""
    return (angle_deg + 180.0) % 360.0 - 180.0


def estimate(
    ai: Optional[float],
    atf: float,
    atd: float,
    atr: float,
    xtd: float,
    geom: Geometry = DEFAULT_GEOMETRY,
    cal: Calibration = DEFAULT_CALIBRATION,
    ac_deg: float = 0.0,
    branch: str = "upright",
) -> Estimate:
    """Estimate base tilt and curb height from one set of sensor readings.

    Parameters
    ----------
    ai
        IMU tilt angle of the base.  Optional: the kinematic chain already
        determines the base tilt relative to the ground plane, so the IMU is
        used only to (a) estimate the ground slope, since the IMU measures
        absolute tilt while `ab` is relative to the local ground plane, so
        slope = imu_tilt - ab, and (b) cross-check the solution.  Pass None
        if no IMU reading is available.
    atf, atd, atr, xtd
        Raw encoder ticks: front caster, driving wheel, rear caster (upper),
        carriage.
    geom, cal
        Mechanical dimensions and tick conversions.
    ac_deg
        Curb face angle relative to the ground plane; 0 for a vertical face.
    branch
        "upright" (default) returns the physically reachable solution.
        "imu" picks whichever of the two roots is closer to the IMU reading,
        which needs a valid `ai`.  "flipped" returns the rolled-over root,
        which is only useful for debugging.

    Returns
    -------
    Estimate
        `ab_estimate` in degrees, `hc_estimate` in inches, plus the joint
        angles, the rejected branch and the diagnostics.
    """
    joints = ticks_to_joints(atf, atd, atr, xtd, cal)
    A, B, C = closure_coefficients(joints, geom)
    ab1, ab2, reachable = base_tilt(joints, geom)

    imu_tilt = None
    if ai is not None:
        # Wrap about the reading seen when the base is level, then apply the
        # sign convention, so that "nose up" is positive in both systems.
        imu_tilt = cal.imu_sign * _wrap180(ai - cal.imu_level_reading)

    if branch == "upright":
        ab, ab_other = ab1, ab2
    elif branch == "flipped":
        ab, ab_other = ab2, ab1
    elif branch == "imu":
        if imu_tilt is None:
            raise ValueError("branch='imu' needs an IMU reading")
        ab, ab_other = (
            (ab1, ab2)
            if abs(_wrap180(ab1 - imu_tilt)) <= abs(_wrap180(ab2 - imu_tilt))
            else (ab2, ab1)
        )
    else:
        raise ValueError("branch must be 'upright', 'flipped' or 'imu'")

    hc = curb_height(ab, joints, geom, ac_deg)
    hc_other = curb_height(ab_other, joints, geom, ac_deg)

    return Estimate(
        ab_estimate=ab,
        hc_estimate=hc,
        joints=joints,
        A=A,
        B=B,
        C=C,
        ab_branch2=ab_other,
        hc_branch2=hc_other,
        imu_tilt=imu_tilt,
        slope_estimate=None if imu_tilt is None else imu_tilt - ab,
        reachable=reachable,
        closure_residual=closure_residual(ab, joints, geom),
    )


def estimate_batch(
    rows: Sequence[Sequence[Optional[float]]],
    geom: Geometry = DEFAULT_GEOMETRY,
    cal: Calibration = DEFAULT_CALIBRATION,
    ac_deg: float = 0.0,
    branch: str = "upright",
) -> List[Estimate]:
    """Run `estimate` over a sequence of (ai, atf, atd, atr, xtd) rows."""
    return [
        estimate(*row, geom=geom, cal=cal, ac_deg=ac_deg, branch=branch) for row in rows
    ]


# ===========================================================================
# 4.  Re-fitting the calibration from measured poses
# ===========================================================================

#: Which calibration members may be fitted, in a fixed order.
FITTABLE = (
    "af_slope",
    "af_offset",
    "ad_slope",
    "ad_offset",
    "ar_slope",
    "ar_offset",
    "xd_slope",
    "xd_offset",
)

#: Physically sensible search box, used both to bound the optimiser and to
#: draw multi-start points.  Without it the optimiser exploits the
#: periodicity of sin/cos and happily reports an "optimal" arm angle of
#: 1378 deg.  Slopes are held within roughly +-50% of the bench values.
DEFAULT_BOUNDS = {
    "af_slope": (-0.0760, -0.0250),
    "af_offset": (-60.0, 60.0),
    "ad_slope": (0.0370, 0.1250),
    "ad_offset": (-30.0, 40.0),
    "ar_slope": (0.0290, 0.0900),
    "ar_offset": (-30.0, 40.0),
    "xd_slope": (0.0020, 0.0036),
    "xd_offset": (15.0, 75.0),
}


def fit_calibration(
    measurements: Sequence[Sequence[float]],
    geom: Geometry = DEFAULT_GEOMETRY,
    start: Calibration = DEFAULT_CALIBRATION,
    free: Sequence[str] = FITTABLE[:6] + ("xd_offset",),
    bounds: Optional[dict] = None,
    weight_tilt: float = 1.0,
    weight_height: float = 1.0,
    n_starts: int = 400,
    seed: int = 0,
    verbose: bool = False,
):
    """Least-squares fit of the tick conversions to measured poses.

    Parameters
    ----------
    measurements
        Rows of (atf, atd, atr, xtd, ab_measured_deg, hc_measured_inch).
    start
        Calibration the search starts from; any member not listed in `free`
        keeps its value from here, so `start` must already be right for
        those.  Beware of starting from BENCH_CALIBRATION with the carriage
        frozen: its xd_offset is the bench intercept only (1.931 cm) and
        omits the roughly 38 cm home offset, which biases A by 15 inches and
        caps the achievable fit at about 1.1 deg / 1.1 inch RMS.
    free
        Names of the calibration members to fit; the rest stay at `start`.
        The default fits the three arm slopes and offsets plus the carriage
        home offset, and leaves only the carriage *scale* fixed, because a
        data set taken at just two distinct carriage positions cannot
        identify it.
    weight_tilt, weight_height
        Relative weights of the two residual families.  Degrees and inches
        are roughly comparable here because 1 deg of base tilt is worth
        about 0.7 inch of curb height, so 1.0/1.0 is a reasonable default.
    n_starts
        Number of random starting points.  The solver only guarantees a
        local minimum, and this problem has about seven of them inside the
        search box, so a single run from one start is not enough.  Roughly
        96% of random starts reach the best basin; the remaining 4% stop
        somewhere an order of magnitude worse while still reporting success.

    Returns
    -------
    (Calibration, dict)
        The fitted calibration and a report containing per-row residuals,
        RMS values, parameter standard errors and the correlation matrix.

    Notes on the solver
    -------------------
    scipy's 'trf' (trust region reflective) is used rather than plain
    Levenberg-Marquardt because 'lm' cannot honour the bounds above.  Both
    are the same idea: linearise the residual, r(p+d) ~ r + J d, and solve
        (J'J + lambda * diag(J'J)) d = -J'r
    which interpolates between the fast Gauss-Newton step (lambda -> 0) and
    a short, safe scaled-gradient step (lambda -> infinity); trust-region
    methods parameterise the same family by a step-size bound instead of
    lambda.  The damping is not optional here: J'J has a condition number
    around 2e9 because each slope is nearly interchangeable with its own
    offset (correlations up to 0.96), so an undamped Gauss-Newton step
    diverges on about 12% of random starts.  Scaling the damping by
    diag(J'J) rather than the identity also matters, since the parameters
    differ in magnitude by four orders (a slope of 0.0027 against an offset
    of 40).
    """
    import numpy as np  # imported lazily: the forward
    from scipy.optimize import least_squares  # model needs neither

    free = list(free)
    unknown = set(free) - set(FITTABLE)
    if unknown:
        raise KeyError(f"not fittable: {sorted(unknown)}")
    box = dict(DEFAULT_BOUNDS)
    if bounds:
        box.update(bounds)

    data = np.asarray(measurements, dtype=float)
    if data.ndim != 2 or data.shape[1] != 6:
        raise ValueError(
            "measurements must be rows of " "(atf, atd, atr, xtd, ab_meas, hc_meas)"
        )
    atf, atd, atr, xtd, ab_meas, hc_meas = data.T

    lo = np.array([box[k][0] for k in free])
    hi = np.array([box[k][1] for k in free])

    def residuals(q):
        cal = start.set(**dict(zip(free, q)))
        out = np.empty(2 * len(data))
        for i in range(len(data)):
            e = estimate(None, atf[i], atd[i], atr[i], xtd[i], geom=geom, cal=cal)
            out[i] = weight_tilt * (e.ab_estimate - ab_meas[i])
            out[len(data) + i] = weight_height * (e.hc_estimate - hc_meas[i])
        return out

    rng = np.random.default_rng(seed)
    best = None
    for k in range(max(1, n_starts)):
        # first start from `start` itself, then sample the box uniformly
        q0 = (
            np.array([getattr(start, k_) for k_ in free])
            if k == 0
            else lo + rng.random(len(free)) * (hi - lo)
        )
        q0 = np.clip(q0, lo, hi)
        try:
            sol = least_squares(
                residuals, q0, bounds=(lo, hi), method="trf", max_nfev=8000
            )
        except Exception:  # pragma: no cover - solver hiccup
            continue
        if best is None or sol.cost < best.cost:
            best = sol
            if verbose:
                print(f"  start {k:4d}: cost {sol.cost:.6f}")
    if best is None:
        raise RuntimeError("every start failed")

    fitted = start.set(**dict(zip(free, best.x)))

    # Parameter uncertainty from the Gauss-Newton approximation of the
    # Hessian: cov = s^2 * (J'J)^-1 with s^2 the residual variance.  With
    # six poses and six free parameters these intervals are wide, and the
    # correlation matrix is the thing worth reading: entries near 1 mean
    # that slope and offset are trading off against each other and that
    # single-axis sweeps are needed to separate them.
    r = best.fun
    n, m = r.size, best.x.size
    report = {
        "equations": fitted.as_equations(),
        "residual_tilt_deg": (r[: len(data)] / weight_tilt).tolist(),
        "residual_height_in": (r[len(data) :] / weight_height).tolist(),
        "rms_tilt_deg": float(np.sqrt(np.mean((r[: len(data)] / weight_tilt) ** 2))),
        "rms_height_in": float(np.sqrt(np.mean((r[len(data) :] / weight_height) ** 2))),
        "max_abs_tilt_deg": float(np.max(np.abs(r[: len(data)] / weight_tilt))),
        "max_abs_height_in": float(np.max(np.abs(r[len(data) :] / weight_height))),
        "free": free,
        "cost": float(best.cost),
    }
    if n > m:
        try:
            s2 = float(r @ r) / (n - m)
            cov = s2 * np.linalg.inv(best.jac.T @ best.jac)
            se = np.sqrt(np.diag(cov))
            report["std_error"] = dict(zip(free, se.tolist()))
            report["correlation"] = (cov / np.outer(se, se)).tolist()
            report["cond_JtJ"] = float(np.linalg.cond(best.jac.T @ best.jac))
        except np.linalg.LinAlgError:
            report["std_error"] = None  # singular: parameters not identified
    return fitted, report


# ===========================================================================
# 5.  Self-test against the measured poses
# ===========================================================================

#: The six measured poses: (ai, atf, atd, atr, xtd, hc_measured, ab_measured).
#: NOTE on the first row: its recorded base tilt was +1.70 deg, but the
#: closure equation gives -1.67 deg and the IMU reading (-179.87, i.e. 0.13
#: deg off level) also points to a small *negative* tilt, so the sign looks
#: inverted in the log.  Leave-one-out refitting agrees: dropping this row
#: alone cuts the tilt RMS from 0.86 to 0.21 deg, dropping any other row
#: leaves it near 0.8.  The value below is the sign-corrected one, with the
#: original kept alongside.
TEST_POSES = [
    # ai,       atf,  atd,  atr,   xtd,   hc_meas,      ab_meas
    (-179.87, 0, 0, 0, 0, 0.0, -1.70),  # logged as +1.70
    (-178.01, 627, 301, 279, -3, 2.25, 4.10),
    (-174.53, 15, 148, 625, -4, 1.625, -7.10),
    (178.72, 15, 405, 865, -3, 7.25, 0.09),
    (164.03, 914, 405, 95, 6001, 7 - 1 / 16, 16.30),
    (174.40, 399, 200, 95, 6000, 2.5 - 1 / 16, 5.70),
]


def _self_test() -> None:
    """Print the model output for the six measured poses and check it."""
    print("Geometry (inch):")
    for k, v in asdict(DEFAULT_GEOMETRY).items():
        print(f"   {k:3s} = {v:8.4f}")
    print(f"   C   = {DEFAULT_GEOMETRY.C:8.4f}   (Rd - Rr)")
    print(f"   dhc/dab ~ {DEFAULT_GEOMETRY.hc_per_deg_of_tilt:.3f} inch per degree\n")

    print("Calibration:")
    print("   " + DEFAULT_CALIBRATION.as_equations().replace("\n", "\n   "))
    print()

    header = (
        f"{'atf':>5}{'atd':>5}{'atr':>5}{'xtd':>6} | "
        f"{'af':>7}{'ad':>7}{'ar':>7}{'xd':>7} | "
        f"{'ab_est':>8}{'ab_meas':>8}{'d_ab':>7} | "
        f"{'hc_est':>8}{'hc_meas':>8}{'d_hc':>7}"
    )
    print(header)
    print("-" * len(header))

    sq_ab = sq_hc = 0.0
    for ai, atf, atd, atr, xtd, hc_m, ab_m in TEST_POSES:
        e = estimate(ai, atf, atd, atr, xtd)
        d_ab = e.ab_estimate - ab_m
        d_hc = e.hc_estimate - hc_m
        sq_ab += d_ab**2
        sq_hc += d_hc**2
        print(
            f"{atf:5.0f}{atd:5.0f}{atr:5.0f}{xtd:6.0f} | "
            f"{e.joints.af:7.2f}{e.joints.ad:7.2f}{e.joints.ar:7.2f}"
            f"{e.joints.xd:7.2f} | "
            f"{e.ab_estimate:8.2f}{ab_m:8.2f}{d_ab:7.2f} | "
            f"{e.hc_estimate:8.2f}{hc_m:8.2f}{d_hc:7.2f}"
        )
        # the solver must reproduce its own equation
        assert abs(e.closure_residual) < 1e-9, "closure not satisfied"
        # the rejected branch is the rolled-over one
        assert 150.0 < e.ab_branch2 < 180.0, "unexpected second branch"

    n = len(TEST_POSES)
    rms_ab = math.sqrt(sq_ab / n)
    rms_hc = math.sqrt(sq_hc / n)
    print(f"\nRMS: base tilt {rms_ab:.3f} deg, curb height {rms_hc:.3f} inch")
    assert rms_ab < 0.30 and rms_hc < 0.35, "calibration regressed"

    # One pose in full, including the IMU diagnostics.
    print("\nFull output for the tallest curb:")
    print(estimate(*TEST_POSES[4][:5]))
    print("\nSelf-test passed.")


def _fit_demo(n_starts: int = 120) -> None:
    """Re-fit the three arm conversions from the measured poses.

    Reproduces the shipped defaults, and is the template for re-calibrating
    after an encoder is re-indexed or a part is replaced.
    """
    measurements = [
        (atf, atd, atr, xtd, ab_m, hc_m)
        for _, atf, atd, atr, xtd, hc_m, ab_m in TEST_POSES
    ]
    cal, report = fit_calibration(measurements, n_starts=n_starts)
    print("Fitted conversions:")
    print("   " + report["equations"].replace("\n", "\n   "))
    print(
        f"\n   tilt   residuals (deg):  "
        f"{[round(v, 3) for v in report['residual_tilt_deg']]}"
    )
    print(
        f"   height residuals (in) :  "
        f"{[round(v, 3) for v in report['residual_height_in']]}"
    )
    print(
        f"   RMS {report['rms_tilt_deg']:.3f} deg / "
        f"{report['rms_height_in']:.3f} inch   "
        f"(worst {report['max_abs_tilt_deg']:.2f} deg, "
        f"{report['max_abs_height_in']:.2f} inch)"
    )
    if report.get("std_error"):
        print("\n   parameter    value        std error")
        for k in report["free"]:
            print(
                f"   {k:11s} {getattr(cal, k):+12.6f} "
                f"{report['std_error'][k]:12.6f}"
            )
        print(
            f"\n   cond(J'J) = {report['cond_JtJ']:.2e}  "
            f"(large: each slope nearly duplicates its own offset)"
        )
        print("   correlation matrix:")
        for name, row in zip(report["free"], report["correlation"]):
            print(f"   {name:11s} " + " ".join(f"{v:+.2f}" for v in row))


# ===========================================================================
# 6.  Mapping to the RAMMP prototype software
# ===========================================================================
#
# Bookkeeping only: where each model input lives in the robot software, the
# load-cell signals and thresholds the curb sequences use today, and
# suggested starting values for adding a curb-height measurement to those
# sequences.  Names follow the robot code:
#
#   firmware   hardware/rammp_prototype_driver/firmware/Base/
#              Base.ino, Constants.h, Telemetry.cpp, SequencePlayer.h,
#              FcMotors.h, IMU_Class.cpp
#   ROS node   hardware/rammp_prototype_driver/rammp_prototype_driver/
#              MEBot_control_node.py: SerialField, update_data(),
#              curb_traverse_action_callback()
#   keyframes  hardware/rammp_prototype_driver/config/*.json
#
# Checks that tie this model to the robot:
#   * Tick ranges agree.  TEST_POSES span FC 0-914, RC 0-865 and drive arm
#     0-405 ticks; the keyframes use FC 0-900, RC 0-850, drive arm 0-360.
#   * FC tick 0 is the top of the front-caster travel in the firmware
#     (FC_MAX_TICKS = 0), and here rising atf lowers the front wheel
#     (af_slope < 0).
#   * xd.  The closure uses Lb - xd as the rear hinge -> carriage distance,
#     so xd runs backward from the front hinge and rising carriage ticks move
#     the drive wheel rearward.  The URDF agrees (hinges 0.610 m = 24.0 in
#     apart; carriage hinge 0.199 m ahead of the rear hinge, against
#     Lb - xd = 0.209 m at tick 0), as do joint_converter.py (carriage
#     sign = -1) and the keyframe label "Drive forward onto curb while
#     moving carriages back" (carriage 0 -> 12600 ticks).
#   * The carriage calibration only saw about 0 and 6000 ticks.  Keyframes
#     go to 12600-13000, so results there are extrapolated.


class KeyframeIndex(IntEnum):
    """Motor order inside a keyframe.

    Same order in the JSON arrays (targets, relative, motor_durations,
    guard_threshold, guard_condition), in the J<idx> serial command and in
    the firmware's FILL_SEQ_MOTORS.
    """

    RC = 0  # rear caster arm actuator, ticks      -> atr
    FC = 1  # front caster arm actuator, ticks     -> atf
    ML = 2  # left drive-wheel arm, ticks          -> atd (left)
    MR = 3  # right drive-wheel arm, ticks         -> atd (right)
    ML_CARRIAGE = 4  # left carriage, ticks                 -> xtd (left)
    MR_CARRIAGE = 5  # right carriage, ticks                -> xtd (right)
    DRIVE_FB = 6  # drive wheels forward/back, encoder ticks
    DRIVE_LR = 7  # drive wheels left/right, encoder ticks
    FC_HUB_R = 8  # right front-caster hub motor, turns
    FC_HUB_L = 9  # left front-caster hub motor, turns


class TelemetryIndex(IntEnum):
    """Field index in one TELEMETRY line split on commas.

    Field 0 is the literal "TELEMETRY", exactly as MEBot_control_node
    indexes it, so these equal its SerialField values.  The firmware sends
    one line every 100 ms (TELEMETRY_RATE_HZ).
    """

    STATE = 2
    RC_POS = 3
    FC_POS = 4
    ML_POS = 5
    MR_POS = 6
    ML_CARRIAGE_POS = 7
    MR_CARRIAGE_POS = 8
    IMU_PITCH = 37  # IMU.pitchf [deg]
    IMU_ROLL = 38  # IMU.rollf  [deg]; the firmware adds 180 deg to it
    RC_LOADCELL = 52
    FC_LOADCELL = 53
    ML_LOADCELL = 54
    MR_LOADCELL = 55
    FB_PWM = 66  # drive_fb PID output, sent to LUCI during sequences
    CARRIAGE_RETURN_DIRECTION = 82


@dataclass(frozen=True)
class SignalSource:
    """Where one model input lives in the robot software."""

    model_input: str  # argument name of estimate()
    side: str  # "both", "left" or "right"
    description: str
    unit: str
    telemetry_index: int  # TelemetryIndex
    keyframe_index: Optional[int]  # KeyframeIndex, None if not a motor
    ros_attribute: str  # attribute of MEBot_control_node
    state_msg_field: Optional[str]  # RAMMPPrototypeState field, if any


#: Robot signal behind each estimate() argument.  The front and rear caster
#: arms are shared by both sides; the drive arm and the carriage are not.
MODEL_INPUT_SOURCES: Tuple[SignalSource, ...] = (
    SignalSource(
        "ai",
        "both",
        "IMU fore-aft tilt (axis to be confirmed)",
        "deg",
        TelemetryIndex.IMU_PITCH,
        None,
        "imu_pitch",
        None,
    ),
    SignalSource(
        "atf",
        "both",
        "front caster arm actuator FC",
        "tick",
        TelemetryIndex.FC_POS,
        KeyframeIndex.FC,
        "FC_pos",
        "fc_pos",
    ),
    SignalSource(
        "atd",
        "left",
        "left drive-wheel arm ML",
        "tick",
        TelemetryIndex.ML_POS,
        KeyframeIndex.ML,
        "ML_pos",
        "ml_pos",
    ),
    SignalSource(
        "atd",
        "right",
        "right drive-wheel arm MR",
        "tick",
        TelemetryIndex.MR_POS,
        KeyframeIndex.MR,
        "MR_pos",
        "mr_pos",
    ),
    SignalSource(
        "atr",
        "both",
        "rear caster arm actuator RC",
        "tick",
        TelemetryIndex.RC_POS,
        KeyframeIndex.RC,
        "RC_pos",
        "rc_pos",
    ),
    SignalSource(
        "xtd",
        "left",
        "left carriage",
        "tick",
        TelemetryIndex.ML_CARRIAGE_POS,
        KeyframeIndex.ML_CARRIAGE,
        "ML_carriage_pos",
        "ml_carriage_pos",
    ),
    SignalSource(
        "xtd",
        "right",
        "right carriage",
        "tick",
        TelemetryIndex.MR_CARRIAGE_POS,
        KeyframeIndex.MR_CARRIAGE,
        "MR_carriage_pos",
        "mr_carriage_pos",
    ),
)
# Notes on these signals:
#   * ai.  The test data reads about +-180 deg on level ground
#     (Calibration.imu_level_reading = 180, imu_sign = -1).  In the current
#     firmware field 37 (IMU.pitchf) has no 180-deg offset, whereas
#     IMU_Class.cpp adds 180 deg to the roll axis (field 38).  Confirm which
#     field is the fore-aft tilt, and its sign, before passing it as `ai`.
#   * RAMMPPrototypeState has no signed pitch.  Its `tilt` is an unsigned
#     total tilt, computed in MEBot_control_node by passing degrees to
#     math.cos, so it cannot stand in for `ai`.
#   * The position fields of RAMMPPrototypeState are commented "Unit:
#     Meters" in the .msg, but MEBot_control_node fills them with the raw
#     ticks this model expects.

#: One curb height, three units:
#:   this model       Estimate.hc_estimate                     inch
#:   neu_navigation   CurbInfo.height on /nav/curb/info        metre
#:   GUI              UpdateCurbInfo "Height" (Gui_bridge.py)  centimetre
METRES_PER_INCH = 0.0254


def inch_to_m(value_in: float) -> float:
    """Inch -> metre, e.g. to compare hc_estimate with CurbInfo.height."""
    return value_in * METRES_PER_INCH


def m_to_inch(value_m: float) -> float:
    """Metre -> inch, e.g. to bring CurbInfo.height into this model."""
    return value_m / METRES_PER_INCH


# ---------------------------------------------------------------------------
# Load cells
# ---------------------------------------------------------------------------


class GuardCondition(IntEnum):
    """Keyframe guard codes (SequencePlayer.h, guard_condition[i]).

    The firmware checks a guard every control loop (FIRMWARE_LOOP_PERIOD_S)
    during the interpolation phase only, not while settling.  When it trips
    it latches that motor at its current position and prints
    SEQ_GUARD_TRIG,m<index>,load=<value>.
    """

    NONE = 0
    GREATER_THAN = 1  # trips when load >= threshold
    LESS_THAN = 2  # trips when load <= threshold


@dataclass(frozen=True)
class LoadCell:
    """One strain-gauge channel on the base."""

    axis: str
    keyframe_index: int  # KeyframeIndex of the actuator it sits on
    telemetry_index: int  # TelemetryIndex
    analog_pin: str  # Constants.h
    ros_attribute: str  # attribute of MEBot_control_node
    state_msg_field: str  # RAMMPPrototypeState field


#: The four load cells.  The front and rear caster arms have one each,
#: shared by both wheels of the pair; each drive-wheel arm has its own.
LOAD_CELLS: Tuple[LoadCell, ...] = (
    LoadCell(
        "RC",
        KeyframeIndex.RC,
        TelemetryIndex.RC_LOADCELL,
        "A16",
        "RC_loadcell",
        "rc_loadcell",
    ),
    LoadCell(
        "FC",
        KeyframeIndex.FC,
        TelemetryIndex.FC_LOADCELL,
        "A17",
        "FC_loadcell",
        "fc_loadcell",
    ),
    LoadCell(
        "ML",
        KeyframeIndex.ML,
        TelemetryIndex.ML_LOADCELL,
        "A15",
        "ML_loadcell",
        "ml_loadcell",
    ),
    LoadCell(
        "MR",
        KeyframeIndex.MR,
        TelemetryIndex.MR_LOADCELL,
        "A14",
        "MR_loadcell",
        "mr_loadcell",
    ),
)

#: Keyframe indices that can carry a guard.  Every other axis reports a
#: load of 0, so a LESS_THAN guard there would trip at once.
GUARDABLE_KEYFRAME_INDICES: Tuple[int, ...] = tuple(
    int(lc.keyframe_index) for lc in LOAD_CELLS
)

LOADCELL_IIR_ALPHA = 0.8  # StrainGauge(pin, 0.8f), all four channels
LOADCELL_UNITS = "filtered raw ADC counts, uncalibrated"
# Range: Base.ino never calls analogReadResolution(), so the Teensy default
# (10 bit, 0-1023) should apply, but the PID tuner scales its bars to 4095.
# Confirm on the robot.
TELEMETRY_RATE_HZ = 10.0  # Base.ino: one TELEMETRY line per 100 ms
FIRMWARE_LOOP_PERIOD_S = 0.005  # Base.ino: delayMicroseconds(5000)

# Thresholds in use today, both in curb_traverse_action_callback() and both
# on the FC cell.  RC, ML and MR are published but not used anywhere.
FC_LOAD_ASCEND_CONTACT = 200.0  # ascend waits while FC_loadcell > 200
FC_LOAD_DESCEND_OFF_CURB = 150.0  # descend waits while FC_loadcell < 150

#: Inferred from those two loops together with this model, not measured:
#: the reading FALLS as the load rises.  The caster hangs about 4-5 in up
#: during the ascend approach (reads > 200) and is pressed down during the
#: descend approach (reads < 150), so 150-200 acts as a hysteresis band.
#: Log all four cells with the wheels in the air, on the ground and on a
#: curb before relying on it.
LOADCELL_READING_FALLS_WITH_LOAD = True


# ---------------------------------------------------------------------------
# Suggested values for measuring curb height inside the sequences
# ---------------------------------------------------------------------------
# Derived from this model and the current keyframes; NOT tested on the
# robot.  Tune them before use.
#
#   ascend probe   after the FC bump, raise the base and the front caster,
#                  creep forward, then lower FC with a guard until it touches
#                  the curb top.  That is the model's own pose: front caster
#                  on the curb, drive wheels and rear caster on the road.
#   descend guard  guard FC in the first keyframe of curb_descending.json.
#                  At the latch hc_estimate is negative; -hc_estimate is the
#                  drop.

PROBE_GUARD_CONDITION = GuardCondition.LESS_THAN
PROBE_GUARD_THRESHOLD = 175.0  # middle of the 150-200 band; better: the
# free-caster reading minus a margin
#: Base level (+0.2 deg) with the front-caster bottom about 7.3 in up.
#: Close to TEST_POSES[3] (measured 7.25 in), so it is a calibrated pose.
PROBE_RAISED_POSE = {
    KeyframeIndex.RC: 865.0,
    KeyframeIndex.ML: 405.0,
    KeyframeIndex.MR: 405.0,
    KeyframeIndex.FC: 0.0,
}
PROBE_FC_TARGET = 900.0  # ticks, the deepest FC target the keyframes
# already use.  From PROBE_RAISED_POSE the
# caster bottom only comes down to ~1.5 in,
# so that pose probes curbs of ~1.5-7.3 in;
# from the ascend_approach pose it reaches
# below the road.
PROBE_FC_DURATION_MS = 3000  # slow, because guards are not checked
# while the keyframe settles
DESCEND_FC_GUARD_THRESHOLD = 130.0  # below 150 so it does not chatter just
# after the caster leaves the edge
ASCEND_MAX_CURB_HEIGHT_IN = 6.0  # with today's kf0/kf1 the drive-wheel
# clearance is ~0.7 in at 6 in, ~0 at 7 in
DESCEND_MAX_DROP_IN = 5.4  # FC = 900 reaches only 5.5-5.8 in below
# the upper surface


# ---------------------------------------------------------------------------
# Per-side evaluation from robot signals
# ---------------------------------------------------------------------------


@dataclass
class SideEstimates:
    """The 2-D model evaluated once per side of the chair."""

    left: Estimate
    right: Estimate

    @property
    def hc_mean(self) -> float:
        """Mean of the two curb heights [inch]."""
        return 0.5 * (self.left.hc_estimate + self.right.hc_estimate)

    @property
    def hc_spread(self) -> float:
        """|left - right| curb height [inch].

        The caster arms are shared, so a large spread points at the
        drive-arm or carriage calibration of one side.  ML and MR differ by
        20-60 ticks in most keyframes, and hc moves about 0.025-0.035 inch
        per drive-arm tick.
        """
        return abs(self.left.hc_estimate - self.right.hc_estimate)


def estimate_left_right(
    fc_pos: float,
    rc_pos: float,
    ml_pos: float,
    mr_pos: float,
    ml_carriage_pos: float,
    mr_carriage_pos: float,
    imu_pitch: Optional[float] = None,
    geom: Geometry = DEFAULT_GEOMETRY,
    cal_left: Calibration = DEFAULT_CALIBRATION,
    cal_right: Optional[Calibration] = None,
    ac_deg: float = 0.0,
    branch: str = "upright",
) -> SideEstimates:
    """Run `estimate` once per side from MEBot_control_node's raw values.

    Arguments are named after the MEBot_control_node attributes (see
    MODEL_INPUT_SOURCES) and are raw ticks.  `cal_right` defaults to
    `cal_left`; give it its own Calibration once the right drive arm has
    been fitted separately.  `imu_pitch` is passed through as `ai`; leave it
    None until the IMU field and Calibration.imu_level_reading / imu_sign
    have been checked.
    """
    if cal_right is None:
        cal_right = cal_left
    left = estimate(
        imu_pitch,
        fc_pos,
        ml_pos,
        rc_pos,
        ml_carriage_pos,
        geom=geom,
        cal=cal_left,
        ac_deg=ac_deg,
        branch=branch,
    )
    right = estimate(
        imu_pitch,
        fc_pos,
        mr_pos,
        rc_pos,
        mr_carriage_pos,
        geom=geom,
        cal=cal_right,
        ac_deg=ac_deg,
        branch=branch,
    )
    return SideEstimates(left=left, right=right)


def estimate_from_telemetry(
    fields: Union[str, Sequence],
    use_imu: bool = False,
    imu_field: int = TelemetryIndex.IMU_PITCH,
    **kwargs,
) -> SideEstimates:
    """Same as estimate_left_right, from one firmware TELEMETRY line.

    `fields` is the raw line, or the list MEBot_control_node gets from
    line.split(",").  Extra keyword arguments go to estimate_left_right.
    """
    if isinstance(fields, str):
        fields = fields.strip().split(",")
    if not fields or str(fields[0]).strip() != "TELEMETRY":
        raise ValueError("not a TELEMETRY line (field 0 must be 'TELEMETRY')")
    needed = max(TelemetryIndex.MR_CARRIAGE_POS, imu_field if use_imu else 0)
    if len(fields) <= needed:
        raise ValueError(f"TELEMETRY line too short: {len(fields)} fields")

    def value(index: int) -> float:
        return float(fields[index])

    return estimate_left_right(
        fc_pos=value(TelemetryIndex.FC_POS),
        rc_pos=value(TelemetryIndex.RC_POS),
        ml_pos=value(TelemetryIndex.ML_POS),
        mr_pos=value(TelemetryIndex.MR_POS),
        ml_carriage_pos=value(TelemetryIndex.ML_CARRIAGE_POS),
        mr_carriage_pos=value(TelemetryIndex.MR_CARRIAGE_POS),
        imu_pitch=value(imu_field) if use_imu else None,
        **kwargs,
    )


if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "fit":
        _fit_demo()
    else:
        _self_test()
