import numpy as np
from typing import Sequence, Mapping, Tuple
import math


def world_to_body_xy(dx: float, dy: float, yaw: float):
    # rotate world vector into body (x fwd, y left)
    c, s = math.cos(yaw), math.sin(yaw)
    xb =  c * dx + s * dy
    yb = -s * dx + c * dy
    return xb, yb

# Helper to check 3D distance
def dist3(a, b):
    return float(np.linalg.norm(np.array(a) - np.array(b)))

def catmull_rom_coeffs(
    P0: np.ndarray, P1: np.ndarray, P2: np.ndarray, P3: np.ndarray
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Compute Catmull–Rom **cubic polynomial coefficients** for one spline segment.

    The segment between control points P1 → P2 is represented (component-wise) as:
        r(t) = a*t^3 + b*t^2 + c*t + d,   with t ∈ [0, 1]

    Inputs
    ------
    P0, P1, P2, P3 : np.ndarray, shape (3,)
        Four consecutive control points (x, y, z) used to define the CR spline.

    Outputs
    -------
    a, b, c, d : np.ndarray, each shape (3,)
        Vector coefficients so that r(t) = a*t^3 + b*t^2 + c*t + d.

    Notes
    -----
    - Uses the standard Catmull–Rom formulation with tension = 0.5 (uniform CR).
    - a/b/c/d are 3D vectors (for x, y, z simultaneously).

    Example
    -------
    >>> a,b,c,d = catmull_rom_coeffs(np.zeros(3), np.array([1,0,0]), np.array([2,0,0]), np.array([3,0,0]))
    """
    m1 = 0.5 * (P2 - P0)  # incoming tangent at P1
    m2 = 0.5 * (P3 - P1)  # outgoing tangent at P2
    a =  2*P1 - 2*P2 + m1 + m2
    b = -3*P1 + 3*P2 - 2*m1 - m2
    c = m1
    d = P1
    return a, b, c, d


def eval_cubic(
    a: np.ndarray, b: np.ndarray, c: np.ndarray, d: np.ndarray, t: float
) -> np.ndarray:
    """
    Evaluate the cubic polynomial r(t) = a*t^3 + b*t^2 + c*t + d at parameter t.

    Inputs
    ------
    a, b, c, d : np.ndarray, each shape (3,)
        Cubic coefficients (from `catmull_rom_coeffs`).
    t : float
        Segment parameter in [0, 1].

    Output
    ------
    r : np.ndarray, shape (3,)
        3D point (x, y, z) on the segment at parameter t.

    Implementation detail
    ---------------------
    Uses Horner's method for numerical stability and speed:
        ((a*t + b)*t + c)*t + d
    """
    return ((a * t + b) * t + c) * t + d


def eval_cubic_der(
    a: np.ndarray, b: np.ndarray, c: np.ndarray, t: float
) -> np.ndarray:
    """
    Evaluate the **tangent/derivative** of the cubic: r'(t) = 3a*t^2 + 2b*t + c.

    Inputs
    ------
    a, b, c : np.ndarray, each shape (3,)
        Cubic coefficients (d is not needed for the derivative).
    t : float
        Segment parameter in [0, 1].

    Output
    ------
    r_prime : np.ndarray, shape (3,)
        Tangent vector (dx/dt, dy/dt, dz/dt) at parameter t.

    Notes
    -----
    - The magnitude ||r'(t)|| is *arc-length per unit t*. It is used to convert
      a desired metric step (meters) into a parameter step Δt.
    """
    return (3 * a * t + 2 * b) * t + c


def pos_at(
    coeffs: Sequence[Mapping[str, float]], i: int, tt: float
) -> np.ndarray:
    """
    Compute the **3D position** on spline segment `i` at parameter `tt`.

    Inputs
    ------
    coeffs : sequence of dict-like
        Each element has keys:
            "ax","bx","cx","dx", "ay","by","cy","dy", "az","bz","cz","dz"
        representing the cubic coefficients for x, y, z.
    i : int
        Segment index (0 ≤ i < len(coeffs)).
    tt : float
        Segment-local parameter in [0, 1].

    Output
    ------
    p : np.ndarray, shape (3,)
        Position (x, y, z) on the segment.

    Example
    -------
    >>> p = pos_at(coeffs, i=0, tt=0.5)
    """
    co = coeffs[i]
    a = np.array([co["ax"], co["ay"], co["az"]], float)
    b = np.array([co["bx"], co["by"], co["bz"]], float)
    c = np.array([co["cx"], co["cy"], co["cz"]], float)
    d = np.array([co["dx"], co["dy"], co["dz"]], float)
    return ((a * tt + b) * tt + c) * tt + d


def tan_at(
    coeffs: Sequence[Mapping[str, float]], i: int, tt: float
) -> np.ndarray:
    """
    Compute the **tangent vector** (derivative) on segment `i` at parameter `tt`.

    Inputs
    ------
    coeffs : sequence of dict-like
        Same format as in `pos_at`.
    i : int
        Segment index (0 ≤ i < len(coeffs)).
    tt : float
        Segment-local parameter in [0, 1].

    Output
    ------
    tvec : np.ndarray, shape (3,)
        Tangent vector r'(tt) at the given point. Use atan2(tvec_y, tvec_x)
        to get a heading/yaw along the spline in the XY plane.

    Example
    -------
    >>> tvec = tan_at(coeffs, i=0, tt=0.5)
    """
    co = coeffs[i]
    a = np.array([co["ax"], co["ay"], co["az"]], float)
    b = np.array([co["bx"], co["by"], co["bz"]], float)
    c = np.array([co["cx"], co["cy"], co["cz"]], float)
    return (3 * a * tt + 2 * b) * tt + c

def advance(
    coeffs,
    i,
    tt,
    dist_m,
    direction,
    closed,
    target_step=0.1,    # desired arc-length per micro-step (meters)
    min_param_step=1e-4,# floor on |Δt| so we always change t a little
    max_param_step=0.1, # cap on |Δt| to avoid jumping too far
    min_arc_step=1e-4   # floor on arc advance (meters) to avoid zero-length moves
):
    """
    Advance ~`dist_m` meters along the Catmull–Rom spline from the current
    parametric location `(i, tt)`.

    Parameters
    ----------
    coeffs : List[dict]
        Per-segment cubic coefficients as dicts with keys
        {"ax","bx","cx","dx","ay","by","cy","dy","az","bz","cz","dz"}.
        One dict per segment (len(coeffs) == num_segments).
    i : int
        Current segment index (0 .. len(coeffs)-1). If out of range, it will
        be clamped/wrapped when `closed=True`.
    tt : float
        Current segment parameter in [0,1]. Values slightly outside will be
        clamped to the segment ends during stepping.
    dist_m : float
        Arc-length distance to advance (in meters), positive magnitude.
        The function will try to move forward by approximately this length.
    direction : {+1, -1}
        +1 = move forward (increasing parameter/segments),
        -1 = move backward (decreasing).
    closed : bool
        If True, wrap around when crossing the first/last segment.
        If False, clamp at the endpoints and stop advancing there.

    Optional stepping knobs
    -----------------------
    target_step : float
        Target arc-length (meters) per micro-step. Smaller = smoother/finer,
        larger = fewer iterations.
    min_param_step : float
        Minimum absolute parameter step |Δt| to ensure `tt` actually changes,
        useful near knots where speed≈0.
    max_param_step : float
        Maximum absolute parameter step |Δt| to prevent overshooting within a segment.
    min_arc_step : float
        Minimum arc-length per micro-step. If a computed step is smaller, we
        force progress (or cross to the next segment) to avoid stalling.

    Returns
    -------
    i_new : int
        New segment index after advancing.
    tt_new : float
        New segment parameter in [0,1] after advancing.
    pos_new : np.ndarray, shape (3,)
        3D position (x,y,z) at the new (segment, parameter) location.

    Notes
    -----
    * The function integrates along the curve using small micro-steps that try
      to achieve `target_step` meters each. It adapts Δt using the local tangent magnitude.
    * Boundaries are handled: we cross to the next/prev segment, wrap if `closed`,
      or clamp and return if open-ended.
    * Floors on param and arc steps ensure we always make progress, which fixes the
      “five identical points right at start” issue that can happen at knots (t≈0 or t≈1).
    """

    # Remaining arc-length we still need to travel
    remaining = float(abs(dist_m))

    # Starting world position at the current param location
    prev = pos_at(coeffs, i, tt)

    # Convert direction flag to sign (+1 forward, -1 backward)
    sgn = 1.0 if direction > 0 else -1.0

    # Safety breaker for pathological cases (degenerate geometry, etc.)
    safety = 0

    while remaining > 1e-6 and safety < 20000:
        safety += 1

        # Local tangent at (i, tt) — its norm is proportional to arc-length speed (|dr/dt|)
        tvec = tan_at(coeffs, i, tt)
        speed = float(np.linalg.norm(tvec))  # meters per 1.0 unit of t

        # Compute a parameter step Δt that *aims* to move ~target_step meters.
        if speed < 1e-12:
            # Near-zero tangent: directly push parameter by the minimum allowed step
            dtt = sgn * min_param_step
        else:
            dtt = (target_step / (speed + 1e-12)) * sgn
            # Enforce parameter step floors/caps so we *do* move but not too far
            if abs(dtt) < min_param_step:
                dtt = sgn * min_param_step
            dtt = float(np.clip(dtt, -max_param_step, max_param_step))

        # Tentative next parameter within the current segment
        next_t = tt + dtt
        crossed = False  # will flip True if we hit a segment boundary

        # Clamp to segment boundaries; flag if we hit one (we'll cross segments below)
        if direction > 0 and next_t > 1.0:
            next_t = 1.0
            crossed = True
        elif direction < 0 and next_t < 0.0:
            next_t = 0.0
            crossed = True

        # Evaluate world position at the tentative parameter
        cur = pos_at(coeffs, i, next_t)

        # Arc-length of this micro-step
        step_len = float(np.linalg.norm(cur - prev))

        # If the arc move is too tiny, force progress or cross segments to avoid stalling
        if step_len < min_arc_step:
            if (direction > 0 and next_t >= 1.0) or (direction < 0 and next_t <= 0.0):
                # We're pinned at the edge; we must cross segments
                crossed = True
            else:
                # Accept the tiny param change but try again without consuming `remaining`
                tt = next_t
                prev = cur
                continue  # go compute another micro-step

        # If this micro-step overshoots the remaining distance,
        # interpolate proportionally to land exactly on the target arc-length.
        if step_len >= remaining:
            frac = remaining / max(step_len, 1e-12)
            pos_new = prev + (cur - prev) * frac
            tt_new = tt + (next_t - tt) * frac
            return i, tt_new, pos_new

        # Consume this micro-step and continue
        remaining -= step_len
        prev = cur
        tt = next_t

        # If we hit a boundary, move to the next/prev segment
        if crossed:
            if direction > 0:
                i += 1
                if i >= len(coeffs):
                    if closed:
                        i = 0       # wrap around for closed paths
                    else:
                        # Open path end — clamp at the very end
                        return len(coeffs) - 1, 1.0, pos_at(coeffs, len(coeffs) - 1, 1.0)
                tt = 0.0
            else:
                i -= 1
                if i < 0:
                    if closed:
                        i = len(coeffs) - 1
                    else:
                        # Open path start — clamp at the very beginning
                        return 0, 0.0, pos_at(coeffs, 0, 0.0)
                tt = 1.0

    # If we exit the loop (very small remaining or safety break), return current state
    return i, tt, prev

def arc_available(coeffs, i, tt, direction, max_len, closed, step_target=0.1):
    """Approximate how much arc-length is available from (i,tt) in given direction,
    up to max_len. Stops early if we reach an open boundary."""
    remain = float(max_len)
    cur_i, cur_t = int(i), float(tt)
    cur_pos = pos_at(coeffs, cur_i, cur_t)
    hops = 0
    while remain > 1e-6 and hops < 2000:
        hops += 1
        tvec = tan_at(coeffs, cur_i, cur_t)
        speed = float(np.linalg.norm(tvec))
        if speed < 1e-8:
            # nudge inside the segment to escape flat derivative
            eps = 1e-3 if direction > 0 else -1e-3
            nxt_t = float(np.clip(cur_t + eps, 0.0, 1.0))
            if nxt_t == cur_t:
                # boundary and cannot move in this direction on an open spline
                if not closed and ((direction < 0 and cur_t <= 1e-9 and cur_i == 0) or
                                   (direction > 0 and cur_t >= 1.0-1e-9 and cur_i == len(coeffs)-1)):
                    break
                # wrap/clamp
                if closed:
                    cur_i = (cur_i + (1 if direction > 0 else -1)) % len(coeffs)
                    cur_t = 0.0 if direction > 0 else 1.0
                continue
            cur_t = nxt_t
            nxt_pos = pos_at(coeffs, cur_i, cur_t)
        else:
            dtt = float(np.clip((step_target / speed) * (1.0 if direction > 0 else -1.0), -0.1, 0.1))
            nxt_t = cur_t + dtt
            crossed = False
            if direction > 0 and nxt_t > 1.0:
                nxt_t = 1.0; crossed = True
            elif direction < 0 and nxt_t < 0.0:
                nxt_t = 0.0; crossed = True
            nxt_pos = pos_at(coeffs, cur_i, nxt_t)

        step = float(np.linalg.norm(nxt_pos - cur_pos))
        if step < 1e-9:
            # Nothing changed; treat as boundary
            if not closed and ((direction < 0 and cur_t <= 1e-9 and cur_i == 0) or
                               (direction > 0 and cur_t >= 1.0-1e-9 and cur_i == len(coeffs)-1)):
                break
            if closed:
                cur_i = (cur_i + (1 if direction > 0 else -1)) % len(coeffs)
                cur_t = 0.0 if direction > 0 else 1.0
            continue

        remain -= step
        cur_pos = nxt_pos
        cur_t = nxt_t

        if step >= remain:
            break

        # segment boundary?
        if (direction > 0 and cur_t >= 1.0-1e-12) or (direction < 0 and cur_t <= 1e-12):
            if closed:
                cur_i = (cur_i + (1 if direction > 0 else -1)) % len(coeffs)
                cur_t = 0.0 if direction > 0 else 1.0
            else:
                # open spline ran out of arc in this direction
                break

    used = max_len - remain
    return used


