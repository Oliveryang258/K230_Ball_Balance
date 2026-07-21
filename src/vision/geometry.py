"""Track-projection geometry without camera dependencies.

APIs: MicroPython arithmetic only; no CanMV hardware API calls.
Hardware: none for unit tests; coordinates originate from the K230 camera.
Runtime: compatible with CanMV K230 Yahboom v1.8.0 MicroPython.
"""


def project_ratio(point, fixed_end, servo_end):
    """Project a point onto directed axis A->B and return ratio t.

    fixed_end is Marker A and servo_end is Marker B.  t=0 is A, t=1 is B.
    Values outside [0, 1] are retained so callers can diagnose bad detections.
    """
    px, py = point
    ax, ay = fixed_end
    bx, by = servo_end
    vx = bx - ax
    vy = by - ay
    length_squared = vx * vx + vy * vy
    if length_squared == 0:
        raise ValueError("track endpoints must be different")
    return ((px - ax) * vx + (py - ay) * vy) / length_squared


def normalized_track_error(point, fixed_end, servo_end, clamp=True):
    """Map projection to -1 at A, 0 at track centre, and +1 at B."""
    error = 2.0 * project_ratio(point, fixed_end, servo_end) - 1.0
    if clamp:
        return max(-1.0, min(1.0, error))
    return error
