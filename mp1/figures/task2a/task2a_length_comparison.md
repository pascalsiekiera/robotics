# Task 2a: measured vs recorded path lengths

The segment lengths were measured with a tape measure in the lab (distance between the
stop points). The recorded lengths come from the wheel odometry (`/odom`) of the bags:
each segment is the straight-line distance between the pose where the robot started
driving and the pose where it stopped (between two turns on the spot).

Error = odometry − measured. Negative: odometry reports a shorter distance than driven.

## Run 11 (`2a_raw_11`)

| segment | measured [cm] | odometry [cm] | error [cm] | error [%] |
|---|---|---|---|---|
| straight | 192 | 187.7 | −4.3 | −2.2 |
| left (turn 87°) | 70 | 66.1 | −3.9 | −5.6 |
| **total** | **262** | **253.8** | **−8.2** | **−3.1** |

## Run 14 (`2a_raw_14`, final run)

Route: l = left turn, r = right turn, rr = turn of about 160° to the right, each followed
by a straight segment.

| segment | measured [cm] | odometry [cm] | error [cm] | error [%] |
|---|---|---|---|---|
| l | 91 | 91.6 | +0.6 | +0.7 |
| r | 120 | 112.9 | −7.1 | −5.9 |
| l | 130 | 125.1 | −4.9 | −3.8 |
| rr | 70 | 56.7 | −13.3 | −19.0 |
| r | 192 | 205.8 * | +13.8 | +7.2 |
| **total** | **603** | **592.1** | **−10.9** | **−1.8** |

\* In the last segment of run 14 the robot did not drive straight: its heading changed by
about 130° while driving, the odometry path along the curve is 229.9 cm. The value is the
straight-line distance between start and end of that segment, so it is less reliable.

## Observations

- The odometry mostly reports **shorter** distances than measured: −3.1 % (run 11) and
  −1.8 % (run 14) over the whole route, single straight segments typically −2 % to −6 %.
- The long straight segment of run 11 agrees best (192 cm: −2.2 %).
- The **"rr" segment (70 cm) of run 14 is clearly too short (−19 %)**. It follows the
  largest turn (~160°), so either the robot started this segment from a slightly
  different point than the tape measurement, or wheel slip during the turn shifted it.
- The error is not random from segment to segment: it accumulates, which is why the
  odometry path drifts over time and why SLAM / AMCL correct it with the laser.
