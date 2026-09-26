from __future__ import annotations

from dataclasses import dataclass
import math
from typing import List, Optional, Sequence


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


@dataclass
class FilterDiagnostics:
    slip_score: float = 0.0
    front_weight: float = 0.0
    rear_weight: float = 0.0
    model_accel: float = 0.0
    adaptive_accel_bias: float = 0.0
    common_slip_prob: float = 0.0
    front_slip_ratio: float = 0.0
    rear_slip_ratio: float = 0.0
    standstill: float = 0.0
    equivalent_motor_torque_nm: float = float("nan")


class NonlinearDriveModel:
    """Hybrid identified + physics longitudinal model.

    The identified part is intentionally linear in parameters and nonlinear in
    inputs, which keeps offline calibration robust and cheap:

      a_id = theta^T [1, u+, u+^2, u-, u-^2, v, v|v|, u+v, u-v] + b_a

    The v and v|v| terms are the specific-force analogue of the linear and
    quadratic Davis resistance terms. The notch/speed interaction captures the
    speed dependence of traction/braking effort. A first-order actuator lag is
    applied by RobustTramFilter.

    If a digital path is available, known grade and curvature resistance are
    added explicitly. grade = dz/ds (positive uphill), curvature = 1/R.
    """

    G = 9.80665

    def __init__(
        self,
        coeffs: Sequence[float],
        accel_bias_limit: float = 0.35,
        grade_gain: float = 1.0,
        curve_accel_per_curvature: float = 0.0,
        vehicle_mass_kg: float = 0.0,
        wheel_radius_m: float = 0.0,
        gear_ratio: float = 0.0,
        drive_efficiency: float = 1.0,
        driven_equivalent_count: float = 1.0,
    ):
        if len(coeffs) != 9:
            raise ValueError("dynamics_coeffs must have exactly 9 values")
        self.coeffs = [float(x) for x in coeffs]
        self.bias = 0.0
        self.bias_limit = float(accel_bias_limit)
        self.grade_gain = float(grade_gain)
        self.curve_accel_per_curvature = max(0.0, float(curve_accel_per_curvature))
        self.vehicle_mass_kg = max(0.0, float(vehicle_mass_kg))
        self.wheel_radius_m = max(0.0, float(wheel_radius_m))
        self.gear_ratio = max(0.0, float(gear_ratio))
        self.drive_efficiency = clamp(float(drive_efficiency), 1e-3, 1.0)
        self.driven_equivalent_count = max(1e-6, float(driven_equivalent_count))

    def _features(self, notch: int, v: float) -> List[float]:
        u = clamp(float(notch) / 15.0, -1.0, 1.0)
        up = max(u, 0.0)
        un = max(-u, 0.0)
        return [1.0, up, up * up, un, un * un, v, v * abs(v), up * v, un * v]

    def identified_accel(self, notch: int, v: float) -> float:
        phi = self._features(notch, v)
        return sum(c * p for c, p in zip(self.coeffs, phi))

    def drive_specific_accel(self, notch: int, v: float) -> float:
        """Identified traction/brake contribution F_drive / m [m/s^2]."""
        u = clamp(float(notch) / 15.0, -1.0, 1.0)
        up = max(u, 0.0); un = max(-u, 0.0)
        c = self.coeffs
        return c[1]*up + c[2]*up*up + c[3]*un + c[4]*un*un + c[7]*up*v + c[8]*un*v

    def equivalent_motor_torque(self, notch: int, v: float) -> Optional[float]:
        """Equivalent shaft torque if physical drivetrain parameters are supplied.

        T_m = (m * a_drive) * R_w / (n_d * eta_g * i_g).
        Returns None when the organizers have not supplied enough physical parameters.
        """
        if self.vehicle_mass_kg <= 0.0 or self.wheel_radius_m <= 0.0 or self.gear_ratio <= 0.0:
            return None
        force = self.vehicle_mass_kg * self.drive_specific_accel(notch, v)
        return force * self.wheel_radius_m / (self.driven_equivalent_count * self.drive_efficiency * self.gear_ratio)

    def accel(self, notch: int, v: float, grade: float = 0.0, curvature: float = 0.0) -> float:
        # sin(arctan(grade)) is exact for grade=dz/ds on the horizontal arc-length parameterization.
        a_grade = -self.grade_gain * self.G * grade / math.sqrt(1.0 + grade * grade)
        a_curve = -self.curve_accel_per_curvature * abs(curvature)
        return self.identified_accel(notch, v) + self.bias + a_grade + a_curve

    def daccel_dv(self, notch: int, v: float) -> float:
        u = clamp(float(notch) / 15.0, -1.0, 1.0)
        up = max(u, 0.0)
        un = max(-u, 0.0)
        # derivative of v*|v| = 2|v| away from zero
        return self.coeffs[5] + 2.0 * self.coeffs[6] * abs(v) + self.coeffs[7] * up + self.coeffs[8] * un

    def adapt_bias(self, observed_accel: float, notch: int, v: float, gain: float) -> None:
        # Adapt only the unknown slowly varying disturbance: load, average grade error,
        # rolling resistance change, etc. Caller gates this update by adhesion confidence.
        residual = observed_accel - self.identified_accel(notch, v)
        self.bias = clamp((1.0 - gain) * self.bias + gain * residual, -self.bias_limit, self.bias_limit)


class AdhesionModeObserver:
    """Small interacting-multiple-model-style adhesion observer.

    Modes: normal, front-slip, rear-slip, common-mode-slip.  We do not pretend
    that the adhesion coefficient is directly observable from only two scalar
    wheel speeds. Instead the mode probabilities are driven by model residuals,
    bogie disagreement and acceleration contradiction, then used as measurement
    reliabilities. This is deliberately lightweight for real-time ROS execution.
    """

    def __init__(self, persistence: float = 0.94):
        self.p = [0.94, 0.02, 0.02, 0.02]
        self.persistence = clamp(float(persistence), 0.5, 0.995)

    @staticmethod
    def _g(x: float, sigma: float) -> float:
        q = x / max(sigma, 1e-6)
        # Clamp exponent for numerical safety.
        return math.exp(-0.5 * min(q * q, 60.0))

    def update(
        self,
        v_model: float,
        front: Optional[float],
        rear: Optional[float],
        accel_contradiction: float,
    ) -> tuple[float, float, float, float, float]:
        # Markov prediction: strong persistence with a small probability leak.
        leak = (1.0 - self.persistence) / 3.0
        pred = [self.persistence * self.p[i] + leak * (1.0 - self.p[i]) for i in range(4)]

        if front is None and rear is None:
            self.p = pred
            return 0.0, 0.0, self.p[3], 1.0 - self.p[0], 0.0

        rf = 0.0 if front is None else front - v_model
        rr = 0.0 if rear is None else rear - v_model
        disagreement = 0.0 if front is None or rear is None else abs(front - rear)
        dis_ev = clamp((disagreement - 0.06) / 0.45, 0.0, 1.0)

        # Normal mode: both wheels close to predicted body speed and to each other.
        ln = 1.0
        if front is not None:
            ln *= self._g(rf, 0.32)
        if rear is not None:
            ln *= self._g(rr, 0.32)
        if front is not None and rear is not None:
            ln *= self._g(disagreement, 0.18)

        # Individual-slip modes: trust the opposite bogie and require disagreement evidence.
        lf = (0.03 + 0.97 * dis_ev) * (self._g(rr, 0.34) if rear is not None else 0.25)
        lr = (0.03 + 0.97 * dis_ev) * (self._g(rf, 0.34) if front is not None else 0.25)

        # Common-mode slip is the dangerous case: bogies agree with each other but
        # jointly disagree with the model/dynamics. Acceleration contradiction is a
        # second independent cue that prevents a mere model offset from firing too hard.
        rs = []
        if front is not None:
            rs.append(rf)
        if rear is not None:
            rs.append(rr)
        common_res = sum(rs) / len(rs)
        common_res_ev = clamp((abs(common_res) - 0.28) / 1.10, 0.0, 1.0)
        agreement_ev = 1.0 - clamp(disagreement / 0.35, 0.0, 1.0)
        common_ev = max(common_res_ev * agreement_ev, accel_contradiction * agreement_ev)
        lc = 0.02 + 0.98 * common_ev

        like = [max(1e-8, ln), max(1e-8, lf), max(1e-8, lr), max(1e-8, lc)]
        post = [pred[i] * like[i] for i in range(4)]
        z = sum(post)
        self.p = [x / z for x in post] if z > 1e-15 else pred

        # Wheel reliabilities from the mode mixture.
        p0, pf, pr, pc = self.p
        front_rel = p0 + 0.06 * pf + 0.97 * pr + 0.06 * pc
        rear_rel = p0 + 0.97 * pf + 0.06 * pr + 0.06 * pc
        slip_score = 1.0 - p0
        return front_rel, rear_rel, pc, slip_score, disagreement


class RobustTramFilter:
    """Robust 3-state nonlinear estimator x=[distance, velocity, acceleration].

    Core design:
      * nonlinear identified traction/brake model + first-order actuator lag;
      * explicit optional grade/curve resistance from the path map;
      * EKF covariance propagation;
      * IMM-style adhesion mode probabilities;
      * innovation/NIS + Huber robust wheel updates;
      * slow disturbance adaptation only on high-confidence adhesion intervals;
      * standstill constraint to prevent drift at stops.
    """

    def __init__(
        self,
        coeffs: Sequence[float],
        tau_accel: float = 0.35,
        wheel_sigma: float = 0.10,
        process_accel_sigma: float = 0.55,
        max_speed: float = 25.0,
        bias_adapt_gain: float = 0.01,
        grade_gain: float = 1.0,
        curve_accel_per_curvature: float = 0.0,
        vehicle_mass_kg: float = 0.0,
        wheel_radius_m: float = 0.0,
        gear_ratio: float = 0.0,
        drive_efficiency: float = 1.0,
        driven_equivalent_count: float = 1.0,
        nis_gate: float = 9.0,
        stop_speed: float = 0.06,
        stop_confirm_s: float = 0.35,
    ):
        self.model = NonlinearDriveModel(
            coeffs,
            grade_gain=grade_gain,
            curve_accel_per_curvature=curve_accel_per_curvature,
            vehicle_mass_kg=vehicle_mass_kg,
            wheel_radius_m=wheel_radius_m,
            gear_ratio=gear_ratio,
            drive_efficiency=drive_efficiency,
            driven_equivalent_count=driven_equivalent_count,
        )
        self.tau_accel = max(0.05, float(tau_accel))
        self.wheel_sigma = max(0.01, float(wheel_sigma))
        self.process_accel_sigma = max(0.05, float(process_accel_sigma))
        self.max_speed = float(max_speed)
        self.bias_adapt_gain = float(bias_adapt_gain)
        self.nis_gate = max(1.0, float(nis_gate))
        self.stop_speed = max(0.01, float(stop_speed))
        self.stop_confirm_s = max(0.05, float(stop_confirm_s))

        self.x = [0.0, 0.0, 0.0]
        self.P = [
            [1e-4, 0.0, 0.0],
            [0.0, 0.25, 0.0],
            [0.0, 0.0, 0.50],
        ]
        self.last_t: Optional[float] = None
        self.last_wheel_t: Optional[float] = None
        self.last_wheel_v: Optional[float] = None
        self.filtered_wheel_accel = 0.0
        self.adhesion = AdhesionModeObserver()
        self.stop_since: Optional[float] = None
        self.diag = FilterDiagnostics()

    @property
    def distance(self) -> float:
        return self.x[0]

    @property
    def velocity(self) -> float:
        return self.x[1]

    @property
    def acceleration(self) -> float:
        return self.x[2]

    def initialize_velocity(self, v: float) -> None:
        self.x[1] = clamp(float(v), 0.0, self.max_speed)

    @staticmethod
    def _matmul(A, B):
        return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

    @staticmethod
    def _transpose(A):
        return [[A[j][i] for j in range(3)] for i in range(3)]

    def predict_to(self, t: float, notch: int, grade: float = 0.0, curvature: float = 0.0) -> None:
        if self.last_t is None:
            self.last_t = float(t)
            return
        dt = float(t) - self.last_t
        if dt < -0.10:
            # New bag / clock reset: keep state but restart the integration clock.
            self.last_t = float(t)
            return
        if dt <= 0.0:
            return
        # Avoid a single timestamp gap exploding state/covariance. Repeated calls
        # after a dropout still propagate in bounded chunks as new data arrive.
        dt = min(dt, 0.5)
        s, v, a = self.x
        aeq = self.model.accel(notch, v, grade=grade, curvature=curvature)
        rho = math.exp(-dt / self.tau_accel)
        s_new = s + v * dt + 0.5 * a * dt * dt
        v_new = clamp(v + a * dt, 0.0, self.max_speed)
        a_new = rho * a + (1.0 - rho) * aeq
        if v_new <= 1e-4 and a_new < 0.0:
            a_new = 0.0
        self.x = [max(0.0, s_new), v_new, a_new]

        da_dv = self.model.daccel_dv(notch, v)
        F = [
            [1.0, dt, 0.5 * dt * dt],
            [0.0, 1.0, dt],
            [0.0, (1.0 - rho) * da_dv, rho],
        ]
        FP = self._matmul(F, self.P)
        Pn = self._matmul(FP, self._transpose(F))
        q = self.process_accel_sigma ** 2
        Q = [
            [0.25 * q * dt**4 + 1e-8, 0.5 * q * dt**3, 0.0],
            [0.5 * q * dt**3, q * dt**2 + 1e-6, 0.0],
            [0.0, 0.0, q * dt + 1e-5],
        ]
        self.P = [[Pn[i][j] + Q[i][j] for j in range(3)] for i in range(3)]
        self.last_t = float(t)
        self.diag.model_accel = aeq
        self.diag.adaptive_accel_bias = self.model.bias
        tq = self.model.equivalent_motor_torque(notch, v)
        self.diag.equivalent_motor_torque_nm = float("nan") if tq is None else tq

    def _wheel_accel_contradiction(self, t: float, fused: float, notch: int) -> float:
        score = 0.0
        if self.last_wheel_t is not None and self.last_wheel_v is not None:
            dt = t - self.last_wheel_t
            if 0.05 <= dt <= 0.35:
                raw_a = (fused - self.last_wheel_v) / dt
                alpha = 0.30
                self.filtered_wheel_accel = (1.0 - alpha) * self.filtered_wheel_accel + alpha * raw_a
                model_a = self.diag.model_accel
                physical_score = clamp((abs(self.filtered_wheel_accel) - 1.8) / 2.2, 0.0, 1.0)
                model_err = abs(self.filtered_wheel_accel - model_a)
                model_score = clamp((model_err - 1.0) / 2.2, 0.0, 1.0)
                score = max(physical_score, model_score)
                self.last_wheel_t = t
                self.last_wheel_v = fused
            elif dt > 0.35:
                self.last_wheel_t = t
                self.last_wheel_v = fused
        else:
            self.last_wheel_t = t
            self.last_wheel_v = fused
        return score

    def _measurement_update(self, z: float, reliability: float) -> float:
        z = clamp(float(z), 0.0, self.max_speed)
        reliability = clamp(reliability, 0.02, 1.0)
        innovation = z - self.x[1]
        # Reliability is interpreted as a standard-deviation multiplier.
        R = (self.wheel_sigma / reliability) ** 2
        S0 = max(1e-12, self.P[1][1] + R)
        nis = innovation * innovation / S0
        if nis > self.nis_gate:
            R *= min(1000.0, (nis / self.nis_gate) ** 2)
        # Huber-type additional protection against gross, isolated outliers.
        S = max(1e-12, self.P[1][1] + R)
        huber = 2.5 * math.sqrt(S)
        if abs(innovation) > huber:
            R *= min(100.0, abs(innovation) / max(huber, 1e-9))
            S = max(1e-12, self.P[1][1] + R)

        K = [self.P[i][1] / S for i in range(3)]
        oldP = [row[:] for row in self.P]
        self.x = [self.x[i] + K[i] * innovation for i in range(3)]
        self.x[1] = clamp(self.x[1], 0.0, self.max_speed)

        # Joseph-stabilized scalar measurement update, H=[0,1,0].
        I_KH = [[1.0, -K[0], 0.0], [0.0, 1.0 - K[1], 0.0], [0.0, -K[2], 1.0]]
        tmp = self._matmul(I_KH, oldP)
        Pn = self._matmul(tmp, self._transpose(I_KH))
        for i in range(3):
            for j in range(3):
                Pn[i][j] += K[i] * R * K[j]
        for i in range(3):
            Pn[i][i] = max(Pn[i][i], 1e-10)
            for j in range(i + 1, 3):
                q = 0.5 * (Pn[i][j] + Pn[j][i])
                Pn[i][j] = Pn[j][i] = q
        self.P = Pn
        return reliability / (1.0 + max(0.0, nis - 1.0) / self.nis_gate)

    def _apply_standstill(self, t: float, notch: int, front: Optional[float], rear: Optional[float]) -> None:
        vals = [x for x in (front, rear) if x is not None and math.isfinite(x)]
        near_zero = bool(vals) and max(vals) < self.stop_speed
        # Do not interpret locked wheels at significant predicted speed as a stop.
        plausible_stop = self.velocity < 0.35 and notch <= 0
        if near_zero and plausible_stop:
            if self.stop_since is None:
                self.stop_since = t
            if t - self.stop_since >= self.stop_confirm_s:
                self.x[1] = 0.0
                self.x[2] = 0.0
                self.P[1][1] = min(self.P[1][1], 0.0025)
                self.P[2][2] = min(self.P[2][2], 0.02)
                self.diag.standstill = 1.0
        else:
            self.stop_since = None
            self.diag.standstill = 0.0

    def update_wheels(
        self,
        t: float,
        notch: int,
        front: Optional[float],
        rear: Optional[float],
        update_front: bool = True,
        update_rear: bool = True,
    ) -> None:
        def valid(v: Optional[float]) -> bool:
            return v is not None and math.isfinite(v) and 0.0 <= v <= self.max_speed

        front = front if valid(front) else None
        rear = rear if valid(rear) else None
        vals = [v for v in (front, rear) if v is not None]
        if not vals:
            self.diag.front_weight = 0.0
            self.diag.rear_weight = 0.0
            return
        if self.velocity <= 0.01 and self.last_wheel_v is None:
            self.initialize_velocity(sum(vals) / len(vals))

        fused = sum(vals) / len(vals)
        accel_score = self._wheel_accel_contradiction(t, fused, notch)
        f_rel, r_rel, p_common, slip_score, _ = self.adhesion.update(
            self.velocity, front, rear, accel_score
        )
        self.diag.common_slip_prob = p_common
        self.diag.slip_score = slip_score

        vden = max(self.velocity, 0.5)
        self.diag.front_slip_ratio = 0.0 if front is None else clamp((front - self.velocity) / vden, -1.0, 1.0)
        self.diag.rear_slip_ratio = 0.0 if rear is None else clamp((rear - self.velocity) / vden, -1.0, 1.0)

        fw = rw = 0.0
        if update_front and front is not None:
            fw = self._measurement_update(front, f_rel)
        if update_rear and rear is not None:
            rw = self._measurement_update(rear, r_rel)
        self.diag.front_weight = fw
        self.diag.rear_weight = rw

        self._apply_standstill(t, notch, front, rear)

        # Adapt only when the observer says adhesion is overwhelmingly normal.
        p_normal = self.adhesion.p[0]
        if p_normal > 0.85 and accel_score < 0.20 and self.last_wheel_v is not None:
            self.model.adapt_bias(self.filtered_wheel_accel, notch, self.velocity, self.bias_adapt_gain)
            self.diag.adaptive_accel_bias = self.model.bias
