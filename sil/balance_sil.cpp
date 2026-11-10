// Host SIL: the nonlinear plant around the bot::Controller that main.cpp flashes.
//
//   pio run -e sil -t exec
//
// One script (hold, engage, push, drive, turn, tip over) runs on the nominal plant, then on a
// plant 20 % off the nameplate with a 2° IMU mount error and every reading one period late. Then
// the pitch loop, linearized, on all 16 corners of ±20 % with the readings late.
// Exit code 0 when every check holds.

#include <cmath>
#include <complex>
#include <cstdio>

#include "balance_controller.hpp"
#include "damp/matrix/eigen.hpp"
#include "damp/systems/discretization.hpp"

namespace {

constexpr double kDeg      = 3.14159265358979323846 / 180.0;
constexpr int    kSubsteps = 10;   // plant steps per control period
constexpr double kSpread   = 0.2;  // nameplate error the design must hold

/// The nameplate off by ±spread: bits 0–3 of c set body mass, body inertia, CoM height above the
/// axle and wheel inertia high (1) or low (0).
bot::Nameplate corner(unsigned c, double spread) {
    const auto     f = [&](unsigned bit) { return ((c >> bit) & 1U) != 0U ? 1.0 + spread : 1.0 - spread; };
    bot::Nameplate n = bot::kBot;
    n.body_mass *= f(0);
    n.body_inertia *= f(1);
    n.com_height = n.wheel_radius + f(2) * (n.com_height - n.wheel_radius);
    n.wheel_inertia *= f(3);
    n.yaw_inertia *= f(0);
    return n;
}

/// Spectral radius of the sampled pitch loop on plant n, the gain the design's, every reading one
/// period late: z = [x; ξ; x_prev], u = −K·[x_prev; ξ], ξ += −Ts·p_prev.
double late_loop_radius(const bot::Nameplate& n) {
    const auto         d = damp::discretize(bot::pitch_plant(n), bot::kTs);
    const auto&        K = bot::kPitchDesign.K;
    damp::Matrix<9, 9> L{};
    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j < 4; ++j) {
            L(i, j)     = d->A(i, j);
            L(i, 5 + j) = -d->B(i, 0) * K(0, j);
        }
        L(i, 4)     = -d->B(i, 0) * K(0, 4);
        L(5 + i, i) = 1.0;  // x_prev ← x
    }
    L(4, 4)    = 1.0;
    L(4, 5)    = -bot::kTs;
    double rho = 0.0;
    for (const auto& z : damp::mat::compute_eigenvalues(L).values) {
        rho = std::fmax(rho, std::abs(std::complex<double>(z.real(), z.imag())));
    }
    return rho;
}

/// The plant as it is, which the design only estimates.
struct Truth {
    bot::Nameplate n            = bot::kBot;
    double         friction     = 0.05;   // [N·m·s/rad] per wheel, wheel against body
    double         pitch_offset = 0.0;    // [rad] the IMU reads this much more than the true lean
    bool           late         = false;  // readings one period old
};

/// Planar pitch dynamics, full nonlinear (docs/derivation.md), plus yaw.
struct Plant {
    Truth  t;
    double x = 0.0, v = 0.0, th = 0.0, w = 0.0, psi = 0.0, psi_rate = 0.0;

    struct Rates {
        double dx, dv, dth, dw, dpsi, dpsi_rate;
    };

    [[nodiscard]] Rates rates(double v_, double th_, double w_, double psi_rate_, const bot::WheelTorques& u) const {
        const auto [a, b, c] = bot::pitch_mass(t.n);
        const double r       = t.n.wheel_radius;
        const double half    = 0.5 * t.n.track_width;
        const double tau_r   = static_cast<double>(u.right) - t.friction * ((v_ + half * psi_rate_) / r - w_);
        const double tau_l   = static_cast<double>(u.left) - t.friction * ((v_ - half * psi_rate_) / r - w_);
        const double tau     = tau_r + tau_l;
        const double cs = std::cos(th_), sn = std::sin(th_);
        const double f1  = tau / r + b * sn * w_ * w_;
        const double f2  = b * t.n.gravity * sn - tau;
        const double det = a * c - b * b * cs * cs;
        return {
            .dx        = v_,
            .dv        = (c * f1 - b * cs * f2) / det,
            .dth       = w_,
            .dw        = (a * f2 - b * cs * f1) / det,
            .dpsi      = psi_rate_,
            .dpsi_rate = half / r * (tau_r - tau_l) / bot::yaw_inertia(t.n),
        };
    }

    void advance(const bot::WheelTorques& u, double dt) {  // RK4
        const Rates k1  = rates(v, th, w, psi_rate, u);
        const Rates k2  = rates(v + 0.5 * dt * k1.dv, th + 0.5 * dt * k1.dth, w + 0.5 * dt * k1.dw, psi_rate + 0.5 * dt * k1.dpsi_rate, u);
        const Rates k3  = rates(v + 0.5 * dt * k2.dv, th + 0.5 * dt * k2.dth, w + 0.5 * dt * k2.dw, psi_rate + 0.5 * dt * k2.dpsi_rate, u);
        const Rates k4  = rates(v + dt * k3.dv, th + dt * k3.dth, w + dt * k3.dw, psi_rate + dt * k3.dpsi_rate, u);
        const auto  sum = [dt](double a, double b, double c, double d) { return dt / 6.0 * (a + 2.0 * b + 2.0 * c + d); };
        x += sum(k1.dx, k2.dx, k3.dx, k4.dx);
        v += sum(k1.dv, k2.dv, k3.dv, k4.dv);
        th += sum(k1.dth, k2.dth, k3.dth, k4.dth);
        w += sum(k1.dw, k2.dw, k3.dw, k4.dw);
        psi += sum(k1.dpsi, k2.dpsi, k3.dpsi, k4.dpsi);
        psi_rate += sum(k1.dpsi_rate, k2.dpsi_rate, k3.dpsi_rate, k4.dpsi_rate);
        if (std::fabs(th) > 90.0 * kDeg) {  // on the floor
            th = std::copysign(90.0 * kDeg, th);
            v = w = 0.0;
        }
    }

    /// [N·s] horizontal impulse at the body CoM, forward positive.
    void push(double J) {
        const auto [a, b, c] = bot::pitch_mass(t.n);
        const double l       = t.n.com_height - t.n.wheel_radius;
        const double cs      = std::cos(th);
        const double q1 = J, q2 = J * l * cs;  // generalized impulse on (x, θ)
        const double det = a * c - b * b * cs * cs;
        v += (c * q1 - b * cs * q2) / det;
        w += (a * q2 - b * cs * q1) / det;
    }

    /// What the encoders and the IMU read: the wheels turn against the body.
    [[nodiscard]] bot::Measurement measure() const {
        const double r = t.n.wheel_radius, half = 0.5 * t.n.track_width;
        return {
            .pitch        = static_cast<float>(th + t.pitch_offset),
            .pitch_rate   = static_cast<float>(w),
            .wheel_r      = static_cast<float>((x + half * psi) / r - th),
            .wheel_l      = static_cast<float>((x - half * psi) / r - th),
            .wheel_rate_r = static_cast<float>((v + half * psi_rate) / r - w),
            .wheel_rate_l = static_cast<float>((v - half * psi_rate) / r - w),
        };
    }
};

int failures = 0;

void check(bool ok, const char* what, double value) {
    std::printf("  %-4s %-44s %+.4f\n", ok ? "ok" : "FAIL", what, value);
    failures += ok ? 0 : 1;
}

/// The control loop of main.cpp, closed on the plant. held: a hand keeps the bot still.
struct Sim {
    Plant            plant;
    bot::Controller  controller{};
    bot::Measurement reading{};
    bot::Output      out{};
    double           t        = 0.0;
    bool             held     = true;
    double           peak_tau = 0.0;

    void run(double seconds, bot::Command cmd = {}) {
        for (int i = 0; i < static_cast<int>(std::lround(seconds / bot::kTs)); ++i) {
            const bot::Measurement now = plant.measure();
            out                        = controller.tick(plant.t.late ? reading : now, {.imu = true, .drives = true}, cmd);
            reading                    = now;
            if (out.mode == bot::Mode::Active) {
                held = false;
            }
            peak_tau = std::fmax(peak_tau, std::fmax(std::fabs(out.torque.right), std::fabs(out.torque.left)));
            for (int k = 0; k < kSubsteps && !held; ++k) {
                plant.advance(out.torque, bot::kTs / kSubsteps);
            }
            t += bot::kTs;
        }
    }
};

void scenario(const char* name, const Truth& truth) {
    std::printf("%s\n", name);
    Sim s{.plant = {.t = truth}};
    s.plant.th = 2.0 * kDeg;  // held leaning a little

    s.run(1.9);
    check(s.out.mode == bot::Mode::Idle, "idle while held under 2 s", s.t);
    s.run(0.2);
    check(s.out.mode == bot::Mode::Active, "engaged after 2 s upright", s.t);

    const double home = s.plant.x;
    s.run(4.0);
    check(std::fabs(s.plant.th) < 0.5 * kDeg, "released at 2 deg: lean settles [deg]", s.plant.th / kDeg);
    check(std::fabs(s.plant.x - home) < 0.05, "released at 2 deg: holds the spot [m]", s.plant.x - home);

    s.plant.push(0.5);  // traction caps the torque at 2 N·m a wheel: more and it slips
    s.run(5.0);
    check(std::fabs(s.plant.x - home) < 0.05, "0.5 N.s push: back on the spot [m]", s.plant.x - home);

    s.run(6.0, {.speed = 0.5f});
    check(std::fabs(s.plant.v - 0.5) < 0.03, "drive 0.5 m/s [m/s]", s.plant.v);
    s.run(6.0);  // the tall body's zero at 3.3 rad/s (docs/derivation.md) keeps the position loop slow
    check(std::fabs(s.plant.v) < 0.02, "stop [m/s]", s.plant.v);

    s.run(5.0, {.yaw_rate = 1.0f});
    check(std::fabs(s.plant.psi_rate - 1.0) < 0.03, "turn 1 rad/s [rad/s]", s.plant.psi_rate);
    s.run(3.0);
    check(std::fabs(s.plant.psi_rate) < 0.02, "stop turning [rad/s]", s.plant.psi_rate);

    check(s.peak_tau <= static_cast<double>(bot::kWheelTorqueMax), "peak wheel torque within the limit [N.m]", s.peak_tau);

    s.plant.push(-25.0);
    s.run(1.0);
    check(s.out.mode == bot::Mode::Idle && s.out.torque.right == 0.0f, "shoved over: motors off [deg]", s.plant.th / kDeg);
}

}  // namespace

int main() {
    // Output stays ASCII: PlatformIO echoes it through the Windows console code page.
    std::printf("pitch K [p v theta omega int_p]:");
    for (size_t i = 0; i < 5; ++i) {
        std::printf(" %+.3f", bot::kPitchDesign.K(0, i));
    }
    std::printf("\nyaw   K [psi psi_rate int_psi]:   ");
    for (size_t i = 0; i < 3; ++i) {
        std::printf(" %+.3f", bot::kYawDesign.K(0, i));
    }
    std::printf("\n");

    std::printf("imu mapping\n");
    const double half = (90.0 + 5.0) * kDeg / 2.0;  // a roll of 95° about the IMU x axis
    const float  lean = bot::pitch_from_imu({static_cast<float>(std::cos(half)), static_cast<float>(std::sin(half)), 0.0f, 0.0f});
    const double want = 5.0 * kDeg - static_cast<double>(bot::kPitchTrim);
    check(std::fabs(static_cast<double>(lean) - want) < 1e-4, "IMU roll 95 deg reads a 5 deg lean, less the trim [deg]", static_cast<double>(lean) / kDeg);

    scenario("nominal plant", {});

    // Heavier, taller body, lighter wheels: the corner closest to the edge.
    scenario("plant 20 % off, IMU 2 deg off, readings one period late", {.n = corner(0b0111U, kSpread), .pitch_offset = 2.0 * kDeg, .late = true});

    std::printf("pitch loop on every corner of +-20 %%, readings one period late\n");
    double worst = 0.0;
    for (unsigned c = 0; c < 16; ++c) {
        worst = std::fmax(worst, late_loop_radius(corner(c, kSpread)));
    }
    check(worst < 1.0, "largest closed-loop |z|", worst);

    if (failures == 0) {
        std::printf("all checks hold\n");
    } else {
        std::printf("%d checks failed\n", failures);
    }
    return failures == 0 ? 0 : 1;
}
