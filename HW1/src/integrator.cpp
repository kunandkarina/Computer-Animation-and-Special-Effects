#include "integrator.h"

#include "configs.h"

void ExplicitEuler::integrate(const std::vector<Particles *> &particles, std::function<void(void)>) const {
  // TODO: Integrate velocity and acceleration
  //   1. Integrate velocity.
  //   2. Integrate acceleration.
  //   3. You should not compute position using acceleration. Since some part only update velocity. (e.g. impulse)
  // Note:
  //   1. You don't need the simulation function in explicit euler.
  //   2. You should do this first because it is very simple. Then you can chech your collision is correct or not.
  //   3. This can be done in 5 lines. (Hint: You can add / multiply all particles at once since it is a large matrix.)
  for (const auto &p : particles) {
    p->velocity() += p->acceleration() * deltaTime;
    p->position() += p->velocity() * deltaTime;
  }
}

struct Backup {
  Eigen::Matrix4Xf position;
  Eigen::Matrix4Xf velocity;
  Eigen::Matrix4Xf acceleration;
};

void ImplicitEuler::integrate(const std::vector<Particles *> &particles,
                              std::function<void(void)> simulateOneStep) const {
  // TODO: Integrate velocity and acceleration
  //   1. Backup original particles' data.
  //   2. Integrate velocity and acceleration using explicit euler to get Xn+1.
  //   3. Compute refined Xn+1 using (1.) and (2.).
  // Note:
  //   1. Use simulateOneStep with modified position and velocity to get Xn+1.

  std::vector<Backup> origin;
  for (const auto &p : particles) {
    Backup tp;
    tp.position = p->position();
    tp.velocity = p->velocity();
    p->velocity() += p->acceleration() * deltaTime;
    p->position() += p->velocity() * deltaTime;
    origin.push_back(tp);
  }
  simulateOneStep();
  int idx = 0;
  for (const auto &p : particles) {
    p->velocity() = origin[idx].velocity + p->acceleration() * deltaTime;
    p->position() = origin[idx].position + p->velocity() * deltaTime;
    idx++;
  }
}

void MidpointEuler::integrate(const std::vector<Particles *> &particles,
                              std::function<void(void)> simulateOneStep) const {
  // TODO: Integrate velocity and acceleration
  //   1. Backup original particles' data.
  //   2. Integrate velocity and acceleration using explicit euler to get Xn+1.
  //   3. Compute refined Xn+1 using (1.) and (2.).
  // Note:
  //   1. Use simulateOneStep with modified position and velocity to get Xn+1.

  std::vector<Backup> origin;
  for (const auto &p : particles) {
    Backup tp;
    tp.position = p->position();
    tp.velocity = p->velocity();
    p->velocity() += p->acceleration() * (deltaTime / 2);
    p->position() += p->velocity() * (deltaTime / 2);
    origin.push_back(tp);
  }
  simulateOneStep();
  int idx = 0;
  for (const auto &p : particles) {
    p->velocity() = origin[idx].velocity + p->acceleration() * deltaTime;
    p->position() = origin[idx].position + p->velocity() * deltaTime;
    idx++;
  }
}

void RungeKuttaFourth::integrate(const std::vector<Particles *> &particles,
                                 std::function<void(void)> simulateOneStep) const {
  // TODO: Integrate velocity and acceleration
  //   1. Backup original particles' data.
  //   2. Compute k1, k2, k3, k4
  //   3. Compute refined Xn+1 using (1.) and (2.).
  // Note:
  //   1. Use simulateOneStep with modified position and velocity to get Xn+1.

  std::vector<Backup> origin;
  for (const auto &p : particles) {
    Backup tp;
    tp.position = p->position();
    tp.velocity = p->velocity();
    tp.acceleration = p->acceleration();
    origin.push_back(tp);
  }
  auto k1 = origin;
  int idx = 0;
  for (const auto &p : particles) {
    k1[idx].position = p->velocity() * deltaTime;
    k1[idx].velocity = p->acceleration() * deltaTime;
    idx++;
  }
  idx = 0;
  for (const auto &p : particles) {
    p->position() = origin[idx].position + k1[idx].position / 2;
    p->velocity() = origin[idx].velocity + k1[idx].velocity / 2;
    p->acceleration() = origin[idx].acceleration;
    idx++;
  }
  simulateOneStep();
  auto k2 = origin;
  idx = 0;
  for (const auto &p : particles) {
    k2[idx].position = p->velocity() * deltaTime;
    k2[idx].velocity = p->acceleration() * deltaTime;
    idx++;
  }
  idx = 0;
  for (const auto &p : particles) {
    p->position() = origin[idx].position + k2[idx].position / 2;
    p->velocity() = origin[idx].velocity + k2[idx].velocity / 2;
    p->acceleration() = origin[idx].acceleration;
    idx++;
  }
  simulateOneStep();
  auto k3 = origin;
  idx = 0;
  for (const auto &p : particles) {
    k3[idx].position = p->velocity() * deltaTime;
    k3[idx].velocity = p->acceleration() * deltaTime;
    idx++;
  }
  idx = 0;
  for (const auto &p : particles) {
    p->position() = origin[idx].position + k3[idx].position;
    p->velocity() = origin[idx].velocity + k3[idx].velocity;
    p->acceleration() = origin[idx].acceleration;
    idx++;
  }
  simulateOneStep();
  auto k4 = origin;
  idx = 0;
  for (const auto &p : particles) {
    k4[idx].position = p->velocity() * deltaTime;
    k4[idx].velocity = p->acceleration() * deltaTime;
    idx++;
  }
  idx = 0;
  for (const auto &p : particles) {
    p->position() =
        origin[idx].position + (k1[idx].position + 2 * k2[idx].position + 2 * k3[idx].position + k4[idx].position) / 6;
    p->velocity() =
        origin[idx].velocity + (k1[idx].velocity + 2 * k2[idx].velocity + 2 * k3[idx].velocity + k4[idx].velocity) / 6;
    p->acceleration() = origin[idx].acceleration;
    idx++;
  }

}
