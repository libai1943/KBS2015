// Chapter 2 / Li and Shao, Knowledge-Based Systems 86 (2015), 11-20.
// Case 3 from LoadCase.m. The fixed poses match this repository's MATLAB demo.
// Forward Euler, 200 configurations, minimum time, bidirectional area
// constraints. Rectangle vertices are algebraically eliminated from the
// original NLP.mod.
#include <algorithm>
#include <array>
#include <casadi/casadi.hpp>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>
extern "C" int plan_rigid_path(const double *, const double *, const double *,
                               const double *, int, int, double *, int, char *,
                               int);
using casadi::DM;
using casadi::SX;
constexpr int N = 200, Q = 7;
using Pose = std::array<double, 3>;
using Point = std::array<SX, 2>;
using Rectangle = std::array<Point, 4>;
const std::array<double, 6> boundary{-10, 6, 0,
                                     0,   2, 1.45 * 3.14159265358979323846};
const std::array<double, 16> obstacles{
    -1.944, -2.353, -3.650, -2.355, -3.656, 2.195, -1.951, 2.197,
    2.175,  -2.585, .375,   -2.021, 1.825,  2.611, 3.625,  2.047};
SX area_sum(const Rectangle &r, const Point &p) {
  SX sum = 0;
  for (int k = 0; k < 4; ++k) {
    auto a = r[k], b = r[(k + 1) % 4];
    sum += fabs((a[0] - p[0]) * (b[1] - p[1]) - (a[1] - p[1]) * (b[0] - p[0])) *
           .5;
  }
  return sum;
}
std::vector<Pose> initialize(double &length) {
  double box[]{-15, 15, -15, 15}, vehicle[]{2.8, 3.76, .929, 1.942, .5};
  std::vector<double> buffer(300000);
  char message[512]{};
  int count = plan_rigid_path(box, vehicle, boundary.data(), obstacles.data(),
                              2, 1, buffer.data(), 100000, message, 512);
  if (count < 2)
    throw std::runtime_error(message);
  std::vector<double> distance(count);
  for (int i = 1; i < count; ++i)
    distance[i] =
        distance[i - 1] + std::hypot(buffer[3 * i] - buffer[3 * i - 3],
                                     buffer[3 * i + 1] - buffer[3 * i - 2]);
  length = distance.back();
  std::vector<Pose> result(N);
  int j = 1;
  for (int i = 0; i < N; ++i) {
    double target = length * i / (N - 1);
    while (j + 1 < count && distance[j] < target)
      ++j;
    double fraction = (target - distance[j - 1]) /
                      std::max(1e-12, distance[j] - distance[j - 1]);
    for (int k = 0; k < 3; ++k)
      result[i][k] = buffer[3 * (j - 1) + k] +
                     fraction * (buffer[3 * j + k] - buffer[3 * (j - 1) + k]);
  }
  return result;
}
int main(int argc, char **argv) {
  try {
    std::filesystem::path output = "results";
    std::string linear_solver = "mumps";
    for (int i = 1; i < argc; ++i) {
      std::string a = argv[i];
      if (a == "--output" && i + 1 < argc)
        output = std::filesystem::u8path(argv[++i]);
      else if (a == "--linear-solver" && i + 1 < argc)
        linear_solver = argv[++i];
      else
        throw std::runtime_error(
            "Usage: kbs_demo [--output DIR] [--linear-solver mumps]");
    }
    auto begin = std::chrono::steady_clock::now();
    double length;
    auto reference = initialize(length);
    std::cout << "Hybrid A* path length=" << length << std::endl;
    SX z = SX::sym("trajectory", N * Q + 1), tf = z(N * Q), h = tf / (N - 1);
    auto q = [&](int i, int j) { return z(i * Q + j); };
    std::vector<double> lower(N * Q + 1, -1e20), upper(N * Q + 1, 1e20),
        guess(N * Q + 1), gl, gu;
    std::vector<SX> expressions;
    auto add = [&](SX e, double l = 0, double u = 0) {
      expressions.push_back(e);
      gl.push_back(l);
      gu.push_back(u);
    };
    lower.back() = .1;
    upper.back() = 50;
    guess.back() = std::min(49., length / 0.8 + 4);
    double dt = guess.back() / (N - 1);
    for (int i = 0; i < N; ++i) {
      for (int k = 0; k < 3; ++k)
        guess[i * Q + k] = reference[i][k];
      for (int k = 3; k < 7; ++k) {
        double b = k < 5 ? 1 : k == 5 ? .5 : .35;
        lower[i * Q + k] = -b;
        upper[i * Q + k] = b;
      }
      if (i < N - 1) {
        double dx = reference[i + 1][0] - reference[i][0],
               dy = reference[i + 1][1] - reference[i][1];
        double ds = dx * cos(reference[i][2]) + dy * sin(reference[i][2]);
        guess[i * Q + 3] = std::clamp(ds / dt, -1., 1.);
        guess[i * Q + 5] =
            std::clamp(atan(2.8 * (reference[i + 1][2] - reference[i][2]) /
                            (std::abs(ds) < 1e-9 ? 1e-9 : ds)),
                       -.5, .5);
      }
      SX c = cos(q(i, 2)), s = sin(q(i, 2));
      Rectangle body;
      std::array<double, 4> longitudinal{3.76, 3.76, -.929, -.929},
          lateral{.971, -.971, -.971, .971};
      for (int k = 0; k < 4; ++k)
        body[k] = {q(i, 0) + longitudinal[k] * c - lateral[k] * s,
                   q(i, 1) + longitudinal[k] * s + lateral[k] * c};
      for (int ob = 0; ob < 2; ++ob) {
        Rectangle obstacle;
        int offset = 8 * ob;
        for (int k = 0; k < 4; ++k)
          obstacle[k] = {SX(obstacles[offset + 2 * k]),
                         SX(obstacles[offset + 2 * k + 1])};
        double area =
            std::hypot(obstacles[offset] - obstacles[offset + 2],
                       obstacles[offset + 1] - obstacles[offset + 3]) *
            std::hypot(obstacles[offset] - obstacles[offset + 6],
                       obstacles[offset + 1] - obstacles[offset + 7]);
        for (int k = 0; k < 4; ++k) {
          add(area_sum(body, obstacle[k]), 4.689 * 1.942 + .01, 1e20);
          add(area_sum(obstacle, body[k]), area + .01, 1e20);
        }
      }
      if (i < N - 1) {
        add(q(i + 1, 0) - q(i, 0) - h * q(i, 3) * cos(q(i, 2)));
        add(q(i + 1, 1) - q(i, 1) - h * q(i, 3) * sin(q(i, 2)));
        add(q(i + 1, 2) - q(i, 2) - h * q(i, 3) * tan(q(i, 5)) / 2.8);
        add(q(i + 1, 3) - q(i, 3) - h * q(i, 4));
        add(q(i + 1, 5) - q(i, 5) - h * q(i, 6));
      }
    }
    for (int i : {0, N - 1}) {
      int offset = i ? 3 : 0;
      for (int j = 0; j < 3; ++j) {
        double value = boundary[offset + j];
        if (i && j == 2)
          value = reference.back()[2]; // same orientation on the search path's
                                       // unwrapped angle branch
        lower[i * Q + j] = upper[i * Q + j] = guess[i * Q + j] = value;
      }
      for (int j = 3; j < 7; ++j)
        lower[i * Q + j] = upper[i * Q + j] = guess[i * Q + j] = 0;
    }
    casadi::Dict options;
    options["print_time"] = false;
    options["ipopt.print_level"] = 0;
    options["ipopt.sb"] = "yes";
    options["ipopt.tol"] = 1e-7;
    options["ipopt.max_iter"] = 4000;
    options["ipopt.max_cpu_time"] = 300.;
    options["ipopt.linear_solver"] = linear_solver;
    if (const char *hsl = std::getenv("HSL_LIBRARY"))
      options["ipopt.hsllib"] = hsl;
    auto solver = casadi::nlpsol(
        "kbs", "ipopt",
        casadi::SXDict{{"x", z}, {"f", tf}, {"g", SX::vertcat(expressions)}},
        options);
    auto result = solver(casadi::DMDict{{"x0", guess},
                                        {"lbx", lower},
                                        {"ubx", upper},
                                        {"lbg", gl},
                                        {"ubg", gu}});
    if (!bool(solver.stats().at("success")))
      throw std::runtime_error(solver.stats().at("return_status").to_string());
    auto solution = result.at("x").nonzeros(),
         constraints = result.at("g").nonzeros();
    double residual = 0;
    for (size_t i = 0; i < solution.size(); ++i)
      residual =
          std::max({residual, lower[i] - solution[i], solution[i] - upper[i]});
    for (size_t i = 0; i < constraints.size(); ++i)
      residual =
          std::max({residual, gl[i] - constraints[i], constraints[i] - gu[i]});
    if (residual > 1e-5)
      throw std::runtime_error("NLP constraint validation failed");
    std::filesystem::create_directories(output);
    std::ofstream file(output / "trajectory.csv");
    file << std::setprecision(17) << "t,x,y,theta,v,a,phi,omega\n";
    for (int i = 0; i < N; ++i) {
      file << i * solution.back() / (N - 1);
      for (int j = 0; j < Q; ++j)
        file << ',' << solution[i * Q + j];
      file << '\n';
    }
    std::cout << "Case 3: Tf=" << solution.back() << " s, nodes=" << N
              << ", residual=" << residual << ", planning="
              << std::chrono::duration<double>(
                     std::chrono::steady_clock::now() - begin)
                     .count()
              << " s\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
