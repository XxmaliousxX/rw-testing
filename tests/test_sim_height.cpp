// Exercise the actual engine's private ray caster without exporting a new API.
#include "../sim/sim-engine.cpp"
#include <cstdio>
#include <cstdlib>

using namespace rwsim;
static int checks = 0;
static void expect(double actual, double expected, const char* label) {
  ++checks;
  if (std::fabs(actual - expected) > 1e-7 || !std::isfinite(actual)) {
    std::fprintf(stderr, "%s: expected %.8f, got %.8f\n", label, expected, actual);
    std::exit(1);
  }
}

int main() {
  world.field_half_in = 70.25;
  circles = {{10, 0, 2, "goal", 3.25}, {20, 0, 2, "goal", 5.77}};
  boxes = {{28, -2, 32, 2, "stack", 6.48}};
  expect(castRay(0, 0, 1, 0), 8, "legacy default Z");
  expect(castRay(0, 0, 1, 0, 0), 8, "explicit zero Z");
  expect(castRay(0, 0, 1, 0, 3.0), 8, "below circle");
  expect(castRay(0, 0, 1, 0, 3.25), 8, "exact circle top");
  expect(castRay(0, 0, 1, 0, 3.26), 18, "above first circle, hits second");
  expect(castRay(0, 0, 1, 0, 6.0), 28, "above both circles, below box");
  expect(castRay(0, 0, 1, 0, 6.48), 28, "exact box top");
  expect(castRay(0, 0, 1, 0, 6.49), 70.25, "above box, hits wall");
  expect(castRay(0, 0, 0, -1, 50), 70.25, "wall at any height");
  boxes.push_back({38, -2, 42, 2, "stack"});
  expect(castRay(0, 0, 1, 0, 50), 38, "unknown box blocks as before");
  circles.push_back({35, 0, 1, "goal"});
  expect(castRay(0, 0, 1, 0, 50), 34, "unknown circle blocks as before");

  // Exercise all four DistanceMount entries through real sensor sampling.
  RobotSetup robot_config;
  robot_config.distance_sensors = {{4, 0, 0, 0, 2}, {8, 0, 0, 90, 4},
                                   {12, 0, 0, 180, 6}, {16, 0, 0, 270, 8}};
  WorldSetup world_config;
  world_config.start_x = world_config.start_y = 0;
  world_config.realistic = false;
  world_config.override_field = false;
  setup(robot_config, world_config, {});
  circles = {{0, 10, 2, "goal", 3}, {10, 0, 2, "goal", 3},
             {0, -10, 2, "goal", 7}, {-10, 0, 2, "goal", 7}};
  physicsStep(0.001);
  expect(distanceMm(4), 8 * 25.4, "front mount Z");
  expect(distanceMm(8), 70.25 * 25.4, "right mount Z");
  expect(distanceMm(12), 8 * 25.4, "back mount Z");
  expect(distanceMm(16), 70.25 * 25.4, "left mount Z");

  // Verify the rotated Pinnacle layout's known height assignments.
  // Old field rays are rotated by (x,y) -> (-y,x), including direction.
  world_config.override_field = true;
  setup(robot_config, world_config, {});
  expect(castRay(0, -47.09, -1, 0, 3.25), 23.55 - 2.8, "alliance goal height");
  expect(castRay(0, -47.09, -1, 0, 3.26), 70.25, "above alliance goal and floor pins");
  expect(castRay(-30, 0, -1, 0, 5.77), 47.09 - 30 - 2.8, "neutral goal height");
  expect(castRay(-30, 0, -1, 0, 5.78), 70.25 - 30, "above neutral goal");
  expect(castRay(0, -10, 0, 1, 0), 10 - 2.8, "center ground footprint unchanged");
  expect(castRay(0, -10, 0, 1, 8.77), 10 - 2.8, "center body at rim");
  expect(castRay(0, -10, 0, 1, 9), 10 - 1.58, "center exposed pin is narrower");
  expect(castRay(-2, -10, 0, 1, 9), 80.25, "high ray misses center pin off-axis");
  expect(castRay(0, -10, 0, 1, 307.0 / 25.4), 10 - 1.58, "center pin exact top");
  expect(castRay(0, -10, 0, 1, 307.0 / 25.4 + 0.01), 80.25, "above center pin");
  expect(castRay(-35.325, 55, 0, 1, 9.7007874), 68.7 - 1.6 - 55, "stack exact top");
  expect(castRay(-35.325, 55, 0, 1, 9.71), 70.25 - 55, "above stack");
  expect(castRay(-58.7, 55, 0, 1, 11.5), 68.5 - 2 - 55, "loader exact top");
  expect(castRay(-58.7, 55, 0, 1, 11.51), 70.25 - 55, "above loader, wall still blocks");
  // Check actual alliance colors, center locations, and rotated box extents.
  for (int side : {-1, 1}) {
    const char* color = side < 0 ? "red" : "blue";
    for (int end : {-1, 1}) {
      int goal_count = 0, loader_count = 0;
      for (const Circle& c : circles) {
        if (std::string(c.color) == color && c.r == 2.8 &&
            c.x == end * 23.55 && c.y == side * 47.09) ++goal_count;
      }
      for (const Box& b : boxes) {
        if (std::string(b.color) == color && b.height_in == INSTALLED_LOADER_HEIGHT_IN &&
            std::fabs((b.x0 + b.x1)/2 - end * 58.7) < 1e-7 &&
            std::fabs((b.y0 + b.y1)/2 - side * 68.5) < 1e-7) {
          ++loader_count;
          expect(b.x1 - b.x0, 5.0, "rotated loader width");
          expect(b.y1 - b.y0, 4.0, "rotated loader depth");
        }
      }
      expect(goal_count, 1, "rotated alliance goal center and color");
      expect(loader_count, 1, "rotated loader center and color");
    }
  }
  expect(castRay(-30, 0, -1, 0, 5.77), 47.09 - 30 - 2.8, "west neutral");
  expect(castRay(30, 0, 1, 0, 5.77), 47.09 - 30 - 2.8, "east neutral");
  expect(castRay(-58.7, -55, 0, -1, 11.5), 68.5 - 2 - 55, "southwest red loader");
  std::printf("Simulator height tests: %d passed\n", checks);
}
