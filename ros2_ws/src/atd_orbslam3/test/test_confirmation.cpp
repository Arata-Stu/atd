#include "../src/confirmation.hpp"
#include <limits>
#include <stdexcept>

void check(bool condition) { if (!condition) { throw std::runtime_error("confirmation test failed"); } }

int main()
{
  atd_orbslam3::Confirmation state(3, .25, 5., 3.);
  check(!state.observe(10000000, 0., 0.));
  check(!state.observe(20000000, .01, .001));
  check(state.observe(30000000, .01, .001));
  check(!state.observe(40000000, 2., .001));
  check(state.epoch() == 1);
  check(!state.observe(50000000, .01, .001));
  check(state.observe(60000000, .01, .001));
  state.invalidate();
  state.invalidate();
  check(state.epoch() == 2);
  check(!state.observe(1000000000, 0., 0.));
  check(!state.observe(1500000000, 0., 0.));
  check(!state.observe(1510000000, 0., std::numeric_limits<double>::infinity()));
  check(!state.observe(1520000000, 0., 0.));
  check(state.observe(1530000000, 0., 0.));
}
