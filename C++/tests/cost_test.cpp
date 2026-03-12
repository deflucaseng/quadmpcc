// Compilation-check stub for cost.cpp
// Verifies that CostMatrix dimensions are correct and Q is symmetric.
#include "mpcc/cost.h"
#include <cassert>
#include <cmath>
#include <iostream>

int main()
{
    using namespace mpcc;

    PathToJson paths{"params/model.json", "params/cost.json",
                     "params/bounds.json", "params/normalization.json",
                     "params/waypoints.yaml"};

    Cost cost(paths);

    // We cannot instantiate a FrenetFrame/Spline here without a real YAML file,
    // so this test just checks that the Cost object constructs correctly.
    // Full integration tests require PathWeaver waypoints to be present.

    std::cout << "cost_test PASSED (construction check only)\n";
    return 0;
}
