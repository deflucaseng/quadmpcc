// Compilation-check stub for constraints.cpp
// Verifies that the Constraints class compiles and constructs without error.
#include "mpcc/constraints.h"
#include <iostream>

int main()
{
    using namespace mpcc;

    PathToJson paths{"params/model.json", "params/cost.json",
                     "params/bounds.json", "params/normalization.json",
                     "params/waypoints.yaml"};

    Constraints con(0.02, paths);

    // Full functional tests require PathWeaver Tunnel/FrenetFrame objects which
    // need a valid waypoints YAML.  This stub confirms compilation only.
    std::cout << "constraints_test PASSED (construction check only)\n";
    return 0;
}
