#pragma once

#include "mpcc/config.h"
#include "mpcc/types.h"
#include <array>

namespace mpcc {

struct OptVariables;
struct Stage;

class SolverInterface {
public:
    virtual std::array<OptVariables, N + 1> solveMPC(
        std::array<Stage, N + 1> &stages,
        const State &x0,
        int *status) = 0;

    virtual ~SolverInterface() {
        std::cout << "Deleting SolverInterface" << std::endl;
    }
};

} // namespace mpcc
