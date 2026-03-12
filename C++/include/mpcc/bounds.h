#pragma once

#include "mpcc/config.h"
#include "mpcc/types.h"
#include "mpcc/params.h"

namespace mpcc {

class Bounds {
public:
    Bounds();
    explicit Bounds(const BoundsParam &bounds_param);

    Bounds_x getBoundsLX() const;
    Bounds_x getBoundsUX() const;
    Bounds_u getBoundsLU() const;
    Bounds_u getBoundsUU() const;
    Bounds_s getBoundsLS() const;
    Bounds_s getBoundsUS() const;

private:
    Bounds_x l_bounds_x_;
    Bounds_x u_bounds_x_;
    Bounds_u l_bounds_u_;
    Bounds_u u_bounds_u_;
    Bounds_s l_bounds_s_;
    Bounds_s u_bounds_s_;
};

} // namespace mpcc
