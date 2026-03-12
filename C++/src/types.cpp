#include "mpcc/types.h"

namespace mpcc {

StateVector stateToVector(const State &x)
{
    StateVector xk;
    xk(si_index.X)  = x.X;
    xk(si_index.Y)  = x.Y;
    xk(si_index.Z)  = x.Z;
    xk(si_index.qw) = x.qw;
    xk(si_index.qx) = x.qx;
    xk(si_index.qy) = x.qy;
    xk(si_index.qz) = x.qz;
    xk(si_index.vx) = x.vx;
    xk(si_index.vy) = x.vy;
    xk(si_index.vz) = x.vz;
    xk(si_index.p)  = x.p;
    xk(si_index.q)  = x.q;
    xk(si_index.r)  = x.r;
    xk(si_index.s)  = x.s;
    xk(si_index.T)  = x.T;
    xk(si_index.vs) = x.vs;
    return xk;
}

InputVector inputToVector(const Input &u)
{
    InputVector uk;
    uk(si_index.dT)  = u.dT;
    uk(si_index.dp)  = u.dp;
    uk(si_index.dq)  = u.dq;
    uk(si_index.dr)  = u.dr;
    uk(si_index.dvs) = u.dvs;
    return uk;
}

State vectorToState(const StateVector &xk)
{
    State x;
    x.X  = xk(si_index.X);
    x.Y  = xk(si_index.Y);
    x.Z  = xk(si_index.Z);
    x.qw = xk(si_index.qw);
    x.qx = xk(si_index.qx);
    x.qy = xk(si_index.qy);
    x.qz = xk(si_index.qz);
    x.vx = xk(si_index.vx);
    x.vy = xk(si_index.vy);
    x.vz = xk(si_index.vz);
    x.p  = xk(si_index.p);
    x.q  = xk(si_index.q);
    x.r  = xk(si_index.r);
    x.s  = xk(si_index.s);
    x.T  = xk(si_index.T);
    x.vs = xk(si_index.vs);
    return x;
}

Input vectorToInput(const InputVector &uk)
{
    Input u;
    u.dT  = uk(si_index.dT);
    u.dp  = uk(si_index.dp);
    u.dq  = uk(si_index.dq);
    u.dr  = uk(si_index.dr);
    u.dvs = uk(si_index.dvs);
    return u;
}

State arrayToState(double *xk)
{
    State x;
    x.X  = xk[si_index.X];
    x.Y  = xk[si_index.Y];
    x.Z  = xk[si_index.Z];
    x.qw = xk[si_index.qw];
    x.qx = xk[si_index.qx];
    x.qy = xk[si_index.qy];
    x.qz = xk[si_index.qz];
    x.vx = xk[si_index.vx];
    x.vy = xk[si_index.vy];
    x.vz = xk[si_index.vz];
    x.p  = xk[si_index.p];
    x.q  = xk[si_index.q];
    x.r  = xk[si_index.r];
    x.s  = xk[si_index.s];
    x.T  = xk[si_index.T];
    x.vs = xk[si_index.vs];
    return x;
}

Input arrayToInput(double *uk)
{
    Input u;
    u.dT  = uk[si_index.dT];
    u.dp  = uk[si_index.dp];
    u.dq  = uk[si_index.dq];
    u.dr  = uk[si_index.dr];
    u.dvs = uk[si_index.dvs];
    return u;
}

} // namespace mpcc
