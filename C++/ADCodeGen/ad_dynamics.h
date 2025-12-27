//
// Created by alexliniger on 24.07.20.
//

#ifndef CPPAD_TEST_AD_DYNAMICS_H
#define CPPAD_TEST_AD_DYNAMICS_H

#include <iosfwd>
#include <string>
#include <vector>

#include <cppad/cg.hpp>

#include "../Config/config.h"
#include "../Config/params.h"
#include "../Config/types.h"

namespace mpcc {

typedef CppAD::cg::CG<double> CGD;
typedef CppAD::AD<CGD> ADCG;

struct TireForces {
  ADCG F_x;
  ADCG F_y;
};

struct NormalForces {
  ADCG F_N_front;
  ADCG F_N_rear;
};

class ADDynamics {
 public:
  ADDynamics();
  ADDynamics(double Ts, const std::string &path);
  void genLibraryRK4();
  void genLibraryGetF();
  void genLibraryTireFront();
  void genLibraryTireRear();

 private:
  std::vector<ADCG> vectorAdd(std::vector<ADCG> x1, std::vector<ADCG> x2);
  std::vector<ADCG> scalerMult(std::vector<ADCG> x, double a);

  ADCG getState(std::vector<ADCG> x, int index);
  ADCG getInput(std::vector<ADCG> u, int index);

  ADCG getSlipAngleFront(std::vector<ADCG> x);
  ADCG getSlipAngleRear(std::vector<ADCG> x);
  TireForces getForceFront(std::vector<ADCG> x);
  TireForces getForceRear(std::vector<ADCG> x);
  ADCG getForceFriction(std::vector<ADCG> x);
  NormalForces getForceNormalStatic(void);
  NormalForces getForceNormalDyn(std::vector<ADCG> x);

  std::vector<ADCG> dx(std::vector<ADCG> x, std::vector<ADCG> u);
  std::vector<ADCG> RK4(std::vector<ADCG> x);
  std::vector<ADCG> f_dyn(std::vector<ADCG> x);

  std::vector<ADCG> TireConFront(std::vector<ADCG> x);
  std::vector<ADCG> TireConRear(std::vector<ADCG> x);

  Param param_;
  const double Ts_;
};
}  // namespace mpcc

#endif  // CPPAD_TEST_AD_DYNAMICS_H
