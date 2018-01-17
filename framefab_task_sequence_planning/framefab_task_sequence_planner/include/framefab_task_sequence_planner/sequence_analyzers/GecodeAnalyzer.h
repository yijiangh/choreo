//
// Created by yijiangh on 1/17/18.
//

#ifndef FRAMEFAB_MPP_GECODEANALYZER_H
#define FRAMEFAB_MPP_GECODEANALYZER_H

#include <cmath>
#include <cstring>

#include "framefab_task_sequence_planner/sequence_analyzers/SeqAnalyzer.h"

#include <gecode/int.hh>
#include <gecode/search.hh>

namespace Gecode
{
class SendMostMoney : public Space
{
 protected:
  IntVarArray l;
 public:
  SendMostMoney(void) : l(*this, 8, 0, 9)
  {
    IntVar s(l[0]), e(l[1]), n(l[2]), d(l[3]), m(l[4]),
        o(l[5]), t(l[6]), y(l[7]);
    rel(*this, s, IRT_NQ, 0);
    rel(*this, m, IRT_NQ, 0);
    distinct(*this, l);
    IntArgs c(4 + 4 + 5);
    IntVarArgs x(4 + 4 + 5);
    c[0] = 1000;
    c[1] = 100;
    c[2] = 10;
    c[3] = 1;
    x[0] = s;
    x[1] = e;
    x[2] = n;
    x[3] = d;
    c[4] = 1000;
    c[5] = 100;
    c[6] = 10;
    c[7] = 1;
    x[4] = m;
    x[5] = o;
    x[6] = s;
    x[7] = t;
    c[8] = -10000;
    c[9] = -1000;
    c[10] = -100;
    c[11] = -10;
    c[12] = -1;
    x[8] = m;
    x[9] = o;
    x[10] = n;
    x[11] = e;
    x[12] = y;
    linear(*this, c, x, IRT_EQ, 0);
    branch(*this, l, INT_VAR_SIZE_MIN(), INT_VAL_MIN());
  }
  SendMostMoney(bool share, SendMostMoney &s) : Space(share, s)
  {
    l.update(*this, share, s.l);
  }
  virtual Space *copy(bool share)
  {
    return new SendMostMoney(share, *this);
  }
  void print(void) const
  {
    std::cout << l << std::endl;
  }
  // constrain function
  virtual void constrain(const Space &_b)
  {
    const SendMostMoney &b = static_cast<const SendMostMoney &>(_b);
    IntVar e(l[1]), n(l[2]), m(l[4]), o(l[5]), y(l[7]);
    IntVar b_e(b.l[1]), b_n(b.l[2]), b_m(b.l[4]),
        b_o(b.l[5]), b_y(b.l[7]);
    int money = (10000 * b_m.val() + 1000 * b_o.val() + 100 * b_n.val() +
        10 * b_e.val() + b_y.val());
    IntArgs c(5);
    IntVarArgs x(5);
    c[0] = 10000;
    c[1] = 1000;
    c[2] = 100;
    c[3] = 10;
    c[4] = 1;
    x[0] = m;
    x[1] = o;
    x[2] = n;
    x[3] = e;
    x[4] = y;
    linear(*this, c, x, IRT_GR, money);
  }
};
}

class GecodeAnalyzer : public SeqAnalyzer
{
 public:
  typedef Eigen::MatrixXd MX;
  typedef Eigen::Matrix3d M3;
  typedef Eigen::VectorXd VX;
  typedef Eigen::Vector3d V3;

 public:
  explicit GecodeAnalyzer(
      DualGraph			*ptr_dualgraph,
      QuadricCollision	*ptr_collision,
      Stiffness			*ptr_stiffness,
      FiberPrintPARM	*ptr_parm,
      char				*ptr_path,
      bool				terminal_output,
      bool				file_output,
      descartes_core::RobotModelPtr hotend_model,
      moveit::core::RobotModelConstPtr moveit_model,
      std::string hotend_group_name
  ) noexcept
      : SeqAnalyzer(ptr_dualgraph, ptr_collision, ptr_stiffness,
                    ptr_parm, ptr_path, terminal_output, file_output,
                    hotend_model, moveit_model, hotend_group_name){}
  ~GecodeAnalyzer();

 public:
  bool SeqPrint();
  void debug();

 private:
  bool	 GenerateSeq(int l, int h, int t);
  double GenerateCost(WF_edge* ei, WF_edge* ej, const int h, const int t, const int layer_id);

 public:
  void PrintOutTimer();

 private:
  std::vector<std::vector<WF_edge*>> layers_; // store dual_node's id for each layers

  double min_z_;
  double max_z_;

  Gecode::SendMostMoney gecode_instance_;

  Timer gecode_analyzer_;
};

#endif //FRAMEFAB_MPP_GECODEANALYZER_H
