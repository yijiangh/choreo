//
// Created by yijiangh on 1/17/18.
//

#ifndef FRAMEFAB_MPP_GECODEANALYZER_H
#define FRAMEFAB_MPP_GECODEANALYZER_H

#include <cmath>
#include <cstring>

#include "framefab_task_sequence_planner/sequence_analyzers/SeqAnalyzer.h"

// Gecode
// int variable & search engines
#include <gecode/driver.hh>
#include <gecode/int.hh>
#include <gecode/search.hh>
#include <gecode/minimodel.hh>

namespace Eigen
{
typedef Array<bool,Dynamic,1> ArrayXb;
typedef Matrix<bool,Dynamic,Dynamic> MatrixXb;
};

namespace Gecode
{

class AssemblySequenceOptions : public Options
{
 public:
  const Eigen::MatrixXb A;
  const Eigen::ArrayXb G;
  const std::vector<Eigen::MatrixXb> T;

  int n;
  int m;

  // convert Eigen matrix to Gecode Matrix

  AssemblySequenceOptions(const char*s,
                          const Eigen::MatrixXb& input_A,
                          const Eigen::ArrayXb& input_G,
                          const std::vector<Eigen::MatrixXb>& input_T,
                          const int input_n,
                          const int input_m)
      : Options(s), A(input_A), G(input_G), T(input_T), n(input_n), m(input_m)
  {
  }
};

class AssemblySequence : public Script
{
 protected:
  const AssemblySequenceOptions& opt;
  IntVarArray o;

 public:
  AssemblySequence(const AssemblySequenceOptions& input_opt)
      : Script(input_opt), opt(input_opt), o(*this, input_opt.n, 0, input_opt.n - 1)
  {
    int n = opt.n;
    int m = opt.m;

    // connectivity matrix
//    BoolArgs A(n*n); // int*
//    BoolArgs G(n);

    // collision matrix (per edge)
//    BoolArgs T(n*m*n);

    // create shared int array, ref. Gecode MPG Pg.70 - Tip 4.9
    BoolVarArray A;
    BoolVarArray G;
    BoolVarArray T;

    // C1: AllDiff
    distinct(*this, o);

    // C2: Connectivity
    for(int i=0; i < n; i++)
    {
      // get A's row O_i
      BoolVarArgs connect_num(i+1);

      for(int j=0; j < i; j++)
      {
        // connect_num[j] = A[o[j] + n*o[i]]
        element(*this, A, o[j], n, o[i], n, connect_num[j]);
      }

      element(*this, G, o[i], connect_num[i]);

      linear(*this, connect_num, IRT_GQ, 1);
    }

    // C4: ExistValidEEFPose
    for(int i=0; i < n; i++)
    {
      IntVarArgs B(m);

      for(int k=0; k < m; k++)
      {
        BoolVarArgs d_sum(i);

        for (int j = 0; j < i; j++)
        {
          // id = (o[i]*n + o[j])*m + k
          IntVar id = expr(*this, o[i]*(n*m) + o[j]*m + k);

          element(*this, T, id, d_sum[j]);
        }

        linear(*this, d_sum, IRT_EQ, B[k]);
      }

      linear(*this, B, IRT_GQ, 1);
    }
  }

  AssemblySequence(bool share, AssemblySequence& s)
      : Script(share, s), opt(s.opt)
  {
    o.update(*this, share, s.o);
  }

  virtual Space *copy(bool share)
  {
    return new AssemblySequence(share, *this);
  }

  void print(void) const
  {
  }

  void print(std::ostream& os) const
  {
  }

};
} // end Gecode namespace

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

  Timer gecode_analyzer_;
};

#endif //FRAMEFAB_MPP_GECODEANALYZER_H
