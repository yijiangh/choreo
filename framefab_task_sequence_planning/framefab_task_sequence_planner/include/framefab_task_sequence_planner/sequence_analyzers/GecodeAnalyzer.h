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

namespace Gecode
{
class AssemblySequenceOptions : public Options
{
 public:
  std::vector<int> A;
  std::vector<int> G;
  std::vector<int> T;

  int n;
  int m;

  // convert Eigen matrix to Gecode Matrix

  AssemblySequenceOptions(const char* s,
                          const std::vector<int>& _A,
                          const std::vector<int>& _G,
                          const std::vector<int>& _T,
                          const int _n,
                          const int _m)
      : Options(s), A(_A), G(_G), T(_T), n(_n), m(_m)
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

    // create shared int array, ref. Gecode MPG Pg.70 - Tip 4.9
    // connectivity matrix
    IntArgs A(opt.A);
    IntArgs G(opt.G);

    // collision matrix
//    IntArgs T(opt.T);

    // C1: AllDiff
    distinct(*this, o);

    // C2: Connectivity
    for(int i=0; i < n; i++)
    {
      // get A's row O_i
      IntVarArgs connect_num(i+1);

      for(int j=0; j < i; j++)
      {
        // connect_num[j] = A[o[j] + n*o[i]]
        element(*this, A, o[j], n, o[i], n, connect_num[j]);
      }

      element(*this, G, o[i], connect_num[i]);

      linear(*this, connect_num, IRT_GQ, 1);
    }

    // C4: ExistValidEEFPose
//    for(int i=0; i < n; i++)
//    {
//      // num of collision free prev edge j's num for each direction k
//      IntVarArgs B(m);
//      IntArgs free_num(m);
//
//      for(int k=0; k < m; k++)
//      {
//        BoolVarArgs d_sum(i);
//
//        for (int j = 0; j < i; j++)
//        {
//          // id = (o[i]*n + o[j])*m + k
//          IntVar id = expr(*this, o[i]*(n*m) + o[j]*m + k);
//
//          // d_sum[j] = T[o_i][o_j][k]
//          element(*this, T, id, d_sum[j]);
//        }
//
//        // B[k] = sum{j} d_sum[j]
//        linear(*this, d_sum, IRT_EQ, B[k]);
//
//        free_num[k] = i-1;
//      }
//      count(*this, B, free_num, IRT_GQ, 1);
//    }

//    branch(*this, o, INT_VAR_MIN_MIN(), INT_VAL_SPLIT_MIN());
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

  void print(std::ostream& os) const
  {
    os << "\tAssembly Sequence(n: "
       << opt.n << ", m: " << opt.m << ")"
       << std::endl;

    os << "Order: " << o << std::endl;

//    Matrix<BoolVarArray> p(_p,opt.b,opt.v);

//    for (int i = 0; i<opt.v; i++)
//    {
//      os << "\t\t";
//      for (int j = 0; j<opt.b; j++)
//      {
//        os << p(j, i) << " ";
//      }
//
//      os << std::endl;
//    }

    os << std::endl;
  }
};

} // end Gecode namespace

class GecodeAnalyzer : public SeqAnalyzer
{
 public:
  enum CSPDataMatrixStorageType
  {
    ROW_MAJOR,
    COL_MAJOR
  };

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
  void ComputeGecodeInput(
      const std::vector<WF_edge*>& layer_e,
      std::vector<int>& A, std::vector<int>& G, std::vector<int>& T, CSPDataMatrixStorageType m_type);

  bool GenerateSeq(int l, int h, int t);
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
