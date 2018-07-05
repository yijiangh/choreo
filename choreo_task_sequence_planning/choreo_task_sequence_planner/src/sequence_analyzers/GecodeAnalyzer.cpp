//
// Created by yijiangh on 1/17/18.
//

#include "choreo_task_sequence_planner/sequence_analyzers/GecodeAnalyzer.h"

#include <gecode/gist.hh>

namespace {
bool writeMiniZincData(
    const char* path,
    const int n, const int m,
    const std::vector<int>& A, const std::vector<int>& G, const std::vector<int>& T)
{
  FILE *fp = fopen(path, "wb+");

  assert(n*n == A.size());
  assert(n == G.size());
  assert(n*n*m == T.size());

  if(!fp)
  {
    ROS_ERROR("[Gecode-ts planning] fails to output minizinc data.");
    return false;
  }

  fprintf(fp, "n = %d;\n\n", n);
  fprintf(fp, "m = %d;\n\n", m);

  fprintf(fp, "G_data = [");
  for(int i = 0; i < G.size(); i++)
  {
    fprintf(fp, "%d,", G[i]);
    if(i == G.size()-1)
    {
      fprintf(fp, "];\n\n");
    }
  }

  fprintf(fp, "A_data = [");
  for(int i = 0; i < A.size(); i++)
  {
    fprintf(fp, "%d,", A[i]);
    if(i == A.size()-1)
    {
      fprintf(fp, "];\n\n");
    }
  }

  fprintf(fp, "T_data = [");
  for(int i = 0; i < T.size(); i++)
  {
    fprintf(fp, "%d,", T[i]);
    if(i == T.size()-1)
    {
      fprintf(fp, "];\n\n");
    }
  }

  if(0 == fclose(fp))
  {
    return true;
  }
  else
  {
    return false;
  }
}


} // anon util namespace

GecodeAnalyzer::~GecodeAnalyzer()
{
}

bool GecodeAnalyzer::SeqPrint()
{
  using namespace Gecode;

  gecode_analyzer_.Start();

  Init();

  /* split layers */
  /* label stores layer index of each dual node */
  int layer_size = ptr_frame_->SizeOfLayer();

  layers_.clear();
  layers_.resize(layer_size);
  for (int i = 0; i < Nd_; i++)
  {
    WF_edge *e = ptr_frame_->GetEdge(ptr_wholegraph_->e_orig_id(i));
    layers_[e->Layer()].push_back(e);
  }

  for (int l = 0; l < layer_size; l++)
  {
    fprintf(stderr, "Size of layer %d is %d\n", l, (int)layers_[l].size());
  }

  /* print starting from the first layer */
  bool bSuccess = true;
  for (int l = 0; l < layer_size; l++)
  {
    if(0 == l)
    {
      /* set pillars as starting edges */
      PrintPillars();
      continue;
    }

    int Nl = layers_[l].size();

    /* max_z_ and min_z_ in current layer */
    min_z_ = 1e20;
    max_z_ = -min_z_;

    for (int i = 0; i < Nl; i++)
    {
      WF_edge* e = layers_[l][i];
      point u = e->pvert_->Position();
      point v = e->ppair_->pvert_->Position();
      min_z_ = min(min_z_, (double)min(u.z(), v.z()));
      max_z_ = max(max_z_, (double)max(u.z(), v.z()));
    }

    // compute input for gecode solver
    int n = layers_[l].size();
    int m = ptr_collision_->Divide();
    std::vector<int> A, G, T;

    ComputeGecodeInput(layers_[l], A, G, T, CSPDataMatrixStorageType::ROW_MAJOR);

    // output minizinc data
    std::string str_path = "/home/yijiangh/Documents/";
    str_path = str_path + "as_minizinc_data_layer_" + std::to_string(l) + ".dzn";

    bSuccess = writeMiniZincData(str_path.c_str(), n, m, A, G, T);

    // trigger minizinc-gecode computation

    // fetch sequence back

    // for edge in sequence:
    //{
    // add edge in print_queue_
    // update structure
    // update state map
    //}
  }

  gecode_analyzer_.Stop();
  return bSuccess;
}

void GecodeAnalyzer::ComputeGecodeInput(const std::vector<WF_edge*>& layer_e,
                                        std::vector<int>& A, std::vector<int>& G, std::vector<int>& T,
                                        CSPDataMatrixStorageType m_type)
{
  int n = layer_e.size();
  int m = ptr_collision_->Divide();

  A = std::vector<int>(n*n, 0);
  G = std::vector<int>(n, 0);
  T = std::vector<int>(n*n*m, 0);

  for(int i=0; i < n; i++)
  {
    WF_edge* e = layer_e[i];

    // connectivity
    // if one of the nodes exist, grounded
    // st node index
    int uj = ptr_frame_->GetEndu(e->ID());
    bool exist_uj = ptr_dualgraph_->isExistingVert(uj);

    // end node index
    int vj = ptr_frame_->GetEndv(e->ID());
    bool exist_vj = ptr_dualgraph_->isExistingVert(vj);

    if(exist_uj || exist_vj || e->isPillar())
    {
      G[i] = 1;
    }

    // for current layer's edge, if share node = 1
    std::vector<int> connect_vert_id(2);
    connect_vert_id[0] = ptr_frame_->GetEndu(e->ID());
    connect_vert_id[1] = ptr_frame_->GetEndv(e->ID());

    for(const auto id : connect_vert_id)
    {
      WF_edge* eu = ptr_frame_->GetNeighborEdge(id);

      while (eu != NULL)
      {
        if (eu->ID() == e->ID() || eu->ID() == e->ppair_->ID())
        {
          eu = eu->pnext_;
          continue;
        }

        for(int j=0; j < n; j++)
        {
          if(eu->ID() == layer_e[j]->ID() || eu->ID() == layer_e[j]->ppair_->ID())
          {
            int index_2d;

            // https://en.wikipedia.org/wiki/Row-_and_column-major_order
            if(m_type == CSPDataMatrixStorageType::ROW_MAJOR)
            {
              // n1: i(n), n2: j(n);
              index_2d = j + n * i;
            }
            else
            {
              index_2d = i + n * j;
            }

            A[index_2d] = 1;
          }
        }

        eu = eu->pnext_;
      }
    }

    int dual_i = ptr_wholegraph_->e_dual_id(e->ID());

    for (int j = 0; j < n; j++)
    {
      WF_edge* ej = layer_e[j];
      int dual_j = ptr_wholegraph_->e_dual_id(ej->ID());

      if (dual_i != dual_j)
      {
        std::vector<lld> tmp(3);
        ptr_collision_->DetectCollision(e, ej, tmp);

        for (int o = 0; o < 3; o++)
        {
          tmp[o] |= angle_state_[dual_i][o];
        }

        // TODO: should use tmp as a mask and impose it on angle_map_ to include previous layers' influence
        std::vector<int> tmp_vector = ptr_collision_->ConvertCollisionMapToIntMap(tmp);

        for(int k=0; k<m; k++)
        {
          int index_3d;

          // https://en.wikipedia.org/wiki/Row-_and_column-major_order
          if(m_type == CSPDataMatrixStorageType::ROW_MAJOR)
          {
            // n1: i(n), n2: j(n), n3: k(m)
            index_3d = k + m * (j + n * i);
          }
          else
          {
            index_3d = i + n * (j + n * k);
          }

          T[index_3d] = tmp_vector[k];
        }
      }
    }

  } // end loop for i
}

void GecodeAnalyzer::PrintOutTimer()
{
  printf("***GecodeAnalyzer timer result:\n");
  gecode_analyzer_.Print("GecodeAnalyzer:");
}

void GecodeAnalyzer::debug()
{
  using namespace Gecode;
}