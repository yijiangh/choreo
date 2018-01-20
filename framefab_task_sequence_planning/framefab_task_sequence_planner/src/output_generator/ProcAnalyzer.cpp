#include <ros/console.h>

#include "framefab_task_sequence_planner/output_generator/ProcAnalyzer.h"

// for task sequence planning result
#include "framefab_task_sequence_planner/sequence_analyzers/SeqAnalyzer.h"

#include <framefab_rapidjson/include/rapidjson/document.h>
#include <framefab_rapidjson/include/rapidjson/prettywriter.h>
#include <framefab_rapidjson/include/rapidjson/filewritestream.h>

const static double FF_TRUNC_SCALE = 0.001;

ProcAnalyzer::ProcAnalyzer()
{
}

ProcAnalyzer::~ProcAnalyzer()
{
}

ProcAnalyzer::ProcAnalyzer(SeqAnalyzer *seqanalyzer, const char *path)
{
  ptr_seqanalyzer_ = seqanalyzer;
  path_ = path;
  debug_ = false;

  ROS_INFO_STREAM("[ts planner] json output path: " << std::string(path_));

  // 60 degrees
  MaxEdgeAngle_ = F_PI / 18 * 6;
}

bool ProcAnalyzer::ProcPrint()
{
  WireFrame *ptr_frame = ptr_seqanalyzer_->ptr_frame_;
  DualGraph *ptr_dualgraph = ptr_seqanalyzer_->ptr_dualgraph_;

  QuadricCollision collision_checker = QuadricCollision(ptr_frame);

  std::vector<SingleTaskPlanningResult> planning_result;
  ptr_seqanalyzer_->OutputTaskSequencePlanningResult(planning_result);

  process_list_.clear();
  process_list_.reserve(planning_result.size());

  support_ = 0;

  // extracting feasible angles
  for (int i = 0; i < planning_result.size(); i++)
  {
    Process temp;
    WF_edge* e = planning_result[i].e_;
    temp.wireframe_id_ = e->ID();

    if (e->isPillar())
    {
      support_++;
    }

    for(const auto& v : planning_result[i].eef_directions_)
    {
      temp.normal_.push_back(GeoV3(v[0], v[1], v[2]));
    }

    process_list_.push_back(temp);
  }

  // extracting start and end nodes
  point prev_end_node(0, 0, 0);

  for (int i = 0; i < planning_result.size(); i++)
  {
    Process temp = process_list_[i];
    WF_edge* e = planning_result[i].e_;

    if (e->isPillar())
    {
      if ((e->pvert_->Position()).z() > (e->ppair_->pvert_->Position()).z())
      {
        temp.end_ = e->pvert_->Position();
        temp.start_ = e->ppair_->pvert_->Position();
      }
      else
      {
        temp.start_ = e->pvert_->Position();
        temp.end_ = e->ppair_->pvert_->Position();
      }

      temp.fan_state_ = true;

      // pillar shouldn't have been printed yet
      if (!IfPointInVector(temp.end_))
      {
        exist_point_.push_back(temp.end_);
      }
      if (!IfPointInVector(temp.start_))
      {
        exist_point_.push_back(temp.start_);
      }
    }
    else
    {
      // non-pillar element
      // init
      point start_node = e->pvert_->Position();
      point end_node = e->ppair_->pvert_->Position();
      const bool start_node_exist = IfPointInVector(start_node);
      const bool end_node_exist = IfPointInVector(end_node);

      // sanity check: at least one of the nodes should exist
//      assert(start_node_exist || end_node_exist);
      // TODO: temp
      if(!start_node_exist && !end_node_exist)
      {
        temp.fan_state_ = true;
        temp.start_ = start_node;
        temp.end_ = end_node;
      }

      // XOR - only one of them exist, "create type"
      if (start_node_exist != end_node_exist)
      {
        temp.fan_state_ = true;
        if (start_node_exist)
        {
          temp.start_ = start_node;
          temp.end_ = end_node;
        }
        else
        {
          temp.start_ = end_node;
          temp.end_ = start_node;
        }
        exist_point_.push_back(temp.end_);
      }
      else
      {
        // AND - both of them exist, "connect type"
        // use previous end point as start node if possible
        // i.e. prefer continuous printing
        if (prev_end_node == end_node || prev_end_node == start_node)
        {
          if (prev_end_node == end_node)
          {
            point tmp_swap = start_node;
            start_node = end_node;
            end_node = tmp_swap;
          }
          //else start node already agrees with prev_end_node, then keep rolling!
        }
        else
        {
          // we prefer the start node to be close
          if (trimesh::dist(end_node, prev_end_node) < trimesh::dist(start_node, prev_end_node))
          {
            point tmp_swap = start_node;
            start_node = end_node;
            end_node = tmp_swap;
          }
        }

        temp.fan_state_ = false;
        temp.start_ = start_node;
        temp.end_ = end_node;
      }
    }

    prev_end_node = temp.end_;
    process_list_[i] = temp;
  } // end loop for all elements (planning_result)

//  for (int i = 0; i < process_list_.size(); i++)
//  {
//    if (process_list_[i].fan_state_)
//    {
//      // prune orientation domain based on fabrication constraint
//      // for "create type", we only allow orientations whose angle to the element
//      // is smaller than max_edge_angle
//      Filter(process_list_[i]);
//    }
//    else
//    {
//      // disable pruning on "connect type" domain prunning
//      CheckProcess(process_list_[i]);
//    }
//  }

  return(WriteJson());
}

bool ProcAnalyzer::WriteJson()
{
  using namespace rapidjson;

  std::string path = path_;
  std::cout << path << std::endl;

  // document is the root of a json message
  rapidjson::Document document;

  // define the document as an object rather than an array
  document.SetObject();

  // must pass an allocator when the object may need to allocate memory
  rapidjson::Document::AllocatorType& allocator = document.GetAllocator();

  Value model_object_container(rapidjson::kArrayType);
  document.AddMember("element_number", process_list_.size(), allocator);
  document.AddMember("support_number", support_, allocator);

  trimesh::vec3 base_center_pt = ptr_seqanalyzer_->ptr_frame_->GetBaseCenterPos();

  rapidjson::Value bc_pt(rapidjson::kArrayType);
  bc_pt.PushBack(Value().SetDouble(truncDigits(base_center_pt.x(), FF_TRUNC_SCALE)), allocator);
  bc_pt.PushBack(Value().SetDouble(truncDigits(base_center_pt.y(), FF_TRUNC_SCALE)), allocator);
  bc_pt.PushBack(Value().SetDouble(truncDigits(base_center_pt.z(), FF_TRUNC_SCALE)), allocator);
  document.AddMember("base_center_pt", bc_pt, allocator);

  for (int i = 0; i < process_list_.size(); i++)
  {
    rapidjson::Value element_object_container(rapidjson::kObjectType);

    Process temp = process_list_[i];
    element_object_container.AddMember("order_id", i, allocator);
    element_object_container.AddMember("wireframe_id", temp.wireframe_id_, allocator);

    // start & end node coordination
    point p_st = temp.start_;

    rapidjson::Value st_pt(rapidjson::kArrayType);
    st_pt.PushBack(Value().SetDouble(truncDigits(p_st.x(), FF_TRUNC_SCALE)), allocator);
    st_pt.PushBack(Value().SetDouble(truncDigits(p_st.y(), FF_TRUNC_SCALE)), allocator);
    st_pt.PushBack(Value().SetDouble(truncDigits(p_st.z(), FF_TRUNC_SCALE)), allocator);

    point p_end = temp.end_;
    rapidjson::Value end_pt(rapidjson::kArrayType);
    end_pt.PushBack(Value().SetDouble(truncDigits(p_end.x(), FF_TRUNC_SCALE)), allocator);
    end_pt.PushBack(Value().SetDouble(truncDigits(p_end.y(), FF_TRUNC_SCALE)), allocator);
    end_pt.PushBack(Value().SetDouble(truncDigits(p_end.z(), FF_TRUNC_SCALE)), allocator);

    element_object_container.AddMember("start_pt", st_pt, allocator);
    element_object_container.AddMember("end_pt", end_pt, allocator);

    // element type
    rapidjson::Value type_object(rapidjson::kObjectType);
    if(i < support_)
    {
      element_object_container.AddMember("type", "support", allocator);
    }
    else
    {
      if(0 == temp.fan_state_)
      {
        element_object_container.AddMember("type", "connect", allocator);
      }
      else
      {
        element_object_container.AddMember("type", "create", allocator);
      }
    }

    // feasible orientations
//    assert(temp.normal_.size() != 0);
    if (temp.normal_.size() == 0)
    {
      cout << "Error: normal vector empty" << endl;
      return false;
    }
    rapidjson::Value feasible_orients(rapidjson::kArrayType);
    feasible_orients.Clear();

    if(i < support_)
    {
      rapidjson::Value feasible_orient(rapidjson::kArrayType);
      feasible_orient.PushBack(Value().SetDouble(0.0), allocator);
      feasible_orient.PushBack(Value().SetDouble(0.0), allocator);
      feasible_orient.PushBack(Value().SetDouble(1.0), allocator);

      std::string vec_id = "f_orient" + std::to_string(0);
      Value vec_id_key(vec_id.c_str(), allocator);
//      feasible_orients.AddMember(vec_id_key, feasible_orient, allocator);
      feasible_orients.PushBack(feasible_orient, allocator);
    }
    else
    {
      for (int j = 0; j < temp.normal_.size(); j++)
      {
        if (temp.normal_[j].getZ() < 0)
        {
          // filter out orientation point down
          continue;
        }
        rapidjson::Value feasible_orient(rapidjson::kArrayType);
        feasible_orient.PushBack(Value().SetDouble(truncDigits(temp.normal_[j].getX(), FF_TRUNC_SCALE)), allocator);
        feasible_orient.PushBack(Value().SetDouble(truncDigits(temp.normal_[j].getY(), FF_TRUNC_SCALE)), allocator);
        feasible_orient.PushBack(Value().SetDouble(truncDigits(temp.normal_[j].getZ(), FF_TRUNC_SCALE)), allocator);

        std::string vec_id = "f_orient" + std::to_string(j);
        Value vec_id_key(vec_id.c_str(), allocator);
//        feasible_orients.AddMember(vec_id_key, feasible_orient, allocator);
        feasible_orients.PushBack(feasible_orient, allocator);
      }
    }

    element_object_container.AddMember(
        "feasible_orientation", feasible_orients, allocator);

    std::string id = "element" + std::to_string(i);
    Value id_key(id.c_str(), allocator);

    model_object_container.PushBack(element_object_container, allocator);
//    document.AddMember(id_key, element_object_container, allocator);
  }

  document.AddMember("sequenced_elements", model_object_container, allocator);

  // output file to path
  std::string json_path = path;
  FILE *js_file = fopen(json_path.c_str(), "w+");
  if(NULL == js_file)
  {
    std::cout << "ERROR: invalid output file path!!!" << endl;
    return false;
  }

  char writeBuffer[65536];
  FileWriteStream os(js_file, writeBuffer, sizeof(writeBuffer));

  PrettyWriter<FileWriteStream> p_writer(os);
  document.Accept(p_writer);

  std::fclose(js_file);
  std::cout << "path file saved successfully!" << std::endl;
  return true;
}

bool ProcAnalyzer::IfPointInVector(point p)
{
  for (int i = 0; i <exist_point_.size(); i++)
  {
    if ((exist_point_[i] - p).length() < GEO_EPS)
      return true;
  }
  return false;
}

bool ProcAnalyzer::IfCoOrientation(GeoV3 a, vector<GeoV3> &b)
{
  for (int i = 0; i < b.size(); i++)
  {
    if (angle(a, b[i]) < (F_PI / 2))
    {
      return true;
    }
  }
  return false;
}

void ProcAnalyzer::CheckProcess(Process &a)
{
  GeoV3 t = a.end_ - a.start_;

  t.normalize();
  vector<GeoV3> temp_normal;

  if (!IfCoOrientation(t, a.normal_))
  {
    point temp = a.end_;
    a.end_ = a.start_;
    a.start_ = temp;
    for (int i = 0; i < a.normal_.size(); i++)
    {
      if (angle(t, a.normal_[i]) < (F_PI / 2))
      {
        continue;
      }
      else
      {
        temp_normal.push_back(a.normal_[i]);
      }
    }
  }
  else
  {
    for (int i = 0; i < a.normal_.size(); i++)
    {
      if (angle(t, a.normal_[i]) < (F_PI / 2))
      {
        temp_normal.push_back(a.normal_[i]);
      }
    }
  }

  a.normal_ = temp_normal;
}

void ProcAnalyzer::Filter(Process &a)
{
  GeoV3 t = a.end_ - a.start_;
  t.normalize();
  vector <GeoV3> temp;

  if (IfCoOrientation(t, a.normal_))
  {
    for (int i = 0; i < a.normal_.size(); i++)
    {
      if (angle(t, a.normal_[i]) < MaxEdgeAngle_)
      {
        temp.push_back(a.normal_[i]);
      }
    }
  }

  if (temp.size())
  {
    a.normal_ = temp;
  }
}