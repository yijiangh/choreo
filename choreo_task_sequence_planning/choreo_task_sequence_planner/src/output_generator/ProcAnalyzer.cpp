#include <ros/console.h>

#include "choreo_task_sequence_planner/output_generator/ProcAnalyzer.h"

// for task sequence planning result
#include "choreo_task_sequence_planner/sequence_analyzers/SeqAnalyzer.h"

#include <choreo_rapidjson/include/rapidjson/document.h>
#include <choreo_rapidjson/include/rapidjson/prettywriter.h>
#include <choreo_rapidjson/include/rapidjson/filewritestream.h>

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
}

bool ProcAnalyzer::ProcPrint()
{
  WireFrame *ptr_frame = ptr_seqanalyzer_->ptr_frame_;
  DualGraph *ptr_dualgraph = ptr_seqanalyzer_->ptr_dualgraph_;

//  QuadricCollision collision_checker = QuadricCollision();

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
  for (int i = 0; i < planning_result.size(); i++)
  {
    Process temp = process_list_[i];
    WF_edge* e = planning_result[i].e_;

    int orig_j = e->ID();

    // st node index
    int uj = ptr_frame->GetEndu(orig_j);
    temp.start_ = ptr_frame->GetVert(uj)->Position();

    // end node index
    int vj = ptr_frame->GetEndv(orig_j);
    temp.end_ = ptr_frame->GetVert(vj)->Position();

    if (e->isPillar())
    {
      temp.fan_state_ = true;
    }
    else
    {
      // non-pillar element
      // init
      const bool start_node_exist = IfPointInVector(temp.start_);
      const bool end_node_exist = IfPointInVector(temp.end_);

      // sanity check: at least one of the nodes should exist
      assert(start_node_exist || end_node_exist);

//      // TODO: temp, to enable output for chosen layers' result
//      if(!start_node_exist && !end_node_exist)
//      {
//        temp.fan_state_ = true;
//        temp.start_ = start_node;
//        temp.end_ = end_node;
//      }

      // XOR - only one of them exist, "create type"
      if (start_node_exist != end_node_exist)
      {
        temp.fan_state_ = true;
      }
      else
      {
        // AND - both of them exist, "connect type"
        temp.fan_state_ = false;
      }
    }

    // pillar shouldn't have been printed yet
    if (!IfPointInVector(temp.start_))
    {
      exist_point_.push_back(temp.start_);
    }

    if (!IfPointInVector(temp.end_))
    {
      exist_point_.push_back(temp.end_);
    }

    process_list_[i] = temp;
  } // end loop for all elements (planning_result)

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