# order id
int32 order_id

# geometry file save dir
string file_path

# pick - object geometry file name (not including file path)
string pick_element_geometry_file_name

# place - object geometry file name (not including file path)
string place_element_geometry_file_name

# pick support surfaces file name (not including file path)
string pick_support_surface_file_name

# place support surfaces file name (not including file path)
string place_support_surface_file_name

# pick contact neighboring object file names (ids)
int32[] pick_contact_ngh_ids

# place contact neighboring object file names (ids)
int32[] place_contact_ngh_ids

framefab_msgs/Grasp[] grasps