# only support
# "spatial_extrusion"
# "picknplace"
# for now

string assembly_type

# element number
int32 element_number

geometry_msgs/Point pick_base_center_point

geometry_msgs/Point place_base_center_point

# geometry file save dir
# note: last character includes separator
string file_path

string[] pick_support_surface_file_names

string[] place_support_surface_file_names

framefab_msgs/SequencedElement[] sequenced_elements