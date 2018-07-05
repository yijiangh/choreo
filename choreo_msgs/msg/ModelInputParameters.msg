# model position reference point (on building plate)
float64 ref_pt_x
float64 ref_pt_y
float64 ref_pt_z

# for collision checking & visualization
float64 element_diameter
float64 shrink_length

# Enumeration supporting model unit
int32 MILLIMETER=0
int32 CENTIMETER=1
int32 INCH=2
int32 FOOT=3
int32 METER=4

# model unit
int32 unit_type

# model file name (whole path: file path + file name)
string file_name

float64 clt_rrt_unit_process_timeout

float64 clt_rrt_timeout
