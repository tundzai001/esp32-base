#ifndef DATA_STRUCTS_H
#define DATA_STRUCTS_H

#include <WString.h>

struct gga_data_struct
{
  double lat;
  double lon;
  String rtk_status;
  String satellites;
};
using gga_data_t = struct gga_data_struct;

#endif