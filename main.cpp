#include "PointCloudPLY.h"


int main(int argc, char** argv) {

  Mode mode = POINTS;
  std::string fn;
  if(argc == 3)
  {
    std::string smode = argv[1];
    if(smode == "points")
      mode = POINTS;
    else if(smode == "normals")
      mode = NORMALS;
    else if(smode == "intensities")
      mode = INTENSITIES;
    else
      mode = POINTS;

    std::string filename = argv[2];
    PointCloudPLY editor{mode, filename};

  }
  else
  {
    std::cout << " Expected ./PointCloudViewer [points | normals | intensities] [filename.las | filename.ply]  " << std::endl;
  }




  return 0;
}