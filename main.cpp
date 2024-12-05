#include "PointCloudPLY.h"


int main(int argc, char** argv) {
  size_t thinner = 1;
  if(argc == 2)
  {
    thinner = std::atoi(argv[1]);
    if(!thinner) thinner = 1;
  }
  if(thinner != 1)
    std::cout << " thinner = " << thinner << std::endl; 
  
  PointCloudPLY editor{thinner};
  
  return 0;
}