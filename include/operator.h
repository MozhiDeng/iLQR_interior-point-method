#include "common.h"
VecOfVecXd operator+(const VecOfVecXd& v1, const VecOfVecXd& v2) {
  VecOfVecXd result = v1;
  for (int i=0; i<v1.size(); i++) {
    for (int j=0; j<v1[0].size(); j++) {
        result[i][j] = v1[i][j] + v2[i][j];
    }
  }
  return result;
}
