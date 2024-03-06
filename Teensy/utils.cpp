/*!
 * \file utils_gr2.cc
 * \brief File description
 */
#include "namespace_ctrl.h"
#include "utils_gr2.h"

NAMESPACE_INIT(ctrlGr2);

void store_vector(int n, double *x, char *filename) {
    FILE *fpt = fopen(filename, "w+");
    for (int i = 0; i < n; i++) {
        fprintf(fpt, "%10.16e\n", x[i]);
    }
    fclose(fpt);
}

void erase_file(char *filename) {
    FILE *fpt = fopen(filename, "w");
    fclose(fpt);
}

void append_to_file(char *filename, double data) {
    FILE *fpt = fopen(filename, "a");
    fprintf(fpt, "%10.16e\n", data);
    fclose(fpt);
}

inline int SAT(int x, int limit) {
  return std::clamp(x, -(limit), limit);
}  // Saturation function for integers

inline double SAT(double x, double limit) {
  return std::clamp(x, -(limit), limit);
}  // Saturation function for doubles

NAMESPACE_CLOSE();