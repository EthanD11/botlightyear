/*!
 * \file utils_gr2.cc
 * \brief File description
 */
#include "utils.h"


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
