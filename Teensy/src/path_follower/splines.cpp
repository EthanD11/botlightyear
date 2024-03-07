#include "splines.h" // adapt it with your headers

void init_spline_set(SplineSet *splines, int n) {
    double *mem = (double *) malloc(sizeof(double)*(6*n+3));
    splines->n = n;
    splines->a = mem;
    splines->b = splines->a + n + 1;
    splines->c = splines->b + n;
    splines->d = splines->c + n;
    splines->q = splines->d + n;
    splines->x = splines->q + n + 1;
}

void free_spline_set(SplineSet *splines) {
    free(splines->a);
}

// Interpolates using natural cubic splines a list of points
// https://en.wikipedia.org/w/index.php?title=Spline_%28mathematics%29&oldid=288288033#Algorithm_for_computing_natural_cubic_splines
SplineSet* compute_splines(double *q, double *x, int size) {
    int n = size - 1;
    SplineSet* splines = (SplineSet *) malloc(sizeof(SplineSet));
    init_spline_set(splines, n);

    double *a = splines->a;
    double *b = splines->b;
    double *c = splines->c;
    double *d = splines->d;

    memcpy(splines->q, q, (n+1)*sizeof(double));
    memcpy(splines->x, x, (n+1)*sizeof(double));

    double *mem = (double *) malloc((2*n+3*(n+1))*sizeof(double));
    double *h       = mem;
    double *alpha   = h     + n;
    double *l       = alpha + n;
    double *mu      = l     + n + 1;
    double *z       = mu    + n + 1;

    for (int i      = 0; i <= n; i++) {
        a[i] = x[i];
    }
    
    for (int i = 0; i < n; i++) {
        h[i] = q[i+1] - q[i];
    } 

    for (int i = 1; i < n; i++) {
        alpha[i] = (3/h[i])*(a[i+1] - a[i]) - (3/h[i-1])*(a[i]-a[i-1]);
    }

    l[0] = 1;
    mu[0] = 0;
    z[0] = 0; 

    for (int i = 1; i < n; i++) {
        l[i] = 2*(q[i+1] - q[i-1]) - h[i-1] * mu[i-1];
        mu[i] = h[i]/l[i];
        z[i] = (alpha[i] - h[i-1]*z[i-1])/l[i];
    }

    l[n] = 1;
    z[n] = 0;
    c[n] = 0;

    for (int j = n-1; j >= 0; j--) {
        c[j] = z[j] - mu[j]*c[j+1];
        b[j] = (a[j+1] - a[j]) / h[j]  - h[j]*(c[j+1] + 2*c[j])/3;
        d[j] = (c[j+1] - c[j])/ (3*h[j]);
    }

    free(mem);
    return splines;
}

double* interpolate_splines(SplineSet *splines, int m) {
    int n = splines->n;
    double *a, *b, *c, *d;
    a = splines->a; b = splines->b; c = splines->c; d = splines->d;
    double qi, dq;

    double *interpolation = (double *) malloc(sizeof(double)*n*m);
    for (int i = 0; i < n; i++) {
        // Loop throug each splines
        qi = splines->q[i];
        for (int j = 0; j < m; j++) {
            dq = (splines->q[i+1] - qi) * ((double) j) / ((double) m);
            interpolation[m*i + j] = evaluate_spline(a[i], b[i], c[i], d[i], dq);
        }
    }
    return interpolation;
}


// void test_splines() {
//     // Test splines
//     double q[4] = {0.0, 1.0, 2.0, 3.0};
//     double path_checkpoints[4] = {0.0, 1.0, 3.0, -1.0};
//     int size = 4; // 4 points, 3 splines
//     SplineSet *splines = compute_splines(q, path_checkpoints, size);

//     int m = 40;
//     double *path;
//     path = interpolate_splines(splines, m);

//     store_vector((size-1)*m, path, "resultR/test_spline.csv");
//     free_spline_set(splines);
//     free(path);
// }
