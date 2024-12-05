#include <math.h>
#include <stdio.h>

typedef struct Array {
    void* data;
    unsigned long size;
    int sparse;
    const unsigned long* idx;
    unsigned long nnz;
} Array;

struct LangCAtomicFun {
    void* libModel;
    int (*forward)(void* libModel,
                   int atomicIndex,
                   int q,
                   int p,
                   const Array tx[],
                   Array* ty);
    int (*reverse)(void* libModel,
                   int atomicIndex,
                   int p,
                   const Array tx[],
                   Array* px,
                   const Array py[]);
};

void swerve_dynamics_flow_map_forward_zero(double const *const * in,
                                           double*const * out,
                                           struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* y = out[0];

   // auxiliary variables
   double v[3];

   v[0] = 0 - x[6] * x[23];
   v[0] = v[0] + v[0];
   v[1] = x[4] * x[23] - x[5] * x[22];
   v[1] = v[1] + v[1];
   v[2] = x[6] * x[22];
   v[2] = v[2] + v[2];
   y[0] = x[7] * v[0] + x[22] + x[5] * v[1] - x[6] * v[2];
   y[1] = x[7] * v[2] + x[23] + x[6] * v[0] - x[4] * v[1];
   y[2] = x[4] * v[2] + x[7] * v[1] - x[5] * v[0];
   v[2] = x[24] / 2.;
   y[3] = x[5] * v[2];
   y[4] = 0 - x[4] * v[2];
   y[5] = x[7] * v[2];
   y[6] = 0 - x[6] * v[2];
   // dependent variables without operations
   y[7] = x[25];
   y[8] = x[26];
   y[9] = x[27];
   y[10] = x[28];
   y[11] = x[29];
   y[12] = x[30];
   y[13] = x[31];
   y[14] = x[32];
   y[15] = x[33];
   y[16] = x[34];
   y[17] = x[35];
   y[18] = x[36];
   y[19] = x[37];
   y[20] = x[38];
}

