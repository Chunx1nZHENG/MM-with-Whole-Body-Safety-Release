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

void swerve_dynamics_flow_map_sparse_jacobian(double const *const * in,
                                              double*const * out,
                                              struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* jac = out[0];

   // auxiliary variables
   double v[8];

   v[0] = x[23] + x[23];
   jac[0] = x[5] * v[0];
   v[1] = 0 - x[22];
   v[1] = v[1] + v[1];
   v[2] = x[4] * x[23] - x[5] * x[22];
   jac[15] = v[2] + v[2];
   jac[1] = x[5] * v[1] + jac[15];
   v[2] = - x[23];
   v[2] = v[2] + v[2];
   v[3] = x[22] + x[22];
   v[4] = x[6] * x[22];
   jac[9] = v[4] + v[4];
   jac[2] = x[7] * v[2] - (x[6] * v[3] + jac[9]);
   v[4] = 0 - x[6] * x[23];
   jac[3] = v[4] + v[4];
   v[4] = 0 - x[5];
   v[4] = v[4] + v[4];
   v[5] = x[6] + x[6];
   jac[4] = 1 + x[5] * v[4] - x[6] * v[5];
   v[6] = - x[6];
   v[6] = v[6] + v[6];
   v[7] = x[4] + x[4];
   jac[5] = x[7] * v[6] + x[5] * v[7];
   jac[6] = 0 - (x[4] * v[0] + jac[15]);
   jac[7] = 0 - x[4] * v[1];
   jac[8] = x[7] * v[3] + x[6] * v[2] + jac[3];
   jac[10] = x[7] * v[5] - x[4] * v[4];
   jac[11] = 1 + x[6] * v[6] - x[4] * v[7];
   jac[12] = jac[9] + x[7] * v[0];
   jac[13] = x[7] * v[1] - jac[3];
   jac[14] = x[4] * v[3] - x[5] * v[2];
   jac[16] = x[4] * v[5] + x[7] * v[4];
   jac[17] = x[7] * v[7] - x[5] * v[6];
   jac[18] = x[24] / 2.;
   jac[19] = x[5] * 0.5;
   jac[20] = - jac[18];
   jac[21] = - x[4] * 0.5;
   jac[23] = x[7] * 0.5;
   jac[24] = - jac[18];
   jac[25] = - x[6] * 0.5;
   // variable duplicates: 1
   jac[22] = jac[18];
   // dependent variables without operations
   jac[26] = 1;
   jac[27] = 1;
   jac[28] = 1;
   jac[29] = 1;
   jac[30] = 1;
   jac[31] = 1;
   jac[32] = 1;
   jac[33] = 1;
   jac[34] = 1;
   jac[35] = 1;
   jac[36] = 1;
   jac[37] = 1;
   jac[38] = 1;
   jac[39] = 1;
}

