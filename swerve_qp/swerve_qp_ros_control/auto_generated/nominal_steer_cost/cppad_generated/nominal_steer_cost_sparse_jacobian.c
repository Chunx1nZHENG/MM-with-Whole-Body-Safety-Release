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

void nominal_steer_cost_sparse_jacobian(double const *const * in,
                                        double*const * out,
                                        struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* jac = out[0];

   // auxiliary variables

   if( x[22] < 0.5 ) {
      jac[0] = 0;
   } else {
      jac[0] = 1;
   }
   if( x[23] < 0.5 ) {
      jac[1] = 0;
   } else {
      jac[1] = 1;
   }
   if( x[24] < 0.5 ) {
      jac[2] = 0;
   } else {
      jac[2] = 1;
   }
   if( x[25] < 0.5 ) {
      jac[3] = 0;
   } else {
      jac[3] = 1;
   }
}

