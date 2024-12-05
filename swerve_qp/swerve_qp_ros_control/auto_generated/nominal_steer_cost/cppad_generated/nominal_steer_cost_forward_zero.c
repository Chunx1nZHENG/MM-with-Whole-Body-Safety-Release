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

void nominal_steer_cost_forward_zero(double const *const * in,
                                     double*const * out,
                                     struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* y = out[0];

   // auxiliary variables

   if( x[22] < 0.5 ) {
      y[0] = 0;
   } else {
      y[0] = x[8];
   }
   if( x[23] < 0.5 ) {
      y[1] = 0;
   } else {
      y[1] = x[10];
   }
   if( x[24] < 0.5 ) {
      y[2] = 0;
   } else {
      y[2] = x[12];
   }
   if( x[25] < 0.5 ) {
      y[3] = 0;
   } else {
      y[3] = x[14];
   }
}

