void nominal_arm_cost_1_jacobian_sparsity(unsigned long const** row,
                                          unsigned long const** col,
                                          unsigned long* nnz) {
   static unsigned long const rows[1] = {0};
   static unsigned long const cols[1] = {17};
   *row = rows;
   *col = cols;
   *nnz = 1;
}
