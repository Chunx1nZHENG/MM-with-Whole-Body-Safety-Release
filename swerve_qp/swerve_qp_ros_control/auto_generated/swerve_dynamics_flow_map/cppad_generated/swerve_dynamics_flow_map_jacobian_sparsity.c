void swerve_dynamics_flow_map_jacobian_sparsity(unsigned long const** row,
                                                unsigned long const** col,
                                                unsigned long* nnz) {
   static unsigned long const rows[40] = {0,0,0,0,0,0,1,1,1,1,1,1,2,2,2,2,2,2,3,3,4,4,5,5,6,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};
   static unsigned long const cols[40] = {4,5,6,7,22,23,4,5,6,7,22,23,4,5,6,7,22,23,5,24,4,24,7,24,6,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38};
   *row = rows;
   *col = cols;
   *nnz = 40;
}
