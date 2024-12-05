void end_effector_kinematics_orientation_jacobian_sparsity(unsigned long const** row,
                                                           unsigned long const** col,
                                                           unsigned long* nnz) {
   static unsigned long const rows[30] = {0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2};
   static unsigned long const cols[30] = {3,4,5,6,15,16,17,18,19,20,3,4,5,6,15,16,17,18,19,20,3,4,5,6,15,16,17,18,19,20};
   *row = rows;
   *col = cols;
   *nnz = 30;
}
