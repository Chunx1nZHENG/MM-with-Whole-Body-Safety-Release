void end_effector_kinematics_velocity_jacobian_sparsity(unsigned long const** row,
                                                        unsigned long const** col,
                                                        unsigned long* nnz) {
   static unsigned long const rows[54] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2};
   static unsigned long const cols[54] = {3,4,5,6,15,16,17,18,19,20,21,22,23,32,33,34,35,36,3,4,5,6,15,16,17,18,19,20,21,22,23,32,33,34,35,36,3,4,5,6,15,16,17,18,19,20,21,22,23,32,33,34,35,36};
   *row = rows;
   *col = cols;
   *nnz = 54;
}
