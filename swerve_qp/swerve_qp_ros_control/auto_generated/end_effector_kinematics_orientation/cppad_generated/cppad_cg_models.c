void cppad_cg_models(char const *const** names,
                     int* count) {
   static const char* const models[] = {
      "end_effector_kinematics_orientation"};
   *names = models;
   *count = 1;
}

