void nominal_steer_cost_info(const char** baseName,
                             unsigned long* m,
                             unsigned long* n,
                             unsigned int* indCount,
                             unsigned int* depCount) {
   *baseName = "double  d";
   *m = 4;
   *n = 26;
   *depCount = 1; // number of dependent array variables
   *indCount = 1; // number of independent array variables
}

