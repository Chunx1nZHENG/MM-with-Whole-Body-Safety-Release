// #pragma once

// #include "Eigen/Dense"

// using namespace Eigen;


// //build a struct to store the SDP problem parameters
// typedef struct SDPproblem
// {
//     struct input
//     {
//         std::string envname;
//         MatrixXd A;
//         VectorXd b;
//         MatrixXd F;
//         VectorXd g;
//         VectorXd q;
//         VectorXd c;
//     }input;
//     struct output
//     {
//         double alpha;
//         VectorXd gradient;
//     }output;
// }SDPproblem;

// typedef struct SDPproblemsArray
// {   
//     std::vector<SDPproblem> SDPproblems(7);
// }SDPproblemsArray;



// void singleSDP(SDPproblem &SDPproblem);
// void singleSDPwithoutGradient(SDPproblem &SDPproblem);
// void palSolveSDP(SDPproblemsArray &SDPproblemsArray);
// void palSolveSDPwithoutGradient(SDPproblemsArray &SDPproblemsArray);