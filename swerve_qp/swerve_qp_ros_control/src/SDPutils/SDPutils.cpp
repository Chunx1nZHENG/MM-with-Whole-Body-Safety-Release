// #include "./SDPutils/SDPutils.hpp"
// #include "./SDPsolver/SDPsolver.hpp"
// #include <thread>

// using namespace Eigen;



// void singleSDP(SDPproblem &SDPproblem)
// {
//     //build a SDPsolver object
//     SDPsolver sdp;
//     //solve the SDP problem
//     SDPproblem.output.alpha = sdp.solveSDP(SDPproblem.input.envname, SDPproblem.input.A, SDPproblem.input.b, SDPproblem.input.F, SDPproblem.input.g, SDPproblem.input.q, SDPproblem.input.c);
//     SDPproblem.output.gradient = sdp.solveSDPGradient();
// }

// void singleSDPwithoutGradient(SDPproblem &SDPproblem)
// {
//     //build a SDPsolver object
//     SDPsolver sdp;
//     //solve the SDP problem
//     SDPproblem.output.alpha = sdp.solveSDP(SDPproblem.input.envname, SDPproblem.input.A, SDPproblem.input.b, SDPproblem.input.F, SDPproblem.input.g, SDPproblem.input.q, SDPproblem.input.c);
// }

// //input is the 7 SDP problem parameters
// void palSolveSDP(SDPproblemsArray &SDPproblemsArray)
// {   
//     //check the number of SDP problems
//     if(SDPproblemsArray.SDPproblems.size() != 7)
//     {
//         std::cout << "The number of SDP problems is not 7" << std::endl;
//         return;
//     }
//     //build 7 thread to solve the SDP problem
//     for (int i = 0; i < SDPproblemsArray.SDPproblems.size(); i++)
//     {
//         std::thread t(singleSDP, std::ref(SDPproblemsArray.SDPproblems[i]));
//         t.join();
//     }
// }


// void palSolveSDPwithoutGradient(SDPproblemsArray &SDPproblemsArray)
// {   
//     //check the number of SDP problems
//     if(SDPproblemsArray.SDPproblems.size() != 7)
//     {
//         std::cout << "The number of SDP problems is not 7" << std::endl;
//         return;
//     }
//     //build 7 thread to solve the SDP problem
//     for (int i = 0; i < SDPproblemsArray.SDPproblems.size(); i++)
//     {
//         std::thread t(singleSDPwithoutGradient, std::ref(SDPproblemsArray.SDPproblems[i]));
//         t.join();
//     }
// }
