/*!
 * @file    ClangToolsExample.cpp
 * @author  Gabriel Hottiger
 * @date    Feb, 2018
 * @version 0.0
 *
 */

#include "ClangToolsExample.hpp"

namespace allWrong {

  int ClangToolsExample::horribleFunction(const int& input) {
    const_cast<int&>(input) = 3;
  }

}
