/*!
 * @file    ClangToolsExample.hpp
 * @author  Gabriel Hottiger
 * @date    Feb, 2018
 * @version 0.0
 *
 */

#pragma once

namespace allWrong {

class ClangToolsExample {

  // Not default constructor
  ClangToolsExample() { }

  // Horrible function
  int horribleFunction(const int& input);

  // Wrong function name
  int NotCamelBack() {
    return NOTCAMELBACK_;
  }

private:
  int NOTCAMELBACK_;
  int missingSuffix;
};

}
