// Copyright 2026 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// HH_260811 - Emulate localized and C-locale `free` output for the memory monitor test.

#include <cstdlib>
#include <cstring>
#include <iostream>

int main()
{
  const char * lc_all = std::getenv("LC_ALL");
  if (lc_all != nullptr && std::strcmp(lc_all, "C") == 0) {
    std::cout << "               total        used        free      shared  buff/cache   available\n"
              << "Mem:          100000       10000       60000        1000       30000       90000\n"
              << "Swap:          10000           0       10000\n"
              << "Total:        110000       10000      100000\n";
  } else {
    std::cout << "                \xec\xb4\x9d\xea\xb3\x84         \xec\x82\xac\xec\x9a\xa9        \xec\x97\xac\xeb\xb6\x84      \xea\xb3\xb5\xec\x9c\xa0    \xeb\xb2\x84\xed\x8d\xbc/\xec\xba\x90\xec\x8b\x9c    \xea\xb0\x80\xec\x9a\xa9\n"
              << "\xeb\xa9\x94\xeb\xaa\xa8\xeb\xa6\xac:       100000       10000       60000        1000       30000       90000\n"
              << "\xec\x8a\xa4  \xec\x99\x91:        10000           0       10000\n"
              << "\xec\xb4\x9d  \xea\xb3\x84:       110000       10000      100000\n";
  }

  return 0;
}
