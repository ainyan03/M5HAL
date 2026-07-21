// SPDX-License-Identifier: MIT
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <vector>

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

int main(int argc, char** argv)
{
    for (int i = 1; i < argc; ++i) {
        std::ifstream input{argv[i], std::ios::binary};
        std::vector<uint8_t> bytes{std::istreambuf_iterator<char>{input}, std::istreambuf_iterator<char>{}};
        LLVMFuzzerTestOneInput(bytes.data(), bytes.size());
    }
    return 0;
}
