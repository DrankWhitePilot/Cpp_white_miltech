#include "ballistics.hpp"

#include <cstddef>
#include <exception>
#include <fstream>
#include <iostream>
#include <span>
#include <string>

namespace {
constexpr std::size_t kInputPathArgument = 1;
constexpr std::size_t kOutputPathArgument = 2;
constexpr int kFailureCode = 1;
constexpr int kSuccessCode = 0;

auto get_argument(std::span<char*> arguments, std::size_t index, const char* default_value) -> std::string
{
  if (arguments.size() <= index) {
    return default_value;
  }

  return arguments[index];
}
}  // namespace

auto main(int argc, char** argv) -> int
{
  const std::span<char*> kArguments(argv, static_cast<std::size_t>(argc));

  const std::string kInputPath = get_argument(kArguments, kInputPathArgument, "input.txt");
  const std::string kOutputPath = get_argument(kArguments, kOutputPathArgument, "output.txt");

  try {
    const BallisticsInput kInput = read_ballistics_input(kInputPath);
    const DropSolution kSolution = compute_drop_solution(kInput);

    std::ofstream output_file(kOutputPath);
    if (!output_file.is_open()) {
      std::cerr << "Cannot open output file: " << kOutputPath << '\n';
      return kFailureCode;
    }

    write_drop_solution(output_file, kSolution);
  }
  catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return kFailureCode;
  }

  return kSuccessCode;
}
