.PHONY: format lint test quality

format:
	clang-format -i \
		homework_06/include/ballistics.hpp \
		homework_06/src/ballistics.cpp \
		homework_06/src/main.cpp \
		homework_06/tests/ballistics_tests.cpp
	cmake-format -i homework_06/CMakeLists.txt

lint:
	clang-tidy -p build/debug homework_06/src/ballistics.cpp
	clang-tidy -p build/debug homework_06/src/main.cpp
	clang-tidy -p build/debug homework_06/tests/ballistics_tests.cpp

test:
	cmake --preset debug
	cmake --build --preset debug
	ctest --test-dir build/debug --output-on-failure

quality: format test lint
