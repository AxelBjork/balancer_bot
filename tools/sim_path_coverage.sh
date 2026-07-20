#!/bin/sh
set -eu

ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
BUILD_DIR="${ROOT_DIR}/build-cov"
JOBS="${JOBS:-8}"
GCOV_TOOL="${GCOV_TOOL:-/usr/local/gcc-16.1.0/bin/gcov}"
HTML_DIR="${BUILD_DIR}/lcov-html-sim-path"

cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" \
  -DBUILD_TESTS=ON \
  -DBUILD_REFLECTION_ARTIFACTS=ON \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS=--coverage \
  -DCMAKE_C_FLAGS=--coverage \
  -DCMAKE_EXE_LINKER_FLAGS=--coverage \
  -DCMAKE_SHARED_LINKER_FLAGS=--coverage
cmake --build "${BUILD_DIR}" -j"${JOBS}"

find "${BUILD_DIR}" -name '*.gcda' -delete
find "${BUILD_DIR}" -name '*.gcov' -delete

SIL_APP="${BUILD_DIR}/sil_app" \
BALANCER_SIM_BIN="${BUILD_DIR}/balancer_simulator" \
python3 -m pytest "${ROOT_DIR}/tests/python/test_sim_scenarios.py" \
  -k test_realistic_profile_stability_scenarios -q

lcov --gcov-tool "${GCOV_TOOL}" --capture \
  --directory "${BUILD_DIR}/CMakeFiles/balancer_common.dir" \
  --base-directory "${ROOT_DIR}" \
  --output-file "${BUILD_DIR}/scenario_common.info" \
  --no-external
lcov --gcov-tool "${GCOV_TOOL}" --capture \
  --directory "${BUILD_DIR}/CMakeFiles/ipc_hub.dir" \
  --base-directory "${ROOT_DIR}" \
  --output-file "${BUILD_DIR}/scenario_ipc.info" \
  --no-external
lcov --gcov-tool "${GCOV_TOOL}" --capture \
  --directory "${BUILD_DIR}/CMakeFiles/balancer_sim_support.dir" \
  --base-directory "${ROOT_DIR}" \
  --output-file "${BUILD_DIR}/scenario_sim_support.info" \
  --no-external
lcov --gcov-tool "${GCOV_TOOL}" --capture \
  --directory "${BUILD_DIR}/CMakeFiles/balancer_simulator.dir" \
  --base-directory "${ROOT_DIR}" \
  --output-file "${BUILD_DIR}/scenario_simulator.info" \
  --no-external
lcov --gcov-tool "${GCOV_TOOL}" --capture \
  --directory "${BUILD_DIR}/CMakeFiles/px4_ratecontrol.dir" \
  --base-directory "${ROOT_DIR}" \
  --output-file "${BUILD_DIR}/scenario_px4.info" \
  --no-external

lcov --add-tracefile "${BUILD_DIR}/scenario_common.info" \
  --add-tracefile "${BUILD_DIR}/scenario_ipc.info" \
  --add-tracefile "${BUILD_DIR}/scenario_sim_support.info" \
  --add-tracefile "${BUILD_DIR}/scenario_simulator.info" \
  --add-tracefile "${BUILD_DIR}/scenario_px4.info" \
  --output-file "${BUILD_DIR}/scenario_merged.info"

lcov --remove "${BUILD_DIR}/scenario_merged.info" \
  '*/_deps/*' \
  '/usr/*' \
  '*/tests/*' \
  '*/tests/*' \
  --output-file "${BUILD_DIR}/scenario_src.info"

genhtml "${BUILD_DIR}/scenario_src.info" --output-directory "${HTML_DIR}"
lcov --summary "${BUILD_DIR}/scenario_src.info"

echo "Coverage HTML: ${HTML_DIR}/index.html"
