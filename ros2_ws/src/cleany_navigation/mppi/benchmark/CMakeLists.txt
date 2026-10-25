find_package(benchmark REQUIRED)

set(BENCHMARK_NAMES
  optimizer_benchmark
  controller_benchmark
)

foreach(name IN LISTS BENCHMARK_NAMES)
  add_executable(${name}
    ${name}.cpp
  )
  ament_target_dependencies(${name}
    ${mppi_dependencies}
  )
  target_link_libraries(${name}
    cleany_mppi_controller cleany_mppi_critics benchmark
  )

target_include_directories(${name} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../test/utils
)
endforeach()
