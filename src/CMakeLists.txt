SET(SourceFiles
  temperature_solver.cpp
)

add_executable (temperature_solver ${SourceFiles})
target_link_libraries (temperature_solver owl-common owl-solver pthread)

INSTALL(TARGETS temperature_solver RUNTIME DESTINATION bin/owl)
