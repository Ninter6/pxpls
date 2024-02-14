add_executable(collision collision.cpp)
target_link_libraries(collision pxpls raylib)

add_executable(spring spring.cpp)
target_link_libraries(spring pxpls raylib)
target_compile_features(spring PUBLIC cxx_std_20)
