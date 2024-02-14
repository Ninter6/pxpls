aux_source_directory(2D src2d)

add_library(pxpls STATIC ${src2d})
target_include_directories(pxpls PUBLIC math)
target_compile_features(pxpls PUBLIC cxx_std_17)
target_compile_options(pxpls PUBLIC -Wno-changes-meaning)
