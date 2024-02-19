aux_source_directory(2D src2d)
aux_source_directory(3D src3d)

add_library(pxpls STATIC ${src2d} ${src3d})
target_include_directories(pxpls PUBLIC math 2D 3D)
target_compile_features(pxpls PUBLIC cxx_std_17)
# target_compile_options(pxpls PUBLIC -Wno-changes-meaning)
