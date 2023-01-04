add_library(Trajectory STATIC
        Trajectory.cpp
        )

target_include_directories(Trajectory PUBLIC
        ../lib/Eigen
        )
