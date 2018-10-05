# ----------- VDMFusionTest

add_executable(VDMFusionTest
    src/VDMFusionTest.cpp
)

target_link_libraries(VDMFusionTest
    ROAMestimation
)
# ----------- IMUGPSFusionTest

add_executable(IMUGPSFusionTest
    src/IMUGPSFusionTest.cpp
)

target_link_libraries(IMUGPSFusionTest
    ROAMestimation
)


# ----------- IMUHandlerFusionTest

add_executable(IMUHandlerFusionTest
    src/IMUHandlerFusionTest.cpp
)

target_link_libraries(IMUHandlerFusionTest
    ROAMestimation ROAMimu 
)

# ----------- CameraIMUTest

add_executable(CameraIMUTest
    src/CameraIMUTest.cpp
)

target_link_libraries(CameraIMUTest
    ROAMestimation ROAMimu ROAMvision
)

# ----------- TestPoseGetters

add_executable(TestPoseGetters
    src/TestPoseGetters.cpp
)

target_link_libraries(TestPoseGetters
    ROAMestimation
)

# ----------- TestInterpolatingEdge

add_executable(TestInterpolatingEdge
    src/TestSE3InterpolationEdge.cpp
)

target_link_libraries(TestInterpolatingEdge
    ROAMestimation
)

