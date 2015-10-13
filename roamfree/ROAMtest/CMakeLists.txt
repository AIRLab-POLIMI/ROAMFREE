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
