set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC TRUE)

find_package(Qt5Sensors)

if(Qt5Sensors_FOUND)
  add_library(provider-sensors SHARED
    provider_sensors.cpp
    )
  target_link_libraries(provider-sensors
    statefs-providers-qt5
    )
  qt5_use_modules(provider-sensors Core Sensors)
  install(TARGETS provider-sensors DESTINATION ${DST_LIB}/statefs)
endif()
