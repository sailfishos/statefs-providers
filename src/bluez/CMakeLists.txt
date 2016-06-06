set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC TRUE)

pkg_check_modules(KF5BLUEZQT KF5BluezQt REQUIRED)

include_directories(
  ${KF5BLUEZQT_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5DBus_INCLUDE_DIRS}
)

link_directories(
  ${KF5BLUEZQT_LIBRARY_DIRS}
  ${Qt5Core_LIBRARY_DIRS}
  ${Qt5DBus_LIBRARY_DIRS}
)

set_source_files_properties(
  PROPERTIES INCLUDE qdbusxml2cpp_common.h
)

add_library(provider-bluez SHARED
  ${SRC}
  provider_bluez.cpp
  )

target_link_libraries(provider-bluez
  statefs-providers-qt5
  ${CMAKE_THREAD_LIBS_INIT}
  ${STATEFS_LIBRARIES}
  ${QTAROUND_DBUS_LIBRARIES}
  ${KF5BLUEZQT_LIBRARIES}
  ${Qt5Core_LIBRARIES}
  ${Qt5DBus_LIBRARIES}
  )

install(TARGETS provider-bluez DESTINATION ${DST_LIB}/statefs)
