set(CMAKE_AUTOMOC TRUE)

include_directories(
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5DBus_INCLUDE_DIRS}
)

link_directories(
  ${Qt5Core_LIBRARY_DIRS}
  ${Qt5DBus_LIBRARY_DIRS}
)

#qt5_wrap_cpp(STATEFS_QT_SRC ${STATEFS_QT_HEADERS})

add_library(statefs-providers-qt5
  SHARED
  ns.cpp
  ${STATEFS_QT_SRC}
)

target_link_libraries(statefs-providers-qt5
  ${Qt5Core_LIBRARIES}
  ${Qt5DBus_LIBRARIES}
  ${STATEFS_LIBRARIES}
  ${STATEFS_QT5_LIBRARIES}
)

configure_file(statefs-providers-qt5.pc.in statefs-providers-qt5.pc @ONLY)
  
install(TARGETS statefs-providers-qt5 DESTINATION ${DST_LIB})
install(FILES statefs-providers-qt5.pc DESTINATION ${DST_LIB}/pkgconfig)
