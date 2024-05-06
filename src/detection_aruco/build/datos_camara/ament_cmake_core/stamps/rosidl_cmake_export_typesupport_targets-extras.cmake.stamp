# generated from
# rosidl_cmake/cmake/template/rosidl_cmake_export_typesupport_targets.cmake.in

set(_exported_typesupport_targets
  "__rosidl_generator_c:datos_camara__rosidl_generator_c;__rosidl_typesupport_fastrtps_c:datos_camara__rosidl_typesupport_fastrtps_c;__rosidl_generator_cpp:datos_camara__rosidl_generator_cpp;__rosidl_typesupport_fastrtps_cpp:datos_camara__rosidl_typesupport_fastrtps_cpp;__rosidl_typesupport_introspection_c:datos_camara__rosidl_typesupport_introspection_c;__rosidl_typesupport_c:datos_camara__rosidl_typesupport_c;__rosidl_typesupport_introspection_cpp:datos_camara__rosidl_typesupport_introspection_cpp;__rosidl_typesupport_cpp:datos_camara__rosidl_typesupport_cpp;__rosidl_generator_py:datos_camara__rosidl_generator_py")

# populate datos_camara_TARGETS_<suffix>
if(NOT _exported_typesupport_targets STREQUAL "")
  # loop over typesupport targets
  foreach(_tuple ${_exported_typesupport_targets})
    string(REPLACE ":" ";" _tuple "${_tuple}")
    list(GET _tuple 0 _suffix)
    list(GET _tuple 1 _target)

    set(_target "datos_camara::${_target}")
    if(NOT TARGET "${_target}")
      # the exported target must exist
      message(WARNING "Package 'datos_camara' exports the typesupport target '${_target}' which doesn't exist")
    else()
      list(APPEND datos_camara_TARGETS${_suffix} "${_target}")
    endif()
  endforeach()
endif()
