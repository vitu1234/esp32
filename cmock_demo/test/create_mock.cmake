function(create_mock mock_name header_abs_path)
  get_filename_component(header_folder ${header_abs_path} DIRECTORY)
  file(MAKE_DIRECTORY ${header_folder}/mocks)
  add_custom_command (
    OUTPUT ${header_folder}/mocks/${mock_name}.c
    COMMAND ruby
            ${CMAKE_SOURCE_DIR}/../components/cmock/lib/cmock.rb
            --mock_prefix=mock_
            ${header_abs_path}
    WORKING_DIRECTORY ${header_folder}
    DEPENDS ${header_abs_path})
  
  # create mock dependency
  #add_custom_target(${mock_name} DEPENDS ${header_abs_path})
  #add_dependencies(${COMPONENT_TARGET} ${mock_name})

  # add mock source
  target_sources(${COMPONENT_TARGET} PRIVATE "../include/mocks/mock_dep_demo.c")

  # add to cleanup path!
  set_property(DIRECTORY "${COMPONENT_PATH}" APPEND PROPERTY
    ADDITIONAL_MAKE_CLEAN_FILES 
    ${header_folder}/mocks/${mock_name}.h 
    ${header_folder}/mocks/${mock_name}.c)
endfunction()
