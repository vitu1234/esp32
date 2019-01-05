function(create_mock mock_name header_abs_path)
  file(MAKE_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/mocks)
  add_custom_command (
    OUTPUT ${CMAKE_CURRENT_LIST_DIR}/mocks/${mock_name}.c
    COMMAND ruby
            ${CMAKE_SOURCE_DIR}/../components/cmock/lib/cmock.rb
            --plugins="ignore_arg$<SEMICOLON>expect_any_args" --mock_prefix=mock_
            ${header_abs_path}
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    DEPENDS ${header_abs_path})
  
  # create mock dependency
  #add_custom_target(${mock_name} DEPENDS ${header_abs_path})
  #add_dependencies(${COMPONENT_TARGET} ${mock_name})

  # add mock source
  target_sources(${COMPONENT_TARGET} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/mocks/${mock_name}.c)

  # add to cleanup path!
  set_property(DIRECTORY "${COMPONENT_PATH}" APPEND PROPERTY
    ADDITIONAL_MAKE_CLEAN_FILES 
    ${CMAKE_CURRENT_LIST_DIR}/mocks/${mock_name}.h 
    ${CMAKE_CURRENT_LIST_DIR}/mocks/${mock_name}.c)
endfunction()
