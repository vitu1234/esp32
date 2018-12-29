function(create_mock mock_name header_abs_path)
  get_filename_component(header_folder ${header_abs_path} DIRECTORY)
  file(MAKE_DIRECTORY ${header_folder}/mocks)
  add_custom_command (
    OUTPUT ${header_folder}/mocks/${mock_name}.c
    COMMAND ruby
            ${CMAKE_SOURCE_DIR}/vendor/cmock/lib/cmock.rb
            -o${CMAKE_SOURCE_DIR}/cmake_common/project.yml
            ${header_abs_path}
    WORKING_DIRECTORY ${header_folder}
    DEPENDS ${header_abs_path})
  
  add_library(${mock_name} ${header_folder}/mocks/${mock_name}.c)
  target_include_directories(${mock_name} PUBLIC  ${header_folder})
  target_include_directories(${mock_name} PUBLIC  ${header_folder}/mocks)
  target_link_libraries(${mock_name} unity)
  target_link_libraries(${mock_name} cmock)
endfunction()
