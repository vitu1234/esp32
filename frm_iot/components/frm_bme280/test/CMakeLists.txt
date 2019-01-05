set(COMPONENT_SRCDIRS ".")
set(COMPONENT_ADD_INCLUDEDIRS ".")
set(COMPONENT_PRIV_INCLUDEDIRS "../driver/")

set(COMPONENT_REQUIRES unity frm_config frm_bme280)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DBME280_FLOAT_ENABLE")

register_component()

#  CREATING THE MOCK  #
get_filename_component(header_abs_path ../driver/bme280.h REALPATH)
create_mock(mock_bme280 ${header_abs_path})
