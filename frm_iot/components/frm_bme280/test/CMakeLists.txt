set(COMPONENT_SRCDIRS ".")
set(COMPONENT_ADD_INCLUDEDIRS "." "../include")
set(COMPONENT_PRIV_INCLUDEDIRS "../driver/" "$ENV{IDF_PATH}/components/driver/include/driver/")

set(COMPONENT_REQUIRES unity cmock frm_bme280)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DBME280_FLOAT_ENABLE")

register_component()

#  CREATING THE MOCK  #
get_filename_component(header_abs_path ../driver/bme280.h REALPATH)
create_mock(mock_bme280 ${header_abs_path})

get_filename_component(header_abs_path $ENV{IDF_PATH}/components/driver/include/driver/i2c.h REALPATH)
create_mock(mock_i2c ${header_abs_path})
