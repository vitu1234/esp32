set(COMPONENT_SRCS "src/frm_bme280_sensor.c" "driver/bme280.c")
set(COMPONENT_ADD_INCLUDEDIRS "include")
set(COMPONENT_PRIV_INCLUDEDIRS "driver/")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DBME280_FLOAT_ENABLE")

register_component()
