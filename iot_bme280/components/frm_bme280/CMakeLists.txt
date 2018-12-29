set(COMPONENT_SRCS "src/frm_bme280_sensor.c" "driver/bme280.c")
set(COMPONENT_ADD_INCLUDEDIRS "include")
set(COMPONENT_PRIV_INCLUDEDIRS "driver/")

#set(COMPONENT_REQUIRES "frm_config")

register_component()
