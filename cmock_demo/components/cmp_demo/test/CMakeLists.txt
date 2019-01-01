#set(COMPONENT_SRCDIRS "." "../include/mocks/")
set(COMPONENT_SRCS "test_cmp_demo.c" "../include/mocks/mock_dep_demo.c")
# set(COMPONENT_SRCS "test_cmp_demo.c")  # TODO regretably this does not work :(
set(COMPONENT_ADD_INCLUDEDIRS "." "../include")

set(COMPONENT_REQUIRES unity cmock cmp_demo)

register_component()

get_filename_component(header_abs_path ../include/dep_demo.h REALPATH )
create_mock(mock_dep_demo ${header_abs_path})
