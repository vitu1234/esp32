    set(COMPONENT_SRCDIRS ".")
    set(COMPONENT_ADD_INCLUDEDIRS "." "../include")

    set(COMPONENT_REQUIRES unity)

    get_filename_component(header_abs_path include/dep_demo.h REALPATH )
    create_mock(mock_dep_init ${header_abs_path})

    register_component()
