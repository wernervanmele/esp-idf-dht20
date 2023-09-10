FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/components/dht20/**)
idf_component_register(SRCS "firfilter.c" "dht20.c" "${app_sources}"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES "toolbox"
                    )
set_source_files_properties("${app_sources}"
                    PROPERTIES COMPILE_FLAGS
                    -Wall -Wextra -Werror
)
