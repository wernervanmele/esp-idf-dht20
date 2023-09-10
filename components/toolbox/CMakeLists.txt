FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/components/toolbox/**)
idf_component_register(SRCS "${app_sources}"
                    INCLUDE_DIRS "include"
                    REQUIRES "driver" "esp_timer"
                    )
set_source_files_properties("${app_sources}"
                    PROPERTIES COMPILE_FLAGS
                    -Wall -Wextra -Werror
)