idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_driver_gpio
                    PRIV_REQUIRES esp_driver_rmt
                    )

set(BOARD_TYPE "Starter")

add_custom_command(
    TARGET ${COMPONENT_LIB}
    PRE_BUILD
    COMMAND powershell -ExecutionPolicy Bypass -File D:/CloudStorage/OneDrive/Business/Etsy/SoftRepo/SerialGen/update_serial.ps1 -BoardType ${BOARD_TYPE} -OutputHeader ${CMAKE_SOURCE_DIR}/main/serial_auto.h
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)
