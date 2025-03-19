cmake_minimum_required(VERSION 3.0)
set(CMAKE_C_COMPILER_WORKS 1)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_LD arm-none-eabi-ld)

project(LDLC-Karmeliet-SECoreBin C ASM)

set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")

set(CMAKE_SYSTEM_NAME FreeRTOS)
set(CMAKE_SYSTEM_PROCESSOR cortex-m4)

set(BOOTLOADER_BINARY_NAME LDLC-Karmeliet-SECoreBin)

if(TOOLCHAIN_PATH)
    message("Use toolchain ${TOOLCHAIN_PATH}")
else(TOOLCHAIN_PATH)
    message(FATAL_ERROR "No toolchain provided")
endif(TOOLCHAIN_PATH)

if(VERSION)
    message("Use version ${VERSION}")
else(VERSION)
    message(FATAL_ERROR "No version provided")
endif(VERSION)

if(NOT DEFINED SOURCE_KEY)
    message(FATAL_ERROR "Source key was not provided")
endif(NOT DEFINED SOURCE_KEY)

set(BOOTLOADER_BINARY_NAME ${BOOTLOADER_BINARY_NAME}-${SOURCE_KEY})

if(INTERNAL_ENABLE_SECURE_BOOT)
    set(BOOTLOADER_BINARY_NAME ${BOOTLOADER_BINARY_NAME}-RDP2)
    add_compile_options(-DENABLE_SECURE_BOOT)
    message("Secure bootloader is built (RDP2 support)")
else(INTERNAL_ENABLE_SECURE_BOOT)
    set(BOOTLOADER_BINARY_NAME ${BOOTLOADER_BINARY_NAME}-UNSECURE)
    message("Unsecure bootloader is built")
endif(INTERNAL_ENABLE_SECURE_BOOT)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)


file(GLOB_RECURSE HEADERS "*.h")

# Add include directories depending on the header files path
FOREACH(header ${HEADERS})
 get_filename_component(dir ${header} DIRECTORY)
 include_directories(${dir})
ENDFOREACH()

include_directories(../../../../../Middlewares/ST/STM32_Secure_Engine/Core/)
include_directories(../../../../../Middlewares/ST/STM32_Cryptographic/Fw_Crypto/STM32L4/Inc/)
include_directories(../../../../../Middlewares/ST/STM32_Secure_Engine/Key/)
include_directories(../../../../../Drivers/STM32L4xx_HAL_Driver/Inc/)
include_directories(../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include/)
include_directories(../../../../../Drivers/CMSIS/Include/)
include_directories(../2_Images_SBSFU/SBSFU/App/)
include_directories(../2_Images_SBSFU/SBSFU/Target/)
include_directories(../Linker_Common/SW4STM32/)

add_compile_options(-mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16)
add_compile_options(-D__weak=__attribute__\(\(weak\)\) -D__packed=__attribute__\(\(__packed__\)\))
add_compile_options(-DUSE_HAL_DRIVER)
add_compile_options(-DSTM32L496xx)
add_compile_options(-DENABLE_IMAGE_STATE_HANDLING)
add_compile_options(-Os -g3 -Wall -fmessage-length=0)
add_compile_options(-fdata-sections -ffunction-sections)
add_compile_options(-MMD -MP)

file(GLOB_RECURSE SOURCES_SRC "Src/*.c")
file(GLOB_RECURSE SOURCES_SW4STM32 "SW4STM32/*.c")
file(GLOB_RECURSE SOURCES_SW4STM32_ASSEMBLY "SW4STM32/*.s")

set(SOURCES_HAL "../../../../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_crc_ex.c")
set(SOURCES_HAL ${SOURCES_HAL} "../../../../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_crc.c")
set(SOURCES_HAL ${SOURCES_HAL} "../../../../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_firewall.c")
set(SOURCES_HAL ${SOURCES_HAL} "../../../../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.c")
set(SOURCES_HAL ${SOURCES_HAL} "../../../../../Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.c")

#set(SOURCES_MIDDLEWARE "../../../../../Middlewares/ST/STM32_Secure_Engine/Core/se_bootinfo.c")
#set(SOURCES_MIDDLEWARE ${SOURCES_MIDDLEWARE} "../../../../../Middlewares/ST/STM32_Secure_Engine/Src/se_low_level.c")
#set(SOURCES_MIDDLEWARE ${SOURCES_MIDDLEWARE} "../../../../../Middlewares/ST/STM32_Secure_Engine/Core/se_bootinfo.c")
set(SOURCES_MIDDLEWARE ${SOURCES_MIDDLEWARE} "../../../../../Middlewares/ST/STM32_Secure_Engine/Core/se_callgate.c")
set(SOURCES_MIDDLEWARE ${SOURCES_MIDDLEWARE} "../../../../../Middlewares/ST/STM32_Secure_Engine/Core/se_crypto_common.c")
set(SOURCES_MIDDLEWARE ${SOURCES_MIDDLEWARE} "../../../../../Middlewares/ST/STM32_Secure_Engine/Core/se_exception.c")
set(SOURCES_MIDDLEWARE ${SOURCES_MIDDLEWARE} "../../../../../Middlewares/ST/STM32_Secure_Engine/Core/se_fwimg.c")
set(SOURCES_MIDDLEWARE ${SOURCES_MIDDLEWARE} "../../../../../Middlewares/ST/STM32_Secure_Engine/Core/se_stack_smuggler_GNU.s")
set(SOURCES_MIDDLEWARE ${SOURCES_MIDDLEWARE} "../../../../../Middlewares/ST/STM32_Secure_Engine/Core/se_startup.c")
set(SOURCES_MIDDLEWARE ${SOURCES_MIDDLEWARE} "../../../../../Middlewares/ST/STM32_Secure_Engine/Core/se_user_application.c")
set(SOURCES_MIDDLEWARE ${SOURCES_MIDDLEWARE} "../../../../../Middlewares/ST/STM32_Secure_Engine/Core/se_utils.c")

set(SOURCE_FILES ${SOURCES_SW4STM32_ASSEMBLY} ${SOURCES_SRC} ${SOURCES_SW4STM32} ${SOURCES_MIDDLEWARE} ${SOURCES_HAL})

add_executable(${BOOTLOADER_BINARY_NAME}  ${SOURCE_FILES})

set (CMAKE_EXE_LINKER_FLAGS "-mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -specs=nosys.specs -specs=nano.specs -Xlinker --gc-sections -L ${CMAKE_CURRENT_SOURCE_DIR}/../Linker_Common/SW4STM32/ -L ${CMAKE_CURRENT_SOURCE_DIR}/SW4STM32/STM32L496-Karmeliet_2_Images_SECoreBin/ -TSTM32L496VGTx_FLASH.ld -Wl,-Map=output.map")

add_library(libSTM32Cryptographic STATIC IMPORTED)
set_target_properties(libSTM32Cryptographic PROPERTIES
	IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/../../../../../Middlewares/ST/STM32_Cryptographic/Fw_Crypto/STM32L4/Lib/libSTM32CryptographicV3.0.0_CM4_GCC.a)
set(PROJECT_LIBRARY libSTM32Cryptographic m)
target_link_libraries(${BOOTLOADER_BINARY_NAME} ${PROJECT_LIBRARY})

add_custom_command(TARGET ${BOOTLOADER_BINARY_NAME}
 POST_BUILD
 COMMAND arm-none-eabi-objcopy -O binary "${BOOTLOADER_BINARY_NAME}" "${BOOTLOADER_BINARY_NAME}.bin"
 COMMENT "Invoking: Binary file creation"
)

add_custom_command(TARGET ${BOOTLOADER_BINARY_NAME}
 POST_BUILD
 COMMAND arm-none-eabi-size --format=berkeley "${BOOTLOADER_BINARY_NAME}"
 COMMENT "Invoking: Cross ARM GNU Print Size"
)
