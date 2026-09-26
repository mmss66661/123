set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# Some default GCC settings
# arm-none-eabi- must be part of path environment
set(TOOLCHAIN_PREFIX                arm-none-eabi-)

# Try to locate the toolchain.
# Priority: ARM_TOOLCHAIN_PATH env var > well-known install dirs > system PATH.
if(NOT DEFINED ARM_TOOLCHAIN_PATH)
    if(DEFINED ENV{ARM_TOOLCHAIN_PATH})
        set(ARM_TOOLCHAIN_PATH "$ENV{ARM_TOOLCHAIN_PATH}")
    else()
        # Candidate absolute directories (bin folder of GNU Arm Embedded Toolchain)
        set(_candidates
            "E:/STM32CubeCLT_1.18.0/GNU-tools-for-STM32/bin"
            "C:/Users/31148/Desktop/demo0701_KEIL/gdbbin"
            "C:/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.10/bin"
            "C:/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/bin"
        )
        foreach(_cand IN LISTS _candidates)
            if(EXISTS "${_cand}/${TOOLCHAIN_PREFIX}gcc.exe" OR
               EXISTS "${_cand}/${TOOLCHAIN_PREFIX}gcc")
                set(ARM_TOOLCHAIN_PATH "${_cand}")
                break()
            endif()
        endforeach()
    endif()
endif()

if(WIN32)
    set(_exe ".exe")
else()
    set(_exe "")
endif()

if(DEFINED ARM_TOOLCHAIN_PATH AND NOT ARM_TOOLCHAIN_PATH STREQUAL "")
    set(CMAKE_C_COMPILER    "${ARM_TOOLCHAIN_PATH}/${TOOLCHAIN_PREFIX}gcc${_exe}")
    set(CMAKE_CXX_COMPILER  "${ARM_TOOLCHAIN_PATH}/${TOOLCHAIN_PREFIX}g++${_exe}")
    set(CMAKE_ASM_COMPILER  "${CMAKE_C_COMPILER}")
    set(CMAKE_LINKER        "${CMAKE_CXX_COMPILER}")
    set(CMAKE_OBJCOPY       "${ARM_TOOLCHAIN_PATH}/${TOOLCHAIN_PREFIX}objcopy${_exe}")
    set(CMAKE_SIZE          "${ARM_TOOLCHAIN_PATH}/${TOOLCHAIN_PREFIX}size${_exe}")
else()
    set(CMAKE_C_COMPILER    ${TOOLCHAIN_PREFIX}gcc)
    set(CMAKE_ASM_COMPILER  ${CMAKE_C_COMPILER})
    set(CMAKE_CXX_COMPILER  ${TOOLCHAIN_PREFIX}g++)
    set(CMAKE_LINKER        ${TOOLCHAIN_PREFIX}g++)
    set(CMAKE_OBJCOPY       ${TOOLCHAIN_PREFIX}objcopy)
    set(CMAKE_SIZE          ${TOOLCHAIN_PREFIX}size)
endif()

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32F407XX_FLASH.ld\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")
