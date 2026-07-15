# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "")
  file(REMOVE_RECURSE
  "Ros2_esp32.bin"
  "Ros2_esp32.map"
  "bootloader/bootloader.bin"
  "bootloader/bootloader.elf"
  "bootloader/bootloader.map"
  "config/sdkconfig.cmake"
  "config/sdkconfig.h"
  "esp-idf/esptool_py/flasher_args.json.in"
  "esp-idf/mbedtls/x509_crt_bundle"
  "flash_app_args"
  "flash_bootloader_args"
  "flash_project_args"
  "flasher_args.json"
  "ldgen_libraries"
  "ldgen_libraries.in"
  "project_elf_src_esp32s3.c"
  "x509_crt_bundle.S"
  "../components/micro_ros_espidf_component/esp32_toolchain.cmake"
  "../components/micro_ros_espidf_component/include"
  "../components/micro_ros_espidf_component/micro_ros_dev"
  "../components/micro_ros_espidf_component/micro_ros_src"
  )
endif()
