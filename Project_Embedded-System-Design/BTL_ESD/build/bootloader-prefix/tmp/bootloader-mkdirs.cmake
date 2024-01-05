# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Espressif/frameworks/esp-idf-v5.0.2/components/bootloader/subproject"
  "D:/Project_Embedded-System-Design_ESP-IDF/Project_Embedded-System-Design/BTL_ESD/build/bootloader"
  "D:/Project_Embedded-System-Design_ESP-IDF/Project_Embedded-System-Design/BTL_ESD/build/bootloader-prefix"
  "D:/Project_Embedded-System-Design_ESP-IDF/Project_Embedded-System-Design/BTL_ESD/build/bootloader-prefix/tmp"
  "D:/Project_Embedded-System-Design_ESP-IDF/Project_Embedded-System-Design/BTL_ESD/build/bootloader-prefix/src/bootloader-stamp"
  "D:/Project_Embedded-System-Design_ESP-IDF/Project_Embedded-System-Design/BTL_ESD/build/bootloader-prefix/src"
  "D:/Project_Embedded-System-Design_ESP-IDF/Project_Embedded-System-Design/BTL_ESD/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Project_Embedded-System-Design_ESP-IDF/Project_Embedded-System-Design/BTL_ESD/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/Project_Embedded-System-Design_ESP-IDF/Project_Embedded-System-Design/BTL_ESD/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
