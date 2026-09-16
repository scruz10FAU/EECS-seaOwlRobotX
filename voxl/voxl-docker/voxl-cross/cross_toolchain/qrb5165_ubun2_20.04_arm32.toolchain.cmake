## just use the same toolchain as the QRB5165 ubun1 18.04 targert

message(STATUS "---------------------------------------------------------")
message(STATUS "WARNING building 32-bit binaries for QRB5165 ubun2 20.04")
message(STATUS "is not recommended. Using the ubun1 18.04 32-bit toolchain.")
message(STATUS "---------------------------------------------------------")

include(/opt/cross_toolchain/qrb5165_ubun1_18.04_arm32.toolchain.cmake)
