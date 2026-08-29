if(NOT DEFINED DESCRIPTOR_FILE OR NOT DEFINED EXPECTED_SHA256)
  message(FATAL_ERROR "descriptor path and expected SHA-256 are required")
endif()

file(SHA256 "${DESCRIPTOR_FILE}" ACTUAL_SHA256)
if(NOT ACTUAL_SHA256 STREQUAL EXPECTED_SHA256)
  message(FATAL_ERROR
    "golden protobuf descriptor changed: expected ${EXPECTED_SHA256}, got ${ACTUAL_SHA256}"
  )
endif()
