set(SHADER_FILES
  shaders/standard.vert
  shaders/standard_pbr.frag)

install(FILES ${SHADER_FILES}
  DESTINATION ${HDRFUN_DATA_DIR}/shaders)
