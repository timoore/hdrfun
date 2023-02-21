set(SOURCES
    hdrfun.cpp
)

add_executable(hdrfun ${SOURCES})

target_link_libraries(hdrfun vsg::vsg)

if (vsgXchange_FOUND)
    target_compile_definitions(hdrfun PRIVATE vsgXchange_FOUND)
    target_link_libraries(hdrfun vsgXchange::vsgXchange)
endif()

install(TARGETS hdrfun RUNTIME DESTINATION bin)
