# examples/sensors/livox_mid360s/CMakeLists.txt

add_executable(livox-mid360s-hakoniwa-asset
    livox-mid360s-hakoniwa-asset.cpp
    ${PROJECT_ROOT_DIR}/src/config/asset_manifest.cpp
    ${PROJECT_ROOT_DIR}/src/runtime/hakoniwa_asset_lifecycle.cpp
)

target_compile_features(livox-mid360s-hakoniwa-asset PRIVATE cxx_std_20)
hako_configure_target_warnings(livox-mid360s-hakoniwa-asset)

target_include_directories(livox-mid360s-hakoniwa-asset
    PRIVATE ${PROJECT_ROOT_DIR}
    PRIVATE ${PROJECT_ROOT_DIR}/src
    PRIVATE ${PROJECT_ROOT_DIR}/include
    PRIVATE ${HAKO_CORE_INCLUDE_BASE}
    PRIVATE ${HAKO_CORE_INCLUDE_ROOT}
    PRIVATE ${HAKO_PDU_TYPES_INCLUDE_DIR}
)
target_include_directories(livox-mid360s-hakoniwa-asset SYSTEM PRIVATE
    ${MUJOCO_SOURCE_DIR}
    ${PROJECT_ROOT_DIR}/thirdparty/nolman/single_include
)

target_link_libraries(livox-mid360s-hakoniwa-asset
    PRIVATE
        msensors
        ${HAKO_ASSETS_LINK_TARGET}
        ${HAKO_CONDUCTOR_LINK_TARGET}
        ${HAKO_PDU_ENDPOINT_LINK_TARGET}
)

hako_configure_windows_runtime(livox-mid360s-hakoniwa-asset)
