set(REXKERNEL_SOURCES
    kernel_init.cpp

    # XAM subsystem - provides __imp__Xam* symbols
    xam/xam_module.cpp
    xam/xam_avatar.cpp
    xam/xam_content.cpp
    xam/xam_content_aggregate.cpp
    xam/xam_content_device.cpp
    xam/xam_debug.cpp
    xam/xam_enum.cpp
    xam/xam_info.cpp
    xam/xam_input.cpp
    xam/xam_locale.cpp
    xam/xam_msg.cpp
    xam/xam_net.cpp
    xam/xam_notify.cpp
    xam/xam_nui.cpp
    xam/xam_party.cpp
    xam/xam_task.cpp
    xam/xam_ui.cpp
    xam/xam_friends.cpp
    xam/xam_user.cpp
    xam/xam_video.cpp
    xam/xam_voice.cpp
    xam/xam_misc.cpp
    xam/apps/xam_app.cpp
    xam/apps/xgi_app.cpp
    xam/apps/xlivebase_app.cpp
    xam/apps/xmp_app.cpp

    # XBDM subsystem - provides __imp__Xbdm* symbols
    xbdm/xbdm_module.cpp
    xbdm/xbdm_misc.cpp

    # xboxkrnl exports - provides __imp__ symbols for generated code
    # xboxkrnl/cert_monitor.cpp    # TODO: Translate JIT methods for AOT
    # xboxkrnl/debug_monitor.cpp   # TODO: Translate JIT methods for AOT
    xboxkrnl/xboxkrnl_crypt.cpp
    xboxkrnl/xboxkrnl_debug.cpp
    xboxkrnl/xboxkrnl_error.cpp
    xboxkrnl/xboxkrnl_hal.cpp
    xboxkrnl/xboxkrnl_hid.cpp
    xboxkrnl/xboxkrnl_io.cpp
    xboxkrnl/xboxkrnl_io_info.cpp
    xboxkrnl/xboxkrnl_memory.cpp
    xboxkrnl/xboxkrnl_misc.cpp
    xboxkrnl/xboxkrnl_module.cpp
    xboxkrnl/xboxkrnl_modules.cpp
    xboxkrnl/xboxkrnl_ob.cpp
    xboxkrnl/xboxkrnl_rtl.cpp
    xboxkrnl/xboxkrnl_strings.cpp
    xboxkrnl/xboxkrnl_threading.cpp
    # xboxkrnl/xboxkrnl_usbcam.cpp  # TODO: lol eventually.
    xboxkrnl/xboxkrnl_video.cpp
    xboxkrnl/xboxkrnl_xconfig.cpp
    xboxkrnl/xboxkrnl_audio.cpp
    xboxkrnl/xboxkrnl_audio_xma.cpp

    crt/heap.cpp
    crt/file.cpp
    crt/memory.cpp
    crt/string.cpp
    crt/threading.cpp
)

list(TRANSFORM REXKERNEL_SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(rexruntime PRIVATE ${REXKERNEL_SOURCES})

target_link_libraries(rexruntime PRIVATE o1heap rexcore rexfilesystem rexui rexaudio rexinput imgui)

if(REXGLUE_ENABLE_REXNET)
    # RexNet netplay module: XNetStartup/XNetCleanup manage the shared
    # instance (see xam/xam_net.cpp).
    #
    # PUBLIC, so consumers that compile SDK reference sources (rex_app.cpp is
    # not built here — apps compile their own copy) can gate on it and agree
    # with the library they link. This is safe *because* no public header's
    # layout depends on it: rex_app.h deliberately holds the RexNet overlay
    # as its base type so the struct is identical either way. Keep it that
    # way — if a public header ever branches on this define, a consumer built
    # without it will silently disagree about object layout.
    # The link stays PRIVATE: rexnet is an OBJECT library absorbed into this
    # shared library, so consumers never link it themselves and exporting it
    # would only add a target they cannot resolve. Only the *define* needs to
    # travel outward.
    target_link_libraries(rexruntime PRIVATE rexnet)
    target_compile_definitions(rexruntime PUBLIC REXGLUE_ENABLE_REXNET=1)
endif()
