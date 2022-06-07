set_xmakever("2.5.9")

add_requires("eigen", "spdlog")
add_rules("mode.release", "mode.debug")
set_languages("cxx17")

target("SoftRender")
    set_kind("binary")
    add_includedirs("src")
    add_files("src/main.cpp", "src/common/*.cpp", "src/graphics/*.cpp", "src/renderer/*.cpp", "src/ik/*.cpp", "src/massspring/*.cpp")
    if is_os("windows") then
        add_files("src/platforms/win32.cpp")
        add_links("Gdi32", "User32")
    elseif is_os("macosx") then
        add_frameworks("Cocoa")
        add_files("src/platforms/macos.mm")
        add_mxxflags("-fno-objc-arc")
    end
    add_packages("eigen", "spdlog", {public=true})
    set_targetdir("bin")