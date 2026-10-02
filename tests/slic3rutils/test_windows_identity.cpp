#include <catch2/catch_all.hpp>

#include <boost/filesystem.hpp>
#include <boost/nowide/fstream.hpp>

#include <sstream>
#include <string>

// Cadence Slicer installs next to OrcaSlicer on Windows, so the two
// must not share an executable name, file association ProgID or version-resource identity. These
// checks read the build and installer sources, because none of it can run on the Mac that builds
// the tests.

namespace fs = boost::filesystem;

namespace {

std::string source_file(const std::string &relative)
{
    // PROFILES_DIR is <source root>/resources/profiles, so the source root is two levels up.
    const fs::path file = fs::path(PROFILES_DIR).parent_path().parent_path() / relative;
    REQUIRE(fs::exists(file));
    boost::nowide::ifstream stream(file.string().c_str(), std::ios::binary);
    REQUIRE(stream.good());
    std::ostringstream text;
    text << stream.rdbuf();
    return text.str();
}

bool contains(const std::string &text, const std::string &needle) { return text.find(needle) != std::string::npos; }

} // namespace

TEST_CASE("The Windows executable is CadenceSlicer.exe", "[WindowsIdentity]")
{
    // Everything that builds, installs, launches or looks for the Windows program.
    for (const char *relative : {"CMakeLists.txt", "src/CMakeLists.txt", "src/slic3r/Utils/Process.cpp",
                                 "src/slic3r/Utils/BBLNetworkPlugin.cpp",
                                 "src/dev-utils/platform/msw/OrcaSlicer.rc.in", "scripts/msix/AppxManifest.xml",
                                 "scripts/msix/build_msix.ps1"}) {
        CAPTURE(relative);
        CHECK_FALSE(contains(source_file(relative), "orca-slicer.exe"));
    }

    const std::string app = source_file("src/CMakeLists.txt");
    const std::size_t gui = app.find("set_target_properties(OrcaSlicer_app_gui PROPERTIES");
    REQUIRE(gui != std::string::npos);
    CHECK(contains(app.substr(gui, 120), "OUTPUT_NAME \"CadenceSlicer\""));

    const std::string root = source_file("CMakeLists.txt");
    CHECK(contains(root, "set (CPACK_NSIS_INSTALLED_ICON_NAME  \"$INSTDIR\\\\\\\\CadenceSlicer.exe\")"));
    CHECK(contains(root, "set(CPACK_PACKAGE_EXECUTABLES \"CadenceSlicer;${SLIC3R_APP_NAME}\")"));
    CHECK(contains(root, "set(CPACK_CREATE_DESKTOP_LINKS \"CadenceSlicer\")"));

    CHECK(contains(source_file("src/slic3r/Utils/Process.cpp"), "\"CadenceSlicer.exe\""));
    CHECK(contains(source_file("src/slic3r/Utils/BBLNetworkPlugin.cpp"), "\"CadenceSlicer.exe\""));
    CHECK(contains(source_file("src/dev-utils/platform/msw/OrcaSlicer.rc.in"),
                   "VALUE \"OriginalFilename\", \"CadenceSlicer.exe\""));
    const std::string manifest = source_file("scripts/msix/AppxManifest.xml");
    CHECK(contains(manifest, "Executable=\"CadenceSlicer.exe\""));
    CHECK(contains(manifest, "Alias=\"CadenceSlicer.exe\""));
}

TEST_CASE("Windows file associations use Cadence's own ProgID", "[WindowsIdentity]")
{
    // OrcaSlicer registers " Orca.Slicer.1". Sharing it would let each app take over the other's
    // .3mf, .stl, .step and .gcode associations.
    const std::string app = source_file("src/slic3r/GUI/GUI_App.cpp");
    CHECK_FALSE(contains(app, "Orca.Slicer.1"));
    CHECK(contains(app, "L\"Cadence.Slicer.1\""));
    // One constant serves the check, the registration and the removal, so they cannot drift apart.
    std::size_t uses = 0;
    for (std::size_t at = app.find("cadence_prog_id"); at != std::string::npos; at = app.find("cadence_prog_id", at + 1))
        ++uses;
    CHECK(uses == 4);

    const std::string manifest = source_file("scripts/msix/AppxManifest.xml");
    CHECK_FALSE(contains(manifest, "Orca.Slicer.1"));
    CHECK(contains(manifest, "<rescap3:MigrationProgId>Cadence.Slicer.1</rescap3:MigrationProgId>"));
}

TEST_CASE("The Windows version resource names Cadence Slicer", "[WindowsIdentity]")
{
    const std::string rc = source_file("src/dev-utils/platform/msw/OrcaSlicer.rc.in");
    // The company is set in one place, version.inc.
    CHECK(contains(rc, "VALUE \"CompanyName\", \"@CADENCE_COMPANY_NAME@\""));
    CHECK(contains(source_file("version.inc"), "set(CADENCE_COMPANY_NAME \"HMITCHR\")"));
    CHECK(contains(rc, "VALUE \"ProductName\", \"Cadence Slicer\""));
    CHECK(contains(rc, "VALUE \"FileDescription\", \"Cadence Slicer\""));
    // Upstream credit stays, and Cadence is added.
    const std::size_t copyright = rc.find("VALUE \"LegalCopyright\"");
    REQUIRE(copyright != std::string::npos);
    const std::string line = rc.substr(copyright, rc.find('\n', copyright) - copyright);
    CHECK(contains(line, "SoftFever"));
    CHECK(contains(line, "OrcaSlicer"));
    CHECK(contains(line, "Cadence Slicer"));
}
