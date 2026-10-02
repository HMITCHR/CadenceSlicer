#ifndef slic3r_GUI_ProjectLoadNative_hpp_
#define slic3r_GUI_ProjectLoadNative_hpp_

namespace Slic3r::GUI {

// The startup / recreate-GUI restore path replaces the current state with a fresh Untitled
// project only when startup genuinely loaded nothing. A project that carried a config but no
// geometry HAS been loaded; resetting it discards project-owned state.
inline bool startup_restore_should_start_new_project(bool project_filename_empty,
                                                     bool model_empty,
                                                     bool project_config_loaded)
{
    return project_filename_empty && model_empty && !project_config_loaded;
}

} // namespace Slic3r::GUI

#endif // slic3r_GUI_ProjectLoadNative_hpp_
