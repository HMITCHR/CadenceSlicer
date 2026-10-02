#include <unistd.h>
#include <sys/sysctl.h>
#import <wx/osx/cocoa/dataview.h>
#import <QuartzCore/QuartzCore.h>
#import "GUI_Utils.hpp"

namespace Slic3r {
namespace GUI {

void dataview_remove_insets(wxDataViewCtrl* dv) {
    NSScrollView* scrollview = (NSScrollView*) ((wxCocoaDataViewControl*)dv->GetDataViewPeer())->GetWXWidget();
    NSOutlineView* outlineview = scrollview.documentView;
    [outlineview setIntercellSpacing: NSMakeSize(0.0, 1.0)];
    if (@available(macOS 11, *)) {
        [outlineview setStyle:NSTableViewStylePlain];
    }
}

void staticbox_remove_margin(wxStaticBox* sb) {
    NSBox* nativeBox = (NSBox*)sb->GetHandle();
    [nativeBox setBoxType:NSBoxCustom];
    [nativeBox setBorderWidth:0];
}

std::string staticbox_content_state(wxStaticBox* sb) {
    NSBox* nativeBox = (NSBox*)sb->GetHandle();
    NSView* content = nativeBox == nil ? nil : [nativeBox contentView];
    if (content == nil)
        return "content=none";
    const NSRect frame = [content frame];
    const NSRect bounds = [nativeBox bounds];
    // Native state wx does not see: alpha and layer flags of the box and its content view, whether
    // the content view is still the box's subview, and how many row views are hidden natively, faded
    // out, or outside the content view's bounds.
    const NSRect content_bounds = [content bounds];
    unsigned long hidden_rows = 0, outside_rows = 0, faded_rows = 0;
    for (NSView* row in [content subviews]) {
        if ([row isHidden]) ++hidden_rows;
        if ([row alphaValue] < 0.01) ++faded_rows;
        if (!NSIntersectsRect([row frame], content_bounds)) ++outside_rows;
    }
    CALayer* box_layer = [nativeBox layer];
    CALayer* content_layer = [content layer];
    char text[400];
    snprintf(text, sizeof(text),
             "content=hidden:%d@%.0f,%.0f,%.0f,%.0f box_bounds=%.0f,%.0f subviews=%lu"
             " native_rows=hidden:%lu,outside:%lu,faded:%lu content_parent=%d box_hidden=%d/%d"
             " alpha=%.2f/%.2f layers=%d:%d:%.2f/%d:%d:%.2f in_window=%d",
             int([content isHidden]), frame.origin.x, frame.origin.y, frame.size.width, frame.size.height,
             bounds.size.width, bounds.size.height, (unsigned long)[[content subviews] count],
             hidden_rows, outside_rows, faded_rows, int([content superview] == nativeBox),
             int([nativeBox isHidden]), int([nativeBox isHiddenOrHasHiddenAncestor]),
             double([nativeBox alphaValue]), double([content alphaValue]),
             int(box_layer != nil), int(box_layer != nil && [box_layer isHidden]),
             box_layer != nil ? double([box_layer opacity]) : -1.,
             int(content_layer != nil), int(content_layer != nil && [content_layer isHidden]),
             content_layer != nil ? double([content_layer opacity]) : -1.,
             int([nativeBox window] != nil));
    return text;
}

bool staticbox_repair_content(wxStaticBox* sb) {
    NSBox* nativeBox = (NSBox*)sb->GetHandle();
    NSView* content = nativeBox == nil ? nil : [nativeBox contentView];
    if (content == nil)
        return false;
    bool repaired = false;
    if ([content isHidden]) {
        [content setHidden:NO];
        repaired = true;
    }
    const NSRect bounds = [nativeBox bounds];
    const NSRect frame = [content frame];
    // Only a collapsed content view is touched; a normal one is left exactly where AppKit tiled it.
    if (bounds.size.width >= 1. && bounds.size.height >= 1. &&
        (frame.size.width < 1. || frame.size.height < 1.)) {
        [content setFrame:bounds];
        repaired = true;
    }
    if (repaired) {
        [content setNeedsDisplay:YES];
        [nativeBox setNeedsDisplay:YES];
    }
    return repaired;
}

bool is_debugger_present()
// Returns true if the current process is being debugged (either
// running under the debugger or has a debugger attached post facto).
// https://stackoverflow.com/a/2200786/3289421
{
    int                 junk;
    int                 mib[4];
    struct kinfo_proc   info;
    size_t              size;

    // Initialize the flags so that, if sysctl fails for some bizarre
    // reason, we get a predictable result.

    info.kp_proc.p_flag = 0;

    // Initialize mib, which tells sysctl the info we want, in this case
    // we're looking for information about a specific process ID.

    mib[0] = CTL_KERN;
    mib[1] = KERN_PROC;
    mib[2] = KERN_PROC_PID;
    mib[3] = getpid();

    // Call sysctl.

    size = sizeof(info);
    junk = sysctl(mib, sizeof(mib) / sizeof(*mib), &info, &size, NULL, 0);
    assert(junk == 0);

    // We're being debugged if the P_TRACED flag is set.

    return ( (info.kp_proc.p_flag & P_TRACED) != 0 );
}

}
}

