#ifndef slic3r_GUI_WebView_hpp_
#define slic3r_GUI_WebView_hpp_

#include <wx/webview.h>
#include <wx/event.h>

#include <string>

wxDECLARE_EVENT(EVT_WEBVIEW_RECREATED, wxCommandEvent);

class WebView
{
public:
    static wxWebView *CreateWebView(wxWindow *parent, wxString const &url);
#if wxUSE_WEBVIEW_EDGE
    static bool CheckWebViewRuntime();
    static bool DownloadAndInstallWebViewRuntime();
#endif
    static void LoadUrl(wxWebView * webView, wxString const &url);

    static bool RunScript(wxWebView * webView, wxString const & msg);

    // Marks "wx" as registered so CreateWebView's deferred add skips the duplicate.
    static void MarkScriptMessageHandlerAdded(wxWebView * webView);

    static void RecreateAll();

    // The token the embedded view uses to name itself, on the same base as the curl user agent.
    // The bundled pages under resources/web look for it to tell they run inside the slicer.
    static std::string ClientToken();

    // The full user agent the embedded view presents. The bundled pages read the
    // theme and the language back out of it, so both stay in the string.
    static std::string UserAgent(bool dark_mode, std::string const &language_code);
};

#endif // !slic3r_GUI_WebView_hpp_
