#include <catch2/catch_all.hpp>

#include <libslic3r/libslic3r.h>  // CADENCE_PRODUCT_VERSION, SLIC3R_APP_NAME (via libslic3r_version.h)
#include "slic3r/GUI/Widgets/WebView.hpp"

#include <boost/filesystem.hpp>
#include <boost/nowide/fstream.hpp>

#include <algorithm>
#include <cctype>
#include <sstream>
#include <string>
#include <vector>

namespace fs = boost::filesystem;

namespace {

bool contains(std::string const &haystack, std::string const &needle)
{
    return haystack.find(needle) != std::string::npos;
}

std::string read_text_file(fs::path const &path)
{
    boost::nowide::ifstream in(path.string(), std::ios::binary);
    std::ostringstream buf;
    buf << in.rdbuf();
    return buf.str();
}

bool is_page_file(fs::path const &path)
{
    std::string const ext = path.extension().string();
    return ext == ".js" || ext == ".html" || ext == ".htm";
}

std::string join(std::vector<std::string> const &names)
{
    std::string out;
    for (auto const &name : names) {
        if (!out.empty())
            out += ", ";
        out += name;
    }
    return out;
}

// Every distinct "BBL-" name in one page, in the order it first appears. The walk below covers
// the whole prefix, not just the client token, so it needs the names and not just a yes or no: the two the guide's sample data carries are printer model ids and have to be told
// apart from a slicer name.
std::vector<std::string> bambu_prefixed_names(std::string const &text)
{
    static std::string const prefix = "BBL-";

    std::vector<std::string> names;
    for (size_t at = text.find(prefix); at != std::string::npos; at = text.find(prefix, at + 1)) {
        size_t end = at + prefix.size();
        while (end < text.size() && (std::isalnum(static_cast<unsigned char>(text[end])) != 0 ||
                                     text[end] == '-' || text[end] == '_'))
            ++end;
        std::string name = text.substr(at, end - at);
        if (std::find(names.begin(), names.end(), name) == names.end())
            names.push_back(std::move(name));
    }
    return names;
}

bool starts_with(std::string const &text, std::string const &prefix)
{
    return text.rfind(prefix, 0) == 0;
}

} // namespace

// Bambu's stated objection to the fork it sent a cease and desist to was that the fork
// presented itself as the official Bambu Studio client when it talked to Bambu's servers.
// The embedded view loads Bambu-hosted pages, so the token it carries has to be ours.
TEST_CASE("The embedded web view names Cadence, not a Bambu client", "[WebViewIdentity]")
{
    std::string const token = WebView::ClientToken();
    std::string const agent = WebView::UserAgent(true, "en");

    CHECK_FALSE(contains(token, "BBL-Slicer"));
    CHECK_FALSE(contains(agent, "BBL-Slicer"));

    // Nothing in what the view sends is Bambu-named, not the client token, not the language
    // token and not the rest of the string.
    CHECK_FALSE(contains(token, "BBL-"));
    CHECK_FALSE(contains(agent, "BBL-"));

    // Built on the same base as the curl user agent in Http.cpp: the application name
    // and the product version, never the Bambu client version the plug-in path reports.
    CHECK(token == std::string(SLIC3R_APP_NAME) + "/" + CADENCE_PRODUCT_VERSION);
    CHECK(contains(agent, "CadenceSlicer/"));
    CHECK(contains(agent, std::string("CadenceSlicer/") + CADENCE_PRODUCT_VERSION));
}

// The bundled pages read the theme and the language back out of the agent, so the rest
// of the string keeps its shape when the token changes.
TEST_CASE("The web view user agent keeps the theme and language the bundled pages read", "[WebViewIdentity]")
{
    CHECK(contains(WebView::UserAgent(true, "en"), "(dark)"));
    CHECK(contains(WebView::UserAgent(false, "en"), "(light)"));
    CHECK(contains(WebView::UserAgent(false, "de"), "Language/de"));

    // Same value and same place in the string, under our own name. A page that ever wants
    // the language back reads it off this spelling.
    CHECK(contains(WebView::UserAgent(false, "de"), "Cadence-Language/de"));
    CHECK(contains(WebView::UserAgent(true, "en"), "Cadence-Language/en"));
}

// A page that sniffs the agent has to sniff for the token the application now sends.
// This walks the shipped directory rather than naming the four pages, so a page added
// later cannot quietly bring the old token back.
TEST_CASE("No bundled page identifies the slicer by a Bambu token", "[WebViewIdentity]")
{
    fs::path const web_dir(WEB_RESOURCES_DIR);
    REQUIRE(fs::is_directory(web_dir));

    std::vector<std::string> carries_bambu_token;
    std::vector<std::string> sniffs_without_cadence;
    std::vector<std::string> carries_bambu_name;
    std::vector<std::string> carries_printer_model_id;
    int sniffing_pages = 0;

    for (fs::recursive_directory_iterator it(web_dir), end; it != end; ++it) {
        if (!fs::is_regular_file(it->path()))
            continue;
        if (!is_page_file(it->path()))
            continue;

        std::string const relative = it->path().string().substr(web_dir.string().size() + 1);
        std::string const text     = read_text_file(it->path());

        if (contains(text, "BBL-Slicer"))
            carries_bambu_token.push_back(relative);

        // Every BBL- name a page could send or sniff for, not just the client token. The printer model ids in the guide's sample preset data are the single
        // exception: those name Bambu hardware, not this slicer, and they are collected
        // separately below so the exception stays where it is instead of being a blanket.
        for (std::string const &name : bambu_prefixed_names(text)) {
            if (starts_with(name, "BBL-3DP-"))
                carries_printer_model_id.push_back(relative);
            else
                carries_bambu_name.push_back(relative + " (" + name + ")");
        }

        if (contains(text, "function IsInSlicer")) {
            ++sniffing_pages;
            if (!contains(text, "CadenceSlicer"))
                sniffs_without_cadence.push_back(relative);
        }
    }

    INFO("pages still matching the Bambu token: " << join(carries_bambu_token));
    CHECK(carries_bambu_token.empty());

    INFO("pages sniffing the agent without matching Cadence: " << join(sniffs_without_cadence));
    CHECK(sniffs_without_cadence.empty());

    INFO("pages carrying a BBL- name that is not a printer model id: " << join(carries_bambu_name));
    CHECK(carries_bambu_name.empty());

    // The exception's own boundary. Only the guide's two sample-data stubs carry Bambu printer
    // model ids, and both are placeholder preset data the guide pages load, not identity. A
    // third page picking them up means reading this again rather than widening the allowance.
    std::sort(carries_printer_model_id.begin(), carries_printer_model_id.end());
    carries_printer_model_id.erase(std::unique(carries_printer_model_id.begin(), carries_printer_model_id.end()),
                                   carries_printer_model_id.end());
    std::vector<std::string> const sample_data_pages{"guide/21/test.js", "guide/22/test.js"};
    INFO("pages carrying Bambu printer model ids: " << join(carries_printer_model_id));
    CHECK(carries_printer_model_id == sample_data_pages);

    // homepage, guide, dialog and include all sniff the agent. If the checks above have gone
    // quiet rather than green, this is what says so.
    CHECK(sniffing_pages >= 4);
}
