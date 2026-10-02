#include "MixedNozzleRanking.hpp"

#include "GCode/GCodeProcessor.hpp"
#include "Print.hpp"

#include <boost/filesystem.hpp>

#include <cstdint>
#include <functional>
#include <iomanip>
#include <sstream>
#include <vector>

namespace Slic3r {

void MixedNozzleRankingCancel::cancel()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_cancelled.store(true, std::memory_order_release);
    if (m_print != nullptr)
        m_print->cancel();
}

void MixedNozzleRankingCancel::attach(Print *print)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_print = print;
    // A cancel that came before the Print existed still stops it.
    if (m_print != nullptr && m_cancelled.load(std::memory_order_acquire))
        m_print->cancel();
}

namespace {

// Detaches the Print from the cancel token however the slice ends, so a late cancel never
// reaches a Print that has gone.
struct AttachedPrint {
    MixedNozzleRankingCancel &cancel;
    AttachedPrint(MixedNozzleRankingCancel &cancel, Print &print) : cancel(cancel) { cancel.attach(&print); }
    ~AttachedPrint() { cancel.attach(nullptr); }
};

void remove_quietly(const boost::filesystem::path &path)
{
    boost::system::error_code ignored;
    boost::filesystem::remove(path, ignored);
}

} // namespace

MixedNozzleSliceTime mixed_nozzle_slice_time(const Model &model, const DynamicPrintConfig &config,
                                             bool is_bbl_printer, const Vec3d &plate_origin,
                                             const std::string &temp_dir,
                                             MixedNozzleRankingCancel &cancel)
{
    MixedNozzleSliceTime out;
    if (cancel.cancelled()) {
        out.status = MixedNozzleSliceTimeStatus::Cancelled;
        return out;
    }
    boost::filesystem::path gcode;
    try {
        Print print;
        print.set_status_silent();
        // BackgroundSlicingProcess sets these before it validates; the native tower type depends
        // on the first and the G-code coordinates on the second.
        print.is_BBL_printer() = is_bbl_printer;
        print.set_plate_origin(plate_origin);
        const AttachedPrint attached(cancel, print);
        // Applied twice, as PartPlate does: the second apply sees the objects the first created.
        print.apply(model, config);
        print.apply(model, config);
        const StringObjectException error = print.validate();
        if (!error.string.empty()) {
            out.status = MixedNozzleSliceTimeStatus::Refused;
            out.diagnostic = error.string;
            return out;
        }
        print.process();
        const boost::filesystem::path directory = temp_dir.empty()
            ? boost::filesystem::temp_directory_path() : boost::filesystem::path(temp_dir);
        gcode = directory / boost::filesystem::unique_path("cadence-ranking-%%%%-%%%%-%%%%.gcode");
        GCodeProcessorResult result;
        print.export_gcode(gcode.string(), &result, nullptr);
        const auto &statistics = result.print_statistics;
        out.seconds = double(statistics.modes[std::size_t(PrintEstimatedStatistics::ETimeMode::Normal)].time);
        out.switches = statistics.total_physical_tool_changes;
        for (const auto &[extruder, volume] : statistics.wipe_tower_volumes_per_extruder)
            out.tower_mm3 += volume;
        out.status = MixedNozzleSliceTimeStatus::Estimated;
    } catch (const CanceledException &) {
        out = MixedNozzleSliceTime{};
        out.status = MixedNozzleSliceTimeStatus::Cancelled;
    } catch (const std::exception &error) {
        out = MixedNozzleSliceTime{};
        out.status = cancel.cancelled() ? MixedNozzleSliceTimeStatus::Cancelled
                                        : MixedNozzleSliceTimeStatus::Failed;
        out.diagnostic = error.what();
    }
    if (!gcode.empty()) {
        remove_quietly(gcode);
        remove_quietly(boost::filesystem::path(gcode.string() + ".tmp"));
    }
    return out;
}

namespace {

// The keys that name a material for one feature. A baseline prints them all with the fine one.
const std::vector<std::string> &feature_filament_keys()
{
    static const std::vector<std::string> keys{
        "outer_wall_filament_id", "inner_wall_filament_id", "sparse_infill_filament_id",
        "internal_solid_filament_id", "top_surface_filament_id", "bottom_surface_filament_id"};
    return keys;
}

// Every part, modifier and layer range of the baseline prints the fine material at the fine
// height, with no Body Split values left on it.
void put_on_fine(ModelConfig &config, int fine_filament, double fine_height)
{
    if (config.has("extruder"))
        config.set_key_value("extruder", new ConfigOptionInt(fine_filament));
    for (const std::string &key : feature_filament_keys())
        if (config.has(key))
            config.set_key_value(key, new ConfigOptionInt(fine_filament));
    if (config.has("layer_height"))
        config.set_key_value("layer_height", new ConfigOptionFloat(fine_height));
    for (const char *key : {"regional_layer_height", "mixed_nozzle_body_fine_skins",
                            "mixed_nozzle_body_fine_skin_layers"})
        config.erase(key);
}

} // namespace

void mixed_nozzle_single_nozzle_baseline(Model &model, DynamicPrintConfig &config,
                                         std::size_t fine_logical_filament, double fine_height)
{
    const int fine_filament = int(fine_logical_filament) + 1;
    config.set_key_value("mixed_nozzle_slicing_mode",
                         new ConfigOptionEnum<MixedNozzleSlicingMode>(MixedNozzleSlicingMode::Off));
    config.set_key_value("layer_height", new ConfigOptionFloat(fine_height));
    if (const auto *map = config.option<ConfigOptionInts>("filament_map");
        map != nullptr && fine_logical_filament < map->values.size()) {
        const int fine_tool = map->values[fine_logical_filament];
        config.set_key_value("filament_map", new ConfigOptionInts(std::vector<int>(map->values.size(), fine_tool)));
    }
    for (const std::string &key : feature_filament_keys())
        config.set_key_value(key, new ConfigOptionInt(fine_filament));
    // Supports and the tower follow the object's material, which is now the fine one.
    for (const char *key : {"support_filament", "support_interface_filament", "wipe_tower_filament"})
        if (config.has(key))
            config.set_key_value(key, new ConfigOptionInt(0));

    for (ModelObject *object : model.objects) {
        if (object == nullptr)
            continue;
        put_on_fine(object->config, fine_filament, fine_height);
        for (auto &[range, range_config] : object->layer_config_ranges)
            put_on_fine(range_config, fine_filament, fine_height);
        for (ModelVolume *volume : object->volumes) {
            if (volume == nullptr)
                continue;
            put_on_fine(volume->config, fine_filament, fine_height);
            // Colour painting names other materials; one nozzle prints one of them.
            volume->mmu_segmentation_facets.reset();
        }
    }
}

namespace {

struct KeyHash {
    std::uint64_t value {1469598103934665603ULL};
    void add(std::size_t item) { value ^= std::uint64_t(item) + 0x9e3779b97f4a7c15ULL + (value << 6) + (value >> 2); }
    void add(const std::string &text) { add(std::hash<std::string>{}(text)); }
    void add(double number) { add(std::hash<double>{}(number)); }
    void add(const ConfigBase &config)
    {
        for (const std::string &key : config.keys()) {
            add(key);
            add(config.opt_serialize(key));
        }
    }
    void add(const FacetsAnnotation &facets)
    {
        const auto &data = facets.get_data();
        add(data.triangles_to_split.size());
        add(std::hash<std::vector<bool>>{}(data.bitstream));
    }
    template<class Matrix> void add_matrix(const Matrix &matrix)
    {
        for (int index = 0; index < 16; ++index)
            add(double(matrix.data()[index]));
    }
    std::string text() const
    {
        std::ostringstream out;
        out << std::hex << std::setw(16) << std::setfill('0') << value;
        return out.str();
    }
};

} // namespace

std::string mixed_nozzle_ranking_key(std::size_t plate_id, const Model &model,
                                     const DynamicPrintConfig &config)
{
    KeyHash model_hash;
    for (const ModelObject *object : model.objects) {
        if (object == nullptr)
            continue;
        model_hash.add(object->name);
        model_hash.add(object->config.get());
        for (const auto &[range, range_config] : object->layer_config_ranges) {
            model_hash.add(range.first);
            model_hash.add(range.second);
            model_hash.add(range_config.get());
        }
        for (const ModelVolume *volume : object->volumes) {
            if (volume == nullptr)
                continue;
            model_hash.add(std::size_t(volume->type()));
            const indexed_triangle_set &its = volume->mesh().its;
            model_hash.add(its.vertices.size());
            model_hash.add(its.indices.size());
            for (const stl_vertex &vertex : its.vertices)
                for (int axis = 0; axis < 3; ++axis)
                    model_hash.add(double(vertex[axis]));
            model_hash.add_matrix(volume->get_matrix());
            model_hash.add(volume->config.get());
            model_hash.add(volume->mmu_segmentation_facets);
            model_hash.add(volume->supported_facets);
            model_hash.add(volume->seam_facets);
        }
        for (const ModelInstance *instance : object->instances) {
            if (instance == nullptr)
                continue;
            model_hash.add_matrix(instance->get_matrix());
            model_hash.add(std::size_t(instance->printable));
        }
    }
    KeyHash config_hash;
    config_hash.add(config);
    return std::to_string(plate_id) + ":" + model_hash.text() + ":" + config_hash.text();
}

} // namespace Slic3r
