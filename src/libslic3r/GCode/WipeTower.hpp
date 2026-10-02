#ifndef WipeTower_
#define WipeTower_

#include <cmath>
#include <limits>
#include <string>
#include <sstream>
#include <utility>
#include <algorithm>

#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include <unordered_set>
#include "libslic3r/MultiNozzleUtils.hpp"
namespace Slic3r
{

class WipeTowerWriter;
class PrintConfig;
enum GCodeFlavor : unsigned char;

// Cuts the tower wall polygon open at each skip point (a toolchange's entry position)
// so the entry travel can pass through instead of crossing the printed wall. Defined in
// WipeTower.cpp, shared by WipeTower and WipeTower2.
Polylines construct_gap_for_skip_points(
    const Polygon& polygon, const std::vector<Vec2f>& skip_points, float wt_width, float gap_length, Polygon& insert_skip_polygon);

// Klipper acts on commands the instant it parses them, and its G4 reads only P (milliseconds),
// so the zero-second and seconds-valued dwells every other flavor uses neither synchronize nor
// pause there. Both defined in WipeTower.cpp, shared by WipeTower and WipeTower2.
const char* flush_planner_queue_command(GCodeFlavor flavor); // finish queued moves, e.g. around M104/M109
std::string wait_command(GCodeFlavor flavor, float seconds);  // pause for `seconds`

class WipeTower
{
public:
    friend class WipeTowerWriter;
    static const std::string never_skip_tag() { return "_GCODE_WIPE_TOWER_NEVER_SKIP_TAG"; }

	// WipeTower height to minimum depth map
	static const std::map<float, float> min_depth_per_height;
    static float get_limit_depth_by_height(float max_height);
    static float get_auto_brim_by_height(float max_height);
    // Minimum nominal footprint (mm) for this tower's planned height; 0 unless mixed-nozzle.
    float tower_footprint_floor() const;
    // The nominal rectangle needed for the printed pad (rib wall and fillet included) to reach
    // pad_extent_floor, measured on rib_section().
    float rect_floor_for_pad_extent(float pad_extent_floor) const;
    // A mixed-nozzle tower lays its own first layer slowly and with extra flow, independent of the
    // model's first layer. Other towers keep the process values.
    static constexpr float mixed_nozzle_tower_first_layer_speed = 15.f;      // mm/s, i.e. F900
    static constexpr float mixed_nozzle_tower_first_layer_flow_ratio = 1.15f;
    float first_layer_speed() const
    {
        return m_mixed_nozzle_slicing ? mixed_nozzle_tower_first_layer_speed : m_first_layer_speed;
    }
    float first_layer_flow_ratio() const
    {
        return m_mixed_nozzle_slicing ? mixed_nozzle_tower_first_layer_flow_ratio : m_first_layer_flow_ratio;
    }
    // Extra retraction (mm of filament) before the fine tool leaves the tower, to avoid a restart
    // blob. Only for a return to the fine nozzle with a resolver-sized hand-off.
    static constexpr float mixed_nozzle_fine_return_extra_retract = 0.3f;
    bool is_fine_return(size_t old_tool, size_t new_tool) const;
    // Top Z of the solid tower base (gapless wall, full-density fill, initial-layer temperature),
    // or 0 when not a mixed-nozzle tower.
    float tower_base_top_z() const { return m_tower_base_top_z; }
    bool  is_tower_base_layer() const;
    // The first layer, plus the whole solid base of a mixed-nozzle tower. Only the first layer
    // uses the first-layer speed and flow (is_first_layer()); the rest of the base does not.
    bool  is_base_layer() const { return is_first_layer() || is_tower_base_layer(); }
    // The part of the first-layer flow boost a road of this width can take without its effective
    // width exceeding 1.25 x the tool's nozzle. Purge rows are already at that limit.
    float base_flow_ratio_for_width(int tool, float road_width) const;
    static TriangleMesh                 its_make_rib_tower(float width, float depth, float height, float rib_length, float rib_width, bool fillet_wall);
    static TriangleMesh                 its_make_rib_brim(const Polygon& brim, float layer_height);
    static Polygon                      rib_section(float width, float depth, float rib_length, float rib_width, bool fillet_wall);
    // Translation that brings a footprint inside the printable outline, padded by offset. The prime
    // tower is validated against the real outline (see layered_print_cleareance_valid), so clamping
    // against the bounding box alone would leave it off a delta or hexagonal bed. box and polygons
    // must share one scaled coordinate frame; the translation comes back in millimeters.
    static Vec2f                        move_box_inside_polygon(const BoundingBox &box, const Polygons &polygons, coord_t offset = 0);
    static Polygon                      rounding_polygon(Polygon &polygon, double rounding = 2., double angle_tol = 30. / 180. * PI);
    struct Extrusion
    {
		Extrusion(const Vec2f &pos, float width, unsigned int tool) : pos(pos), width(width), tool(tool) {}
		// End position of this extrusion.
		Vec2f				pos;
		// Width of a squished extrusion, corrected for the roundings of the squished extrusions.
		// This is left zero if it is a travel move.
		float 			width;
		// Current extruder index.
		unsigned int    tool;
	};

    // Structural records taken from the writer's positive extrusions, for the tower structure audit.
    enum class StructuralRole : unsigned char {
        None,
        TowerPerimeter,
        TowerBlock,
        IntentionalBridge,
        // Block-local purge or ramming roads: support evidence only, never a perimeter record.
        InteriorDeposit
    };
    struct StructuralEmission
    {
        float z{0.f};
        float height{0.f};
        int logical_tool{-1};
        StructuralRole role{StructuralRole::None};
        Vec2f footprint_min{Vec2f::Zero()};
        Vec2f footprint_max{Vec2f::Zero()};
        unsigned int support_domain{0};
        // Ramming held above its level to meet the outgoing tool's minimum; audited as a road but
        // not the top of its support chain.
        bool held_above_level{false};
    };

    struct NozzleChangeResult
    {
        std::string gcode;
        float elapsed_time { 0.f };

        Vec2f start_pos;  // rotated
        Vec2f end_pos;

		Vec2f origin_start_pos;  // not rotated

        std::vector<Vec2f> wipe_path;
        bool is_extruder_change{true};
        std::vector<StructuralEmission> structural_emissions;
    };

	struct ToolChangeResult
	{
		// Print heigh of this tool change.
		// Scalars are initialised: a result for a level that laid nothing is still summed.
		float					print_z { 0.f };
		float 					layer_height { 0.f };
		// Tower top when this visit began, where the head must be before the first road. Equals
		// print_z unless the tower lags.
		float					tower_z_start { 0.f };
		// G-code section to be directly included into the output G-code.
		std::string				gcode;
		// For path preview.
		std::vector<Extrusion> 	extrusions;
        std::vector<StructuralEmission> structural_emissions;
		// Initial position, at which the wipe tower starts its action.
		// At this position the extruder is loaded and there is no Z-hop applied.
		Vec2f						start_pos { Vec2f::Zero() };
		// Last point, at which the normal G-code generator of Slic3r shall continue.
		// At this position the extruder is loaded and there is no Z-hop applied.
		Vec2f						end_pos { Vec2f::Zero() };
		// Time elapsed over this tool change.
		// This is useful not only for the print time estimation, but also for the control of layer cooling.
		float  				    elapsed_time { 0.f };

        // Is this a priming extrusion? (If so, the wipe tower rotation & translation will not be applied later)
        bool                    priming { false };

		bool                    is_tool_change{false};
        // Applied by the integration writer after every merged tower road, with balanced restoration.
        float                   departure_retract_extra { 0.f };
        // A purge row from the middle of this visit, set with departure_retract_extra; the fine
        // nozzle wipes along it before leaving the tower.
        std::vector<Vec2f>      departure_wipe_path;
		Vec2f                   tool_change_start_pos { Vec2f::Zero() };

        // Pass a polyline so that normal G-code generator can do a wipe for us.
        // The wipe cannot be done by the wipe tower because it has to pass back
        // a loaded extruder, so it would have to either do a wipe with no retraction
        // (leading to https://github.com/prusa3d/PrusaSlicer/issues/2834) or do
        // an extra retraction-unretraction pair.
        std::vector<Vec2f> wipe_path;

		// BBS
        float purge_volume = 0.f;

        // Initial tool
        int initial_tool { -1 };

        // New tool
        int new_tool { -1 };

        // BBS: in bbl filament_change_gcode, toolhead will be moved to the wipe tower automatically.
        // But if finish_layer_tcr is before tool_change_tcr, we have to travel to the wipe tower before
        // executing the gcode finish_layer_tcr.
        bool is_finish_first = false;

        bool               is_contact = false;
        NozzleChangeResult nozzle_change_result;

		// Sum the total length of the extrusion.
		float total_extrusion_length_in_plane() {
			float e_length = 0.f;
			for (size_t i = 1; i < this->extrusions.size(); ++ i) {
				const Extrusion &e = this->extrusions[i];
				if (e.width > 0) {
					Vec2f v = e.pos - (&e - 1)->pos;
					e_length += v.norm();
				}
			}
			return e_length;
		}
		// Orca: set by WipeTower2 (non-BBL tower) to force a travel to the tower even when the
		// previous position is unknown; read by WipeTowerIntegration::append_tcr2 (GCode.cpp).
		bool force_travel = false;
	};

    struct box_coordinates
    {
        box_coordinates(float left, float bottom, float width, float height) :
            ld(left        , bottom         ),
            lu(left        , bottom + height),
            rd(left + width, bottom         ),
            ru(left + width, bottom + height) {}
        box_coordinates(const Vec2f &pos, float width, float height) : box_coordinates(pos(0), pos(1), width, height) {}
        void translate(const Vec2f &shift) {
            ld += shift; lu += shift;
            rd += shift; ru += shift;
        }
        void translate(const float dx, const float dy) { translate(Vec2f(dx, dy)); }
        void expand(const float offset) {
            ld += Vec2f(- offset, - offset);
            lu += Vec2f(- offset,   offset);
            rd += Vec2f(  offset, - offset);
            ru += Vec2f(  offset,   offset);
        }
        void expand(const float offset_x, const float offset_y) {
            ld += Vec2f(- offset_x, - offset_y);
            lu += Vec2f(- offset_x,   offset_y);
            rd += Vec2f(  offset_x, - offset_y);
            ru += Vec2f(  offset_x,   offset_y);
        }
        Vec2f ld;  // left down
        Vec2f lu;	// left upper
        Vec2f rd;	// right lower
        Vec2f ru;  // right upper
    };

    // Construct ToolChangeResult from current state of WipeTower and WipeTowerWriter.
    // WipeTowerWriter is moved from !
    ToolChangeResult construct_tcr(WipeTowerWriter& writer,
                                   bool priming,
                                   size_t old_tool,
                                   bool is_finish,
                                   bool is_tool_change, float purge_volume, bool is_contact,
                                   StructuralRole structural_role = StructuralRole::None) const;

    ToolChangeResult construct_block_tcr(WipeTowerWriter& writer,
                                   bool priming,
                                   size_t filament_id,
                                   bool is_finish, float purge_volume,
                                   StructuralRole structural_role = StructuralRole::None,
                                   float structural_height = -1.f,
                                   unsigned int support_domain = 0) const;


	// x			-- x coordinates of wipe tower in mm ( left bottom corner )
	// y			-- y coordinates of wipe tower in mm ( left bottom corner )
	// width		-- width of wipe tower in mm ( default 60 mm - leave as it is )
	// wipe_area	-- space available for one toolchange in mm
	// BBS: add partplate logic
	WipeTower(const PrintConfig& config, int plate_idx, Vec3d plate_origin, size_t initial_tool,
              const float wipe_tower_height, const std::vector<unsigned int>& slice_used_filaments,
              float nominal_width_override = -1.f);


	// Set the extruder properties.
    void set_extruder(size_t idx, const PrintConfig& config);

    void set_shared_print_bed(const Polygons &bed) { m_shared_print_bed = bed; }
    // Orca: has_filament_switcher is not a static PrintConfig member here, so it is pushed in from
    // Print via a setter rather than read in the ctor. Device-set only.
    void set_has_filament_switcher(bool v) { m_has_filament_switcher = v; }
	// Appends into internal structure m_plan containing info about the future wipe tower
	// to be used before building begins. The entries must be added ordered in z.
    // force_emit marks a compact-tower filler layer: no real switch, but the tower must still
    // deposit here to keep the gap within every tool's max layer height.
    // ram_length_override_mm >= 0 replaces the vendor filament_change_length of the outgoing tool
    // for both planning and the emission budget; -1 keeps the vendor value.
    void plan_toolchange(float z_par, float layer_height_par, unsigned int old_tool, unsigned int new_tool, float wipe_volume_ec = 0.f, float wipe_volume_nc = 0.f, float prime_volume = 0.f, bool force_emit = false, float ram_length_override_mm = -1.f);


	// Iterates through prepared m_plan, generates ToolChangeResults and appends them to "result"
	void generate(std::vector<std::vector<ToolChangeResult>> &result);

	WipeTower::ToolChangeResult only_generate_out_wall(bool is_new_mode = false);
    Polygon generate_support_wall(WipeTowerWriter &writer, const box_coordinates &wt_box, double feedrate, bool first_layer);
    Polygon generate_support_wall_new(WipeTowerWriter &writer, const box_coordinates &wt_box, double feedrate, bool first_layer,bool rib_wall, bool extrude_perimeter, bool skip_points);

    Polygon generate_rib_polygon(const box_coordinates &wt_box);
    float get_depth() const { return m_wipe_tower_depth; }
    float get_brim_width() const { return m_wipe_tower_brim_width_real; }
    BoundingBoxf get_bbx() const {
        if (m_outer_wall.empty()) return BoundingBoxf({Vec2d(0,0)});
        BoundingBox  box = get_extents(m_outer_wall.begin()->second);
        BoundingBoxf res = BoundingBoxf(unscale(box.min), unscale(box.max));
        return res;
    }
    std::map<float, Polylines> get_outer_wall() const
    {
        return m_outer_wall;
    }
    float get_height() const { return m_wipe_tower_height; }
    float get_layer_height() const { return m_layer_height; }
    float get_rib_length() const { return m_rib_length; }
    float get_rib_width() const { return m_rib_width; }

	void set_last_layer_extruder_fill(bool extruder_fill) {
        if (!m_plan.empty()) {
			m_plan.back().extruder_fill = extruder_fill;
		}
	}


	// Switch to a next layer.
	void set_layer(
		// Print height of this layer.
		float print_z,
		// Layer height, used to calculate extrusion the rate.
		float layer_height,
		// Maximum number of tool changes on this layer or the layers below.
		size_t max_tool_changes,
		// Is this the first layer of the print? In that case print the brim first.
		bool is_first_layer,
		// Is this the last layer of the waste tower?
		bool is_last_layer)
	{
		m_z_pos 				= print_z;
		// The part layer this level belongs to; generate_new() overrides it for a lagging level.
		m_part_z_pos			= print_z;
		m_layer_height			= layer_height;
		m_depth_traversed  = 0.f;
        m_current_layer_finished = false;
		//m_current_shape = (! is_first_layer && m_current_shape == SHAPE_NORMAL) ? SHAPE_REVERSED : SHAPE_NORMAL;
		m_current_shape = SHAPE_NORMAL;
		if (is_first_layer) {
            m_num_layer_changes = 0;
            m_num_tool_changes 	= 0;
        } else
            ++ m_num_layer_changes;

		// Calculate extrusion flow from desired line width, nozzle diameter, filament diameter and layer_height:
		m_extrusion_flow = extrusion_flow(layer_height);
        // Advance m_layer_info iterator, making sure we got it right
		while (!m_plan.empty() && m_layer_info->z < print_z - WT_EPSILON && m_layer_info+1 != m_plan.end())
			++m_layer_info;
	}

	// Return the wipe tower position.
	const Vec2f& 		 position() const { return m_wipe_tower_pos; }
	// Return the wipe tower width.
	float     		 width()    const { return m_wipe_tower_width; }
    void set_preserve_planned_width(bool preserve) { m_preserve_planned_width = preserve; }
    // Used by Print for a Body Split plate on the lagging tower; call before plan_toolchange().
    void set_no_sparse_layers(bool no_sparse) { m_no_sparse_layers = no_sparse; }
	// The wipe tower is finished, there should be no more tool changes or wipe tower prints.
	bool 	  		 finished() const { return m_max_color_changes == 0; }

	// Diagnostic: the plan as received and as generated (after plan_lagging_tower() and
	// plan_tower_new() rewrite it). Empty until generate_new() has run.
	std::string          plan_digest() const;
	// The planning half of generate_new(): rewrites the plan without laying anything.
	void                 plan_levels();

	// Returns gcode to prime the nozzles at the front edge of the print bed.
	std::vector<ToolChangeResult> prime(
		// print_z of the first layer.
		float 						initial_layer_print_height,
		// Extruder indices, in the order to be primed. The last extruder will later print the wipe tower brim, print brim and the object.
		const std::vector<unsigned int> &tools,
		// If true, the last priming are will be the same as the other priming areas, and the rest of the wipe will be performed inside the wipe tower.
		// If false, the last priming are will be large enough to wipe the last extruder sufficiently.
		bool 						last_wipe_inside_wipe_tower);

	// Returns gcode for a toolchange and a final print head position.
	// On the first layer, extrude a brim around the future wipe tower first.
	// BBS
    ToolChangeResult tool_change(size_t new_tool, bool extrude_perimeter = false, bool first_toolchange_to_nonsoluble = false);

	NozzleChangeResult nozzle_change(int old_filament_id, int new_filament_id);

	// Fill the unfilled space with a sparse infill.
	// Call this method only if layer_finished() is false.
    ToolChangeResult finish_layer(bool extruder_perimeter = true, bool extruder_fill = true);

	// Calculates extrusion flow needed to produce required line width for given layer height
    float extrusion_flow(float layer_height = -1.f) const // negative layer_height - return current m_extrusion_flow
    {
        if (layer_height < 0) return m_extrusion_flow;
        return layer_height * (m_perimeter_width - layer_height * (1.f - float(M_PI) / 4.f)) / filament_area();
    }
    float nozzle_change_extrusion_flow(float layer_height = -1.f) const // negative layer_height - return current m_extrusion_flow
    {
        if (layer_height < 0)
            return m_extrusion_flow;
        return layer_height * (m_nozzle_change_perimeter_width - layer_height * (1.f - float(M_PI) / 4.f)) / filament_area();
    }
    // Purge rows are drawn by the incoming tool alone, so in mixed-nozzle modes they use that
    // tool's own width. Off mode, or an out-of-range tool, uses the shared m_perimeter_width.
    float purge_width(int tool) const
    {
        if (! m_mixed_nozzle_slicing)
            return m_perimeter_width;
        return (tool >= 0 && size_t(tool) < m_filpar.size()) ? m_filpar[tool].nozzle_diameter * Width_To_Nozzle_Ratio : m_perimeter_width;
    }
    // extrusion_flow() at the incoming tool's purge_width(); the inverse of volume_to_length() at
    // the same width.
    float purge_extrusion_flow(int tool, float layer_height = -1.f) const
    {
        if (layer_height < 0)
            return m_extrusion_flow;
        const float w = purge_width(tool);
        return layer_height * (w - layer_height * (1.f - float(M_PI) / 4.f)) / filament_area();
    }

    // The wall and sparse grid use the smallest nozzle's width, which a much larger nozzle draws as
    // a thin strand. With this on, a tool wider than the shared width lays those roads at its own
    // width, the first layer included: a coarse tool that lays the tower's bed level puts its wall
    // down at its own width too. Paths, spacing, the brim and solid fills are unchanged.
    static constexpr bool solid_structural_roads_per_tool = true;
    // The flow a structural road of this tool is laid at, given the flow the caller would use.
    float structural_extrusion_flow(int tool, float shared_flow) const
    {
        if (! solid_structural_roads_per_tool || ! m_mixed_nozzle_slicing)
            return shared_flow;
        const float w = purge_width(tool);
        if (! (w > m_perimeter_width + 1e-4f))
            return shared_flow;
        return m_layer_height * (w - m_layer_height * (1.f - float(M_PI) / 4.f)) / filament_area();
    }
    float structural_road_width(int tool) const
    {
        if (! solid_structural_roads_per_tool || ! m_mixed_nozzle_slicing)
            return m_perimeter_width;
        return std::max(m_perimeter_width, purge_width(tool));
    }

	bool get_floating_area(float& start_pos_y, float& end_pos_y) const;
	bool need_thick_bridge_flow(float pos_y) const;
    float get_extrusion_flow() const { return m_extrusion_flow; }

	// Is the current layer finished?
	bool 			 layer_finished() const {
        return m_current_layer_finished;
	}

    std::vector<float> get_used_filament() const { return m_used_filament_length; }
    int get_number_of_toolchanges() const { return m_num_tool_changes; }

	void set_has_tpu_filament(bool has_tpu) { m_has_tpu_filament = has_tpu; }

    bool has_tpu_filament() const { return m_has_tpu_filament; }
    struct FilamentParameters {
        std::string 	    material = "PLA";
        int                 category;
        bool                is_soluble = false;
        // BBS
        bool                is_support = false;
        int  			    nozzle_temperature = 0;
        int  			    nozzle_temperature_initial_layer = 0;
        // BBS: remove useless config
        //float               loading_speed = 0.f;
        //float               loading_speed_start = 0.f;
        //float               unloading_speed = 0.f;
        //float               unloading_speed_start = 0.f;
        //float               delay = 0.f ;
        //int                 cooling_moves = 0;
        //float               cooling_initial_speed = 0.f;
        //float               cooling_final_speed = 0.f;
        float               ramming_line_width_multiplicator = 1.f;
        float               ramming_step_multiplicator = 1.f;
        float               max_e_speed = std::numeric_limits<float>::max();
        std::vector<float>  ramming_speed;
        float               nozzle_diameter;
        // Max road height for this filament's nozzle, infinity when unresolved.
        float               max_layer_height { std::numeric_limits<float>::infinity() };
        // Min road height for this filament's nozzle, 0 when unknown or not mixed-nozzle.
        float               min_layer_height { 0.f };
        float               filament_area;
        float               retract_length;
        float               retract_speed;
        float               wipe_dist;
        std::pair<float,float>  max_e_ramming_speed;//[0]extruder change [1]nozzle change
        std::pair<float, float> ramming_travel_time; // Travel time after ramming
        std::pair<std::vector<float>,std::vector<float>>  precool_t;//Pre-cooling time, set to 0 to ensure the ramming speed is controlled solely by ramming volumetric speed.
        std::pair<std::vector<float>, std::vector<float>> precool_t_first_layer;
        std::pair<int,int>    precool_target_temp;
        float filament_cooling_before_tower = 0.f;
        float flat_iron_area;
        float filament_tower_interface_print_temp;
        float filament_tower_interface_pre_extrusion_dist = 0;
        float filament_tower_interface_pre_extrusion_length = 0;
        float filament_petg_pre_extrusion_offset_dist = 0;
    };


    void set_used_filament_ids(const std::vector<int> &used_filament_ids) { m_used_filament_ids = used_filament_ids; };
    void set_filament_categories(const std::vector<int> & filament_categories) { m_filament_categories = filament_categories;};
    void set_nozzle_group_result(const MultiNozzleUtils::LayeredNozzleGroupResult &multi_nozzle_group_result) { m_multi_nozzle_group_result = &multi_nozzle_group_result; };
    std::vector<int> m_used_filament_ids;
    std::vector<int> m_filament_categories;
    const MultiNozzleUtils::LayeredNozzleGroupResult *m_multi_nozzle_group_result{nullptr};

    enum class WipeTowerLayerType : unsigned char { Normal, Contact, Solid, Contact_UP};// Contact layer should be solid and reduce feed

	struct WipeTowerBlock
    {
        int              block_id{0};
        int              filament_adhesiveness_category{0};
        std::vector<float>      layer_depths;
        //std::vector<bool>       solid_infill;
        std::vector<float>      finish_depth{0}; // the start pos of finish frame for every layer
        std::vector<WipeTowerLayerType> layers_type;     // type of the layer, normal, Contact or Solid
        float            depth{0};
        float            start_depth{0};
        float            cur_depth{0};
        int              last_filament_change_id{-1};
        int              last_nozzle_change_id{-1};
	};

	struct BlockDepthInfo
    {
        int category{-1};
        float depth{0};
        float nozzle_change_depth{0};
	};

	std::vector<std::vector<BlockDepthInfo>> m_all_layers_depth;
	std::vector<WipeTowerBlock> m_wipe_tower_blocks;
    int                  m_last_block_id;
    WipeTowerBlock*      m_cur_block{nullptr};

	// help function
    WipeTowerBlock* get_block_by_category(int filament_adhesiveness_category, bool create);
    void add_depth_to_block(int filament_id, int filament_adhesiveness_category, float depth, bool is_nozzle_change = false);
	int get_filament_category(int filament_id);
	void reset_block_status();
    int get_wall_filament_for_all_layer();
	// for generate new wipe tower
    void generate_new(std::vector<std::vector<WipeTower::ToolChangeResult>> &result);

    // Enable the lagging schedule. coarse_tool is a representative coarse filament, the two
    // heights are the cadences, lag_max the allowed lag. filament_extruders maps each filament to
    // its physical extruder and coarse_extruder is the widest nozzle's. Without a coarse tool or
    // cadences the tower keeps pace.
    void set_lagging(float lag_max, int coarse_tool, std::vector<int> filament_extruders,
                     int coarse_extruder, float coarse_layer_height, float fine_layer_height);
    // Whether this filament is on the nozzle that builds the tower (falls back to coarse_tool
    // without a map).
    bool on_coarse_nozzle(int filament_id) const
    {
        if (m_lag_coarse_extruder >= 0 && filament_id >= 0 &&
            size_t(filament_id) < m_lag_filament_extruders.size())
            return m_lag_filament_extruders[size_t(filament_id)] == m_lag_coarse_extruder;
        return filament_id == m_lag_coarse_tool;
    }
    bool lagging() const { return m_lag_coarse_tool >= 0 && m_lag_coarse_height > 0.f && m_lag_fine_height > 0.f; }

	void plan_tower_new();
	void generate_wipe_tower_blocks(bool add_solid_flag);
    void update_all_layer_depth(float wipe_tower_depth);
    void set_nozzle_last_layer_id();
    void set_first_layer_flow_ratio(const float flow_ratio);
    // Orca: default/initial-layer/travel acceleration are object-scope options here (PrintConfig
    // members in BBS), so Print pushes the resolved per-variant columns in via this setter.
    void set_accelerations(const std::vector<double> &normal, const std::vector<double> &first_layer_normal,
                           const std::vector<double> &travel, const std::vector<double> &first_layer_travel);
    void calc_block_infill_gap();
    ToolChangeResult   tool_change_new(size_t new_tool, bool solid_change = false, bool solid_nozzlechange=false);
    NozzleChangeResult ramming(int old_filament_id, int new_filament_id, bool solid_change = false, bool extruder_change = true); // extruder_chang means nozzle_change
    ToolChangeResult   finish_layer_new(bool extrude_perimeter = true, bool extrude_fill = true, bool extrude_fill_wall = true);
    ToolChangeResult   finish_block(const WipeTowerBlock &block, int filament_id, bool extrude_fill = true);
    ToolChangeResult   finish_block_solid(const WipeTowerBlock &block, int filament_id, bool extrude_fill = true, WipeTowerLayerType layer_type = WipeTowerLayerType::Normal);
    // wipe_volume_budget: requested prime volume (mm^3), 0 for none; emission stops once it is
    // deposited. is_nozzle_change skips the colour-flush ironing scrub (the tools share no filament).
    void toolchange_wipe_new(WipeTowerWriter &writer, const box_coordinates &cleaning_box, float wipe_length,bool solid_toolchange=false, float wipe_volume_budget=0.f, bool is_nozzle_change=false);
    Vec2f              get_rib_offset() const { return m_rib_offset; }
    // Whether this tool may lay the 0.2 mm bridge road; unknown tools may.
    bool               tool_can_thicken(int tool) const
    {
        if (tool < 0 || size_t(tool) >= m_filpar.size())
            return true;
        return m_filpar[tool].max_layer_height + 1e-4f >= 0.2f;
    }
    // Ramming height for the outgoing tool: level_height clamped to its nozzle limits (mixed-nozzle only).
    float              ramming_height(int tool, float level_height) const
    {
        if (!m_mixed_nozzle_slicing || tool < 0 || size_t(tool) >= m_filpar.size())
            return level_height;
        float height = level_height;
        const float lowest  = m_filpar[tool].min_layer_height;
        const float highest = m_filpar[tool].max_layer_height;
        if (lowest > 0.f && height < lowest - 1e-4f)
            height = lowest;
        if (std::isfinite(highest) && highest > 0.f && height > highest + 1e-4f)
            height = highest;
        return height;
    }
    bool               is_need_ramming(int filament_id_1, int filament_id_2, int layer_id) const;
    bool               is_same_extruder(int filament_id_1, int filament_id_2, int layer_id) const;
    bool               is_same_nozzle(int filament_id_1, int filament_id_2, int layer_id) const;
    int                get_nozzle_id(int filament_id, int layer_id) const;
    int                get_extruder_id(int filament_id, int layer_id) const;

private:
	enum wipe_shape // A fill-in direction
	{
		SHAPE_NORMAL = 1,
		SHAPE_REVERSED = -1
	};

    const float Width_To_Nozzle_Ratio = 1.25f; // desired line width (oval) in multiples of nozzle diameter - may not be actually neccessary to adjust
    const float WT_EPSILON            = 1e-3f;
    float filament_area() const {
        return m_filpar[0].filament_area; // all extruders are assumed to have the same filament diameter at this point
    }

    int    m_slice_used_filaments      = 0;
    int    m_wrapping_detection_layers = 0;
    bool   m_enable_wrapping_detection = false;
	bool   m_enable_timelapse_print = false;
	bool   m_semm               = true; // Are we using a single extruder multimaterial printer?
    Vec2f  m_wipe_tower_pos; 			// Left front corner of the wipe tower in mm.
	bool   m_preserve_planned_width { false };
	float  m_wipe_tower_width; 			// Width of the wipe tower.
	float  m_wipe_tower_depth 	= 0.f; 	// Depth of the wipe tower
	// BBS
	float  m_wipe_tower_height = 0.f;
    float  m_wipe_tower_brim_width      = 0.f; 	// Width of brim (mm) from config
    float  m_wipe_tower_brim_width_real = 0.f; 	// Width of brim (mm) after generation
    bool   m_mixed_nozzle_slicing       = false;
	float  m_wipe_tower_rotation_angle = 0.f; // Wipe tower rotation angle in degrees (with respect to x axis)
    float  m_internal_rotation  = 0.f;
	float  m_y_shift			= 0.f;  // y shift passed to writer
	float  m_z_pos 				= 0.f;  // Current Z position.
	float  m_layer_height 		= 0.f; 	// Current layer height.
	size_t m_max_color_changes 	= 0; 	// Maximum number of color changes per layer.
    int    m_old_temperature    = -1;   // To keep track of what was the last temp that we set (so we don't issue the command when not neccessary)
    float  m_travel_speed       = 0.f;
    float  m_first_layer_speed  = 0.f;
    size_t m_first_layer_idx    = size_t(-1);
    // Highest Y the last tool change's purge rows reached, in the block's frame; -1 before any.
    float  m_purge_rows_top_y   = -1.f;
    // Top of the solid base, recomputed by plan_tower_new() once the plan is final.
    float  m_tower_base_top_z   = 0.f;
    // True while generating a lagging catch-up layer, so each block emits its own Z.
    bool   m_lag_emit_block_z   = false;
    // Lagging schedule inputs, and the part layer of the level being generated.
    float  m_lag_max            = 0.f;
    int    m_lag_coarse_tool    = -1;
    std::vector<int> m_lag_filament_extruders;
    int    m_lag_coarse_extruder = -1;
    float  m_lag_coarse_height  = 0.f;
    float  m_lag_fine_height    = 0.f;
    float  m_part_z_pos         = 0.f;
    void   plan_lagging_tower();
    // Last temperature requested per filament, so the base emits one M104 per change of target.
    std::vector<int> m_tower_temperature_held;
    void   update_tower_base_extent();
    Vec2f            m_origin;
    std::vector<int>    m_last_layer_id;
    std::pair<std::vector<double>,std::vector<double>> m_filaments_change_length;//[0]extruder change [1]nozzle change
    size_t       m_cur_layer_id;
    NozzleChangeResult m_nozzle_change_result;
    bool               m_has_tpu_filament{false};
    bool               m_is_multi_extruder{false};
    bool               m_use_gap_wall{false};
    bool               m_use_rib_wall{false};
    float              m_rib_length=0.f;
    float              m_rib_width=0.f;
    float              m_extra_rib_length=0.f;
    bool               m_used_fillet{false};
    Vec2f              m_rib_offset{Vec2f(0.f, 0.f)};
    bool               m_tower_framework{false};
    bool               m_need_reverse_travel{false};
    bool               m_enable_tower_interface_features{false};
	// G-code generator parameters.
    // BBS: remove useless config
    //float           m_cooling_tube_retraction   = 0.f;
    //float           m_cooling_tube_length       = 0.f;
    //float           m_parking_pos_retraction    = 0.f;
    //float           m_extra_loading_move        = 0.f;
    float           m_bridging                  = 0.f;
    bool            m_no_sparse_layers          = false;
    // BBS: remove useless config
    //bool            m_set_extruder_trimpot      = false;
    bool            m_adhesion                  = true;
    GCodeFlavor     m_gcode_flavor;
    bool                      m_is_multiple_nozzle = false;
    std::vector<unsigned int> m_normal_accels;
    std::vector<unsigned int> m_first_layer_normal_accels;
    std::vector<unsigned int> m_travel_accels;
    std::vector<unsigned int> m_first_layer_travel_accels;
    unsigned int              m_max_accels;
    bool                      m_accel_to_decel_enable;
    float                     m_accel_to_decel_factor;
    bool                      m_enable_arc_fitting = true;
    std::vector<double>       m_hotend_heating_rate;
    std::vector<double>       m_hotend_cooling_rate;
    Polygons                  m_shared_print_bed;

    // Bed properties
    enum {
        RectangularBed,
        CircularBed,
        CustomBed
    } m_bed_shape;
    float m_bed_width; // width of the bed bounding box
    Vec2f m_bed_bottom_left; // bottom-left corner coordinates (for rectangular beds)

    float m_first_layer_flow_ratio;
	float m_perimeter_width = 0.4f * Width_To_Nozzle_Ratio; // Width of an extrusion line, also a perimeter spacing for 100% infill.
    float m_nozzle_change_perimeter_width = 0.4f * Width_To_Nozzle_Ratio;
	float m_extrusion_flow = 0.038f; //0.029f;// Extrusion flow is derived from m_perimeter_width, layer height and filament diameter.
    std::unordered_map<int, std::pair<float,float>> m_block_infill_gap_width; // categories to infill_gap: toolchange gap, nozzlechange gap
	// Extruder specific parameters.
    std::vector<FilamentParameters> m_filpar;


	// State of the wipe tower generator.
	unsigned int m_num_layer_changes = 0; // Layer change counter for the output statistics.
	unsigned int m_num_tool_changes  = 0; // Tool change change counter for the output statistics.
	///unsigned int 	m_idx_tool_change_in_layer = 0; // Layer change counter in this layer. Counting up to m_max_color_changes.
	bool m_print_brim = true;
	// A fill-in direction (positive Y, negative Y) alternates with each layer.
	wipe_shape   	m_current_shape = SHAPE_NORMAL;
    size_t 	m_current_tool  = 0;
	// BBS
    //const std::vector<std::vector<float>> wipe_volumes;

	float           m_depth_traversed = 0.f; // Current y position at the wipe tower.
    bool            m_current_layer_finished = false;
	bool 			m_left_to_right   = true;
	float			m_extra_spacing   = 1.f;
	float           m_tpu_fixed_spacing = 2;
    float           m_max_speed = 5400.f;  // the maximum printing speed on the prime tower.
    std::vector<std::vector<Vec2f>> m_wall_skip_points;
    std::map<float,Polylines> m_outer_wall;
    std::vector<double>        m_printable_height;
    bool is_first_layer() const { return size_t(m_layer_info - m_plan.begin()) == m_first_layer_idx; }
    bool                       is_valid_last_layer(int tool, int layer_id, double layer_z) const;
    bool                       m_flat_ironing=false;
    bool                       m_contact_ironing = false;
    bool                       m_has_filament_switcher = false;
    float                      m_contact_speed   = 20 * 60.f;
    std::vector<int>           m_physical_extruder_map;
	// Calculates length of extrusion line to extrude given volume
	float volume_to_length(float volume, float line_width, float layer_height) const {
		return std::max(0.f, volume / (layer_height * (line_width - layer_height * (1.f - float(M_PI) / 4.f))));
	}
    // Calculates volume of extrusion line
    float length_to_volume(float length,float line_width, float layer_height) const
    {
        return std::max(0.f, length * (layer_height * (line_width - layer_height * (1.f - float(M_PI) / 4.f))));
    }
	// Calculates depth for all layers and propagates them downwards
	void plan_tower();

	// Goes through m_plan and recalculates depths and width of the WT to make it exactly square - experimental
	void make_wipe_tower_square();

	Vec2f get_next_pos(const WipeTower::box_coordinates &cleaning_box, float wipe_length, bool solid_toolchange);

    // Goes through m_plan, calculates border and finish_layer extrusions and subtracts them from last wipe
    void save_on_last_wipe();

	bool is_tpu_filament(int filament_id) const;
	bool is_petg_filament(int filament_id) const;
    bool is_need_reverse_travel(int filament, bool extruder_change) const;
	// BBS
	box_coordinates align_perimeter(const box_coordinates& perimeter_box);

    void set_for_wipe_tower_writer(WipeTowerWriter &writer);

    // to store information about tool changes for a given layer
	struct WipeTowerInfo{
		struct ToolChange {
            size_t old_tool;
            size_t new_tool;
			float required_depth;
            float ramming_depth;
            float first_wipe_line;
            float wipe_volume;
			float wipe_length;
            // Requested purge volume; wipe_volume may be quantised to whole rows.
            float wipe_volume_budget{0};
            float nozzle_change_depth{0};
            float nozzle_change_length{0};
            // Ramming budget (mm of filament) the block was planned against; emission stops there.
            float nozzle_change_budget{0};
            // Resolved ram length for the outgoing tool, or -1 for the vendor value. Kept so that
            // re-deriving the block in set_toolchange() does not drop it.
            float ram_length_override_mm{-1.f};
			// BBS
			float purge_volume;
            ToolChange(size_t old, size_t newtool, float depth=0.f, float ramming_depth=0.f, float fwl=0.f, float wv=0.f, float wl = 0, float pv = 0)
				: old_tool{ old }, new_tool{ newtool }, required_depth{ depth }, ramming_depth{ ramming_depth }, first_wipe_line{ fwl }, wipe_volume{ wv }, wipe_length{ wl }, wipe_volume_budget{ wv }, purge_volume{ pv } {}
		};
		float z;		// z position of the layer
		float height;	// layer height
		float depth;	// depth of the layer based on all layers above
		float extra_spacing;
        bool  extruder_fill{true};
        // Lagging tower: z is the tower height, part_z the part layer the level belongs to.
        // wall_owner draws everything on the level; prime_only is a fine return (prime road only);
        // catch_up joins the visit below; idle is a switchless level that is not visited.
        float part_z{0.f};
        int   wall_owner{-1};
        bool  prime_only{false};
        // A prime-only level on the bed that is the tower's first layer: it still lays the whole
        // solid bed footprint (brim, wall and block fill), so the tower top rises to it.
        bool  bed_fill{false};
        bool  catch_up{false};
        bool  idle{false};
        // A later run of the same part layer's tool changes on the other nozzle; its results join
        // the previous level's visit.
        bool  joins_visit{false};
        // Compact-tower filler layer: deposit (and count) its wall although it has no tool change.
        bool  force_emit{false};
		float toolchanges_depth() const { float sum = 0.f; for (const auto &a : tool_changes) sum += a.required_depth; return sum; }

		std::vector<ToolChange> tool_changes;

		WipeTowerInfo(float z_par, float layer_height_par)
			: z{z_par}, height{layer_height_par}, depth{0}, extra_spacing{1.f} {}
	};

	std::vector<WipeTowerInfo> m_plan; 	// Stores information about all layers and toolchanges for the future wipe tower (filled by plan_toolchange(...))
	std::vector<WipeTowerInfo>::iterator m_layer_info = m_plan.end();

	// See plan_digest().
	std::string          format_plan() const;
	std::string          m_plan_as_received;

    // Stores information about used filament length per extruder:
    std::vector<float> m_used_filament_length;

    // BBS: consider both soluable and support properties
    // Return index of first toolchange that switches to non-soluble extruder
    // ot -1 if there is no such toolchange.
    int first_toolchange_to_nonsoluble_nonsupport(
            const std::vector<WipeTowerInfo::ToolChange>& tool_changes) const;
	WipeTowerInfo::ToolChange set_toolchange(int old_tool, int new_tool, float layer_height, float wipe_volume, float purge_volume, int layer_id, float ram_length_override_mm = -1.f, float wipe_volume_budget = -1.f);
	void toolchange_Unload(
		WipeTowerWriter &writer,
		const box_coordinates  &cleaning_box,
		const std::string&	 	current_material,
		const int 				new_temperature);

	void toolchange_Change(
		WipeTowerWriter &writer,
        const size_t		new_tool,
		const std::string& 		new_material);

	void toolchange_Load(
		WipeTowerWriter &writer,
		const box_coordinates  &cleaning_box);

	void toolchange_Wipe(
		WipeTowerWriter &writer,
		const box_coordinates  &cleaning_box,
		float wipe_volume);
    void get_wall_skip_points(const WipeTowerInfo &layer,int layer_id);
    void get_all_wall_skip_points();
    ToolChangeResult merge_tcr(ToolChangeResult &first, ToolChangeResult &second);
    float            get_block_gap_width(int tool, bool is_nozzlechangle = false);
};




} // namespace Slic3r

#endif // WipeTowerPrusaMM_hpp_
