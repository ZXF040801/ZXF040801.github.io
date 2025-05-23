import cv2
import numpy as np
import yaml
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial.distance import cdist

# --- Configuration ---
MAP_YAML_PATH = "/home/jetson/robot-2/src/slam/map/cmap3.yaml"
ROBOT_WIDTH_d = 0.9  # meters, for obstacle inflation
EFFECTIVE_COVERAGE_WIDTH_D = 0.5  # meters, for zigzag spacing
START_PHYSICAL_POS = (0.2, 0.0)  # meters, (x, y) - user-specified start point

# --- Helper Functions ---
class MapProcessor:
    def __init__(self, yaml_path):
        with open(yaml_path, 'r') as f:
            self.map_meta = yaml.safe_load(f)

        self.resolution = self.map_meta['resolution']
        self.origin_x = self.map_meta['origin'][0]
        self.origin_y = self.map_meta['origin'][1]
        self.map_image_path = self.map_meta['image']

        self.map_image = cv2.imread(self.map_image_path, cv2.IMREAD_GRAYSCALE)
        if self.map_image is None:
            raise FileNotFoundError(f"Map image not found: {self.map_image_path}")

        self.height_pixels, self.width_pixels = self.map_image.shape
        print(f"Map Info: Path='{self.map_image_path}', Resolution={self.resolution} m/px")
        print(f"          Origin (world coords of map [0,0]px bottom-left): Ox={self.origin_x}m, Oy={self.origin_y}m")
        print(f"          Dimensions (pixels): Width={self.width_pixels}, Height={self.height_pixels}")

        # --- Critical Change: Map Binarization ---
        # Based on user map characteristics: 254 is white (free), 0 is black (obstacle), 205 is gray (unknown)
        self.binary_map = np.zeros_like(self.map_image, dtype=np.uint8)
        # Mark areas with pixel value 254 (white) in the original image as 255 (free) in the binary map
        self.binary_map[self.map_image == 254] = 255
        # All other pixel values (black obstacles 0, gray unknown areas 205) remain 0 (non-passable)
        print("Map binarized based on specific color rules (254->free, others->non-passable).")


    def world_to_map_pixels(self, world_x, world_y):
        map_px = int((world_x - self.origin_x) / self.resolution)
        map_py = self.height_pixels - 1 - int((world_y - self.origin_y) / self.resolution)
        map_px = np.clip(map_px, 0, self.width_pixels - 1)
        map_py = np.clip(map_py, 0, self.height_pixels - 1)
        return map_px, map_py

    def map_pixels_to_world(self, map_px, map_py):
        world_x = self.origin_x + map_px * self.resolution
        world_y = self.origin_y + (self.height_pixels - 1 - map_py) * self.resolution
        return world_x, world_y

    def inflate_map(self, robot_width_m):
        inflation_radius_pixels = int(np.ceil((robot_width_m / 2.0) / self.resolution))
        if inflation_radius_pixels == 0: inflation_radius_pixels = 1
        
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                           (2 * inflation_radius_pixels + 1, 2 * inflation_radius_pixels + 1))
        inverted_map = cv2.bitwise_not(self.binary_map)
        dilated_obstacles = cv2.dilate(inverted_map, kernel, iterations=1)
        self.inflated_map = cv2.bitwise_not(dilated_obstacles)
        return self.inflated_map

    def decompose_into_rectangles(self, processed_map):
        # OpenCV 3.x vs 4.x for findContours return values
        try:
            # OpenCV 4.x
            contours, hierarchy = cv2.findContours(processed_map, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        except ValueError:
            # OpenCV 3.x
            _image_returned, contours, hierarchy = cv2.findContours(processed_map, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        rectangles = []
        min_dim_pixels = EFFECTIVE_COVERAGE_WIDTH_D / self.resolution
        
        for contour in contours:
            if cv2.contourArea(contour) > min_dim_pixels**2 : # Basic area filtering
                x, y, w, h = cv2.boundingRect(contour)
                if w > min_dim_pixels and h > min_dim_pixels:
                    rectangles.append({'x': x, 'y': y, 'w': w, 'h': h, 'covered': False,
                                       'centroid': (x + w // 2, y + h // 2)})
        return rectangles

    def _get_line_pixels(self, p1, p2):
        """Get all pixels on the line segment from p1(x,y) to p2(x,y) (Bresenham)"""
        x0, y0 = p1
        x1, y1 = p2
        pixels = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            pixels.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy: # Step X
                err -= dy
                x0 += sx
            if e2 < dx: # Step Y
                err += dx
                y0 += sy
        return pixels

    def generate_zigzag_path_in_rect(self, rect_coords, coverage_width_m, inflated_map_for_collision_check):
        x_rect, y_rect, w_rect, h_rect = rect_coords['x'], rect_coords['y'], rect_coords['w'], rect_coords['h']
        coverage_pixels = int(coverage_width_m / self.resolution)
        if coverage_pixels == 0: coverage_pixels = 1

        path_pixels_rect = []
        
        # Ensure scanning starts near the top of the rectangle and ends near the bottom
        y_start = y_rect + coverage_pixels // 2 
        y_end_max = y_rect + h_rect - coverage_pixels // 2 # Max Y for the center of the last scan line
        
        if y_start > y_end_max : # Region too narrow
             if h_rect >= coverage_pixels: # At least one line can be drawn
                y_current = y_rect + h_rect // 2
                # Check this single line for collision
                line_pts = self._get_line_pixels((x_rect, y_current), (x_rect + w_rect, y_current))
                current_segment_start = None
                for i_pt, pt_on_line in enumerate(line_pts):
                    is_obs = inflated_map_for_collision_check[pt_on_line[1], pt_on_line[0]] == 0
                    if not is_obs:
                        if current_segment_start is None: current_segment_start = pt_on_line
                        if i_pt == len(line_pts) -1: # end of line
                            path_pixels_rect.append(current_segment_start)
                            path_pixels_rect.append(pt_on_line)
                    else:
                        if current_segment_start is not None:
                            path_pixels_rect.append(current_segment_start)
                            path_pixels_rect.append(line_pts[i_pt-1])
                            current_segment_start = None
                return [self.map_pixels_to_world(px, py) for px, py in path_pixels_rect]
             else: # Region too narrow for even one line
                return []


        y_current = y_start
        sweep_right = True 

        while y_current <= y_end_max:
            p1_current_sweep, p2_current_sweep = None, None
            if sweep_right:
                p1_current_sweep = (x_rect, y_current)
                p2_current_sweep = (x_rect + w_rect -1, y_current) # -1 to stay within rect
            else:
                p1_current_sweep = (x_rect + w_rect -1, y_current)
                p2_current_sweep = (x_rect, y_current)
            
            line_pixels_on_sweep = self._get_line_pixels(p1_current_sweep, p2_current_sweep)
            
            current_segment_start_px = None
            last_valid_point_on_sweep = None

            for i_px, current_px_on_line in enumerate(line_pixels_on_sweep):
                # Clamp pixel coordinates to be within map bounds before checking inflated_map
                clamped_y = np.clip(current_px_on_line[1], 0, self.height_pixels - 1)
                clamped_x = np.clip(current_px_on_line[0], 0, self.width_pixels - 1)

                is_obstacle = (inflated_map_for_collision_check[clamped_y, clamped_x] == 0)

                if not is_obstacle:
                    if current_segment_start_px is None:
                        current_segment_start_px = current_px_on_line
                    last_valid_point_on_sweep = current_px_on_line # Track last good point
                    
                    if i_px == len(line_pixels_on_sweep) - 1: # Reached end of sweep line
                        if current_segment_start_px is not None:
                             path_pixels_rect.append(current_segment_start_px)
                             path_pixels_rect.append(current_px_on_line) 
                else: # Hit obstacle
                    if current_segment_start_px is not None:
                        path_pixels_rect.append(current_segment_start_px)
                        path_pixels_rect.append(line_pixels_on_sweep[i_px-1]) # Point before obstacle
                        current_segment_start_px = None
            
            sweep_right = not sweep_right
            
            next_y_candidate = y_current + coverage_pixels
            if next_y_candidate <= y_end_max:
                if path_pixels_rect and last_valid_point_on_sweep : # If last sweep generated points
                    # Connect to the x-coordinate of the last point of the previous sweep segment
                    connect_to_x = path_pixels_rect[-1][0]
                    path_pixels_rect.append((connect_to_x, next_y_candidate))
                    y_current = next_y_candidate
                elif not path_pixels_rect and last_valid_point_on_sweep is None: # Whole sweep was obstacle
                    y_current = next_y_candidate # Try next line, hoping it's clear
                else: # No valid point from last sweep to connect from, or no path yet
                    y_current = next_y_candidate # Try next line
            else: # Next line would be outside or too close to edge
                break
        
        return [self.map_pixels_to_world(px, py) for px, py in path_pixels_rect]


# --- Main Logic ---
def plan_full_coverage(map_processor, robot_d, coverage_D, start_world_pos):
    print("1. Inflating map...")
    inflated_map = map_processor.inflate_map(robot_d)
    
    print("2. Decomposing map into rectangular regions...")
    print("   Note: Current decomposition finds bounding boxes of external free space contours.")
    print("   It does not split a region due to internal obstacles within that bounding box.")
    print("   Path generation within a region will attempt to avoid internal obstacles.")
    regions = map_processor.decompose_into_rectangles(inflated_map)
    if not regions:
        print("No suitable regions found for coverage.")
        return [], [], inflated_map, map_processor

    print(f"Found {len(regions)} regions.")

    full_path_world = []
    start_map_px, start_map_py = map_processor.world_to_map_pixels(start_world_pos[0], start_world_pos[1])
    
    start_point_valid = (inflated_map[start_map_py, start_map_px] != 0)
    
    if start_point_valid:
        print(f"Specified start position {start_world_pos} (pixels: {start_map_px, start_map_py}) is valid.")
        full_path_world.append(start_world_pos)
        current_pos_for_sort_px = (start_map_px, start_map_py)
    else:
        print(f"WARNING: Specified start position {start_world_pos} (pixels: {start_map_px, start_map_py}) is in an obstacle or inflated zone.")
        print("         The path will start from the entry of the nearest valid region instead.")
        # Initial sort reference will be the invalid start point to find closest region's centroid
        current_pos_for_sort_px = (start_map_px, start_map_py)

    # Determine starting region (closest to current_pos_for_sort_px)
    if not regions: return [], [], inflated_map, map_processor # Should be caught earlier
    
    min_dist_to_region = float('inf')
    current_region_idx = -1
    for i, r in enumerate(regions):
        # Use distance to centroid for initial region selection
        dist = np.sqrt((current_pos_for_sort_px[0] - r['centroid'][0])**2 + 
                       (current_pos_for_sort_px[1] - r['centroid'][1])**2)
        if dist < min_dist_to_region:
            min_dist_to_region = dist
            current_region_idx = i
            
    if current_region_idx == -1: # Should not happen if regions exist
        print("Error: Could not determine a starting region.")
        return [], [], inflated_map, map_processor

    ordered_regions_for_viz = []
    
    print("3. Generating paths for regions...")
    num_regions_to_cover = len(regions)
    
    last_path_endpoint_px = current_pos_for_sort_px # Initialize with start or fallback

    for i in range(num_regions_to_cover):
        regions[current_region_idx]['covered'] = True
        current_region = regions[current_region_idx]
        ordered_regions_for_viz.append(current_region)
        
        print(f"Processing region {i+1}/{num_regions_to_cover} at map pixels (x,y): ({current_region['x']}, {current_region['y']})")
        
        region_path_world = map_processor.generate_zigzag_path_in_rect(current_region, coverage_D, inflated_map)
        
        if not region_path_world:
            print(f"Warning: No path generated for region {current_region_idx}. It might be too small or entirely obstructed.")
        else:
            # If full_path_world is empty AND start_point_valid was FALSE,
            # the first point of this region_path_world becomes the true start of the path.
            if not full_path_world and not start_point_valid:
                 full_path_world.extend(region_path_world)
            elif full_path_world and region_path_world: 
                # Connect last point of full_path_world to first point of region_path_world if different
                # For simplicity, matplotlib will draw the line. Just extend.
                 full_path_world.extend(region_path_world)
            elif not full_path_world and region_path_world: # Start point was valid, path only had it.
                 full_path_world.extend(region_path_world)


            if full_path_world: # Update last_path_endpoint_px from the actual last point added
                last_wp_world = full_path_world[-1]
                last_path_endpoint_px = map_processor.world_to_map_pixels(last_wp_world[0], last_wp_world[1])


        # Find next closest uncovered region's centroid from the end of the last generated path segment
        min_dist = float('inf')
        next_region_idx = -1
        for idx, r_next in enumerate(regions):
            if not r_next['covered']:
                dist = np.sqrt((last_path_endpoint_px[0] - r_next['centroid'][0])**2 + 
                               (last_path_endpoint_px[1] - r_next['centroid'][1])**2)
                if dist < min_dist:
                    min_dist = dist
                    next_region_idx = idx
        
        if next_region_idx == -1:
            break 
        current_region_idx = next_region_idx
        
    return full_path_world, ordered_regions_for_viz, inflated_map, map_processor

# --- Visualization ---
fig, ax = plt.subplots(figsize=(10, 10))
current_point_plot = None
past_points_plot = None
future_points_plot = None
path_lines_plot_list = [] 

def visualize_coverage(path_data_world, regions_for_viz, map_display, map_proc_obj_for_viz):
    global current_point_plot, past_points_plot, future_points_plot, path_lines_plot_list
    # Clear previous artists for re-runs if any, or just clear axis
    ax.clear()
    path_lines_plot_list.clear()


    map_extent = [
        map_proc_obj_for_viz.origin_x,
        map_proc_obj_for_viz.origin_x + map_proc_obj_for_viz.width_pixels * map_proc_obj_for_viz.resolution,
        map_proc_obj_for_viz.origin_y,
        map_proc_obj_for_viz.origin_y + map_proc_obj_for_viz.height_pixels * map_proc_obj_for_viz.resolution
    ]
    # Display original map for context
    ax.imshow(np.flipud(map_proc_obj_for_viz.map_image), cmap='gray', extent=map_extent, origin='lower')
    # Optionally, show inflated map boundaries
    # ax.imshow(np.flipud(map_display), cmap='Reds', alpha=0.3, extent=map_extent, origin='lower')


    for r_viz in regions_for_viz:
        world_x0, world_y1 = map_proc_obj_for_viz.map_pixels_to_world(r_viz['x'], r_viz['y']) 
        world_x1, world_y0 = map_proc_obj_for_viz.map_pixels_to_world(r_viz['x'] + r_viz['w'], r_viz['y'] + r_viz['h'])
        rect_patch = plt.Rectangle((world_x0, world_y0), (world_x1 - world_x0), (world_y1 - world_y0),
                                   edgecolor='cyan', facecolor='none', linewidth=1, alpha=0.7, zorder=2)
        ax.add_patch(rect_patch)

    if not path_data_world:
        ax.set_title("Full Coverage Path (No Path Generated or All Points Processed)")
        plt.xlabel("X (meters)")
        plt.ylabel("Y (meters)")
        plt.axis('equal')
        plt.show()
        return None

    path_np = np.array(path_data_world)

    past_points_plot, = ax.plot([], [], 'go', markersize=5, label="Visited", zorder=3)
    current_point_plot, = ax.plot([], [], 'bo', markersize=7, label="Current", zorder=4)
    future_points_plot, = ax.plot(path_np[:, 0], path_np[:, 1], 'ro', markersize=3, alpha=0.5, label="To Visit", zorder=3)

    for _ in range(len(path_np) - 1):
        line, = ax.plot([], [], 'g-', linewidth=1.5, alpha=0.7, zorder=2)
        path_lines_plot_list.append(line)
    
    ax.set_title("Full Coverage Path (Frame 0)")
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.legend(loc='upper right')
    plt.axis('equal')
    plt.grid(True, linestyle='--', alpha=0.5)


    def update_anim(frame_idx):
        current_point_plot.set_data([path_np[frame_idx, 0]], [path_np[frame_idx, 1]]) # Use list for single point
        
        if frame_idx > 0:
            past_points_plot.set_data(path_np[:frame_idx, 0], path_np[:frame_idx, 1])
            if frame_idx <= len(path_lines_plot_list): # Ensure index is valid for lines
                 path_lines_plot_list[frame_idx-1].set_data(path_np[frame_idx-1:frame_idx+1, 0], path_np[frame_idx-1:frame_idx+1, 1])
        else: # First frame
            past_points_plot.set_data([],[]) # No past points yet
            # Clear all lines (in case of repeat)
            for line_plt in path_lines_plot_list:
                line_plt.set_data([],[])


        if frame_idx < len(path_np) -1 :
            future_points_plot.set_data(path_np[frame_idx+1:, 0], path_np[frame_idx+1:, 1])
        else: # Last point is current
            future_points_plot.set_data([],[])
            
        ax.set_title(f"Full Coverage Path (Waypoint {frame_idx+1}/{len(path_np)})")
        
        artists_to_update = [past_points_plot, current_point_plot, future_points_plot]
        artists_to_update.extend(path_lines_plot_list) # path_lines_plot_list already contains artists
        return artists_to_update


    ani = FuncAnimation(fig, update_anim, frames=len(path_data_world),
                        interval=100, blit=False, repeat=False) # interval 1000ms = 1s. blit=False for debugging/robustness
    
    plt.tight_layout()
    plt.show()
    return ani

# --- Main Execution ---
if __name__ == "__main__":
    # Create a dummy map.yaml if it doesn't exist for basic testing
    # YOU MUST REPLACE THIS WITH YOUR ACTUAL map.yaml
    try:
        with open(MAP_YAML_PATH, 'r') as f:
            yaml.safe_load(f) # Try to parse it
    except FileNotFoundError:
        print(f"'{MAP_YAML_PATH}' not found. Creating a DUMMY one for testing.")
        print("PLEASE REPLACE 'map.yaml' WITH YOUR ACTUAL FILE AND ENSURE 'image' points to your PGM.")
        dummy_yaml_content = {
            'image': 'map.pgm', # MAKE SURE 'map.pgm' (or your map image) IS IN THE SAME DIRECTORY
            'resolution': 0.05,
            'origin': [-1.0, -1.0, 0.0], # Adjust this to match your map's origin if (0,0) world is not map bottom-left
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }
        with open(MAP_YAML_PATH, 'w') as f:
            yaml.dump(dummy_yaml_content, f)
        print(f"Dummy '{MAP_YAML_PATH}' created. Please verify its content, especially 'origin' and 'image'.")
        # You might also need a dummy map.pgm if you don't have one:
        # from previous examples: create_dummy_pgm("map.pgm")
        # Ensure your actual map image (e.g., "input_file_0.png") is renamed or pointed to by map.yaml


    try:
        map_processor_obj = MapProcessor(MAP_YAML_PATH)
        
        # For testing, use the PGM from the YAML, or if you want to use the uploaded PNG:
        # 1. Ensure map.yaml's 'image' field points to 'input_file_0.png' (or your map file)
        # 2. Or, temporarily override map_processor_obj.map_image here if needed for a quick test,
        #    but changing YAML is cleaner.
        # Example: map_processor_obj.map_image = cv2.imread("input_file_0.png", cv2.IMREAD_GRAYSCALE)
        #          map_processor_obj.height_pixels, map_processor_obj.width_pixels = map_processor_obj.map_image.shape
        #          # And then re-binarize if you overrode map_image AFTER __init__
        #          map_processor_obj.binary_map = np.zeros_like(map_processor_obj.map_image, dtype=np.uint8)
        #          map_processor_obj.binary_map[map_processor_obj.map_image == 254] = 255


        full_path, ordered_regions, inflated_map_for_viz, mp_obj_for_viz = \
            plan_full_coverage(map_processor_obj, ROBOT_WIDTH_d, EFFECTIVE_COVERAGE_WIDTH_D, START_PHYSICAL_POS)

        if full_path:
            print(f"\nGenerated {len(full_path)} waypoints.")
            waypoints_to_save = np.array(full_path)
            np.save("waypoints2.npy", waypoints_to_save)
            print(f"Navigation waypoints saved to waypoints.npy (Shape: {waypoints_to_save.shape})")
            animation_obj = visualize_coverage(full_path, ordered_regions, inflated_map_for_viz, mp_obj_for_viz)
            # if animation_obj: animation_obj.save("coverage_animation.gif", writer='pillow', fps=5)
            
        else:
            print("Path planning failed or no path generated.")
            # Still show map and regions if any for debugging
            animation_obj = visualize_coverage([], ordered_regions, inflated_map_for_viz, mp_obj_for_viz)

    except FileNotFoundError as e:
        print(f"Error: {e}")
        print("Please ensure your map.yaml and the map image file (e.g., map.pgm or input_file_0.png) exist and are correctly named.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        import traceback
        traceback.print_exc()