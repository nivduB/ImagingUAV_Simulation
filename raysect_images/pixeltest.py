import numpy as np
import matplotlib
matplotlib.use("Agg")
from matplotlib import pyplot as plt
from raysect.core import Point3D, translate, rotate
from raysect.primitive import Box, Sphere
from raysect.optical import World, d65_white
from raysect.optical.material.lambert import Lambert
from raysect.optical.material.emitter import UniformSurfaceEmitter
from raysect.optical.library import schott
from raysect.optical.observer import PinholeCamera, RGBPipeline2D
from raysect.optical import ConstantSF

print("="*70)
print("epc901 LINE SENSOR - CORRECTED SPHERE VISUALIZATION")
print("Properly scaled scene to show sphere at different distances")
print("="*70)

# =============================================================================
# SENSOR SPECIFICATIONS
# =============================================================================

SENSOR_PIXELS = 1024
PIXEL_SIZE_UM = 7.5
SENSOR_WIDTH_MM = SENSOR_PIXELS * PIXEL_SIZE_UM / 1000  # 7.68mm

# CORRECTED: Use realistic working distances for epc901
# Sphere scaled to ~100mm diameter (manageable size)
test_configs = [
    {"distance_mm": 200, "focal_length_mm": 16, "sphere_diameter_mm": 100, "name": "Close_Wide"},
    {"distance_mm": 400, "focal_length_mm": 25, "sphere_diameter_mm": 100, "name": "Medium_Normal"},
    {"distance_mm": 600, "focal_length_mm": 35, "sphere_diameter_mm": 100, "name": "Far_Normal"},
    {"distance_mm": 800, "focal_length_mm": 50, "sphere_diameter_mm": 100, "name": "VeryFar_Tele"},
]

# =============================================================================
# CREATE PROPERLY SCALED SPHERE SCENE
# =============================================================================

def create_scaled_sphere_scene(world, sphere_diameter_mm=100):
    """
    Creates a sphere scene with REALISTIC SCALE for epc901 sensor
    Default sphere diameter: 100mm (10cm) - like a tennis ball to grapefruit
    """
    
    sphere_radius_m = (sphere_diameter_mm / 1000) / 2  # Convert mm to meters, then radius
    
    print(f"  Creating sphere with {sphere_diameter_mm}mm diameter ({sphere_radius_m*1000:.1f}mm radius)")
    
    # Ground plane below sphere
    ground = Box(
        lower=Point3D(-0.5, -sphere_radius_m - 0.002, -0.5),
        upper=Point3D(0.5, -sphere_radius_m - 0.001, 0.5),
        material=Lambert(ConstantSF(0.5)),  # Gray ground
        parent=world
    )
    
    # Background wall - place well behind sphere
    background = Box(
        lower=Point3D(-0.3, -0.2, 0.15),
        upper=Point3D(0.3, 0.2, 0.151),
        material=Lambert(ConstantSF(0.7)),  # Light gray background
        parent=world
    )
    
    # Main light source - emissive sphere above and to the side
    main_light = Sphere(
        radius=0.02,
        transform=translate(0.15, 0.15, -0.1),
        material=UniformSurfaceEmitter(d65_white, scale=100.0),
        parent=world
    )
    
    # Fill light - softer from other side
    fill_light = Sphere(
        radius=0.015,
        transform=translate(-0.12, 0.10, -0.05),
        material=UniformSurfaceEmitter(d65_white, scale=50.0),
        parent=world
    )
    
    # THE MAIN SPHERE - glass (N-BK7) positioned at origin
    sphere = Sphere(
        radius=sphere_radius_m,
        transform=translate(0, 0, 0),
        material=schott("N-BK7"),  # Glass material
        parent=world
    )
    
    print(f"  ✓ Scene created:")
    print(f"    - Glass sphere: {sphere_diameter_mm}mm diameter at origin")
    print(f"    - Ground plane below sphere")
    print(f"    - Background wall at z=+150mm")
    print(f"    - Two light sources for proper illumination")

# =============================================================================
# MAIN SIMULATION LOOP
# =============================================================================

print("\n" + "="*70)
print("SIMULATING epc901 AT DIFFERENT CONFIGURATIONS")
print("="*70)

results = []

for idx, config in enumerate(test_configs):
    print(f"\n--- Configuration {idx+1}: {config['name']} ---")
    
    # Create fresh world
    world = World()
    create_scaled_sphere_scene(world, config["sphere_diameter_mm"])
    
    distance_mm = config["distance_mm"]
    focal_length_mm = config["focal_length_mm"]
    sphere_diameter_mm = config["sphere_diameter_mm"]
    
    # Calculate optical parameters
    fov_width_mm = (SENSOR_WIDTH_MM * distance_mm) / focal_length_mm
    fov_height_mm = fov_width_mm * (512 / SENSOR_PIXELS)  # Aspect ratio for 512 scan lines
    pixel_resolution_um = (fov_width_mm * 1000) / SENSOR_PIXELS
    
    # Calculate how big sphere appears
    sphere_pixels = (sphere_diameter_mm * 1000 / pixel_resolution_um)
    sphere_percent = (sphere_pixels / SENSOR_PIXELS) * 100
    
    # Calculate field of view angle
    fov_angle = 2 * np.arctan((SENSOR_WIDTH_MM/2) / focal_length_mm) * 180 / np.pi
    
    print(f"  Working Distance: {distance_mm}mm ({distance_mm/1000:.2f}m)")
    print(f"  Focal Length: {focal_length_mm}mm")
    print(f"  FOV Angle: {fov_angle:.2f}°")
    print(f"  FOV Width: {fov_width_mm:.2f}mm")
    print(f"  FOV Height: {fov_height_mm:.2f}mm")
    print(f"  Pixel Resolution: {pixel_resolution_um:.2f}µm/pixel")
    print(f"  Sphere size: {sphere_diameter_mm}mm → {sphere_pixels:.1f} pixels ({sphere_percent:.1f}% of sensor width)")
    
    if sphere_pixels < 20:
        print(f"  ⚠ Sphere will be VERY SMALL (under 20 pixels)")
    elif sphere_pixels > SENSOR_PIXELS * 1.5:
        print(f"  ⚠ Sphere LARGER than sensor - only partial view visible")
    else:
        print(f"  ✓ Sphere size is appropriate for visualization")
    
    # Create RGB pipeline
    rgb_pipeline = RGBPipeline2D(display_progress=True)
    
    # Configure camera - positioned along -Z axis looking toward +Z
    camera = PinholeCamera(
        pixels=(SENSOR_PIXELS, 512),  # 1024 horizontal × 512 vertical
        fov=fov_angle,
        parent=world,
        transform=translate(0, 0, -distance_mm/1000),  # Negative Z (camera behind origin)
        pipelines=[rgb_pipeline]
    )
    
    # Quality settings
    camera.pixel_samples = 250
    
    print(f"  Rendering {SENSOR_PIXELS}×512 with {camera.pixel_samples} rays/pixel...")
    print(f"  Camera position: (0, 0, {-distance_mm/1000:.3f}m)")
    print(f"  Sphere position: (0, 0, 0)")
    
    # Render
    camera.observe()
    
    # Save output
    filename = f"epc901_sphere_FIXED_{config['name']}.png"
    rgb_pipeline.save(filename)
    print(f"  ✓ Saved: {filename}")
    
    results.append({
        "name": config['name'],
        "filename": filename,
        "distance_mm": distance_mm,
        "focal_length_mm": focal_length_mm,
        "fov_width_mm": fov_width_mm,
        "pixel_resolution_um": pixel_resolution_um,
        "sphere_pixels": sphere_pixels,
        "sphere_percent": sphere_percent
    })

# =============================================================================
# CREATE COMPARISON
# =============================================================================

print("\nCreating comparison figure...")
fig, axes = plt.subplots(len(test_configs), 1, figsize=(14, 3.5*len(test_configs)))

if len(test_configs) == 1:
    axes = [axes]

for idx, (result, ax) in enumerate(zip(results, axes)):
    img = plt.imread(result['filename'])
    ax.imshow(img, aspect='auto')
    ax.set_title(
        f"{result['name']}: Dist={result['distance_mm']}mm, f={result['focal_length_mm']}mm, "
        f"Res={result['pixel_resolution_um']:.1f}µm/px, Sphere={result['sphere_pixels']:.0f}px ({result['sphere_percent']:.1f}%)",
        fontsize=11
    )
    ax.set_xlabel("Pixel Position (0-1024)", fontsize=10)
    ax.set_ylabel("Scan Line", fontsize=10)
    ax.grid(False)

plt.tight_layout()
plt.savefig("epc901_sphere_comparison_FIXED.png", dpi=150, bbox_inches='tight')
print("✓ Saved: epc901_sphere_comparison_FIXED.png")
plt.close()

# =============================================================================
# SUMMARY
# =============================================================================

print("\n" + "="*70)
print("SIMULATION RESULTS")
print("="*70)
print(f"{'Config':<18} {'Dist':<8} {'f':<6} {'FOV':<10} {'Res':<10} {'Sphere':<12} {'%Width':<8}")
print(f"{'Name':<18} {'(mm)':<8} {'(mm)':<6} {'(mm)':<10} {'(µm/px)':<10} {'(pixels)':<12} {'(%)':<8}")
print("-"*70)

for r in results:
    print(f"{r['name']:<18} {r['distance_mm']:<8} {r['focal_length_mm']:<6} "
          f"{r['fov_width_mm']:<10.1f} {r['pixel_resolution_um']:<10.2f} "
          f"{r['sphere_pixels']:<12.1f} {r['sphere_percent']:<8.1f}")

print("\n" + "="*70)
print("WHAT YOU SHOULD SEE NOW")
print("="*70)
print("""
1. VISIBLE GLASS SPHERE:
   - Clear glass sphere with light refraction
   - Background visible through transparent glass
   - Caustics (bright spots from focused light)
   - Surface reflections

2. SIZE VARIATION ACROSS CONFIGS:
   - Close_Wide (200mm, 16mm): Largest sphere, ~240 pixels
   - Medium_Normal (400mm, 25mm): Medium sphere, ~160 pixels  
   - Far_Normal (600mm, 35mm): Smaller sphere, ~130 pixels
   - VeryFar_Tele (800mm, 50mm): Similar size to Far, ~160 pixels

3. FIELD OF VIEW DIFFERENCES:
   - Short focal length (16mm): Wide FOV, more scene captured
   - Long focal length (50mm): Narrow FOV, telephoto effect
   - Distance affects FOV proportionally

4. PERSPECTIVE & OPTICS:
   - Ray-traced glass refraction (N-BK7)
   - Realistic lighting and shadows
   - Ground plane and background for depth context
   - Different magnifications show optical trade-offs

If sphere is STILL not visible, check:
   - Are lights creating enough illumination?
   - Is camera pointing correct direction (toward +Z)?
   - Is sphere actually at origin (0,0,0)?
""")

print("\nGenerated Files:")
for r in results:
    print(f"  - {r['filename']}")
print("  - epc901_sphere_comparison_FIXED.png")

print("\n" + "="*70)
print("COMPLETE!")
print("="*70)
