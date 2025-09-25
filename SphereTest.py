# --- imports
import matplotlib
matplotlib.use("Agg")  # headless backend (prevents "FigureCanvasAgg is non-interactive")
from raysect.core import Point3D, translate, rotate
from raysect.primitive import Box, Sphere, Mesh
from raysect.optical import World, d65_white
from raysect.optical.material.lambert import Lambert
from raysect.optical.material.emitter import UniformSurfaceEmitter
from raysect.optical.library import schott
from raysect.optical.observer import PinholeCamera, RGBPipeline2D

# --- world
world = World()

# --- ground (Lambertian)
ground = Box(
    lower=Point3D(-50, -1.51, -50),
    upper=Point3D(50, -1.5, 50),
    material=Lambert(),
    parent=world
)

# --- emissive wall
emitter = Box(
    lower=Point3D(-10, -10, 10),
    upper=Point3D(10, 10, 10.1),
    material=UniformSurfaceEmitter(d65_white, scale=4.0),
    transform=rotate(45, 0, 0),
    parent=world
)

# --- glass sphere (make sure it's in the world)
sphere = Sphere(
    radius=1.5,
    transform=translate(0, 0.0001, 0),
    material=schott("N-BK7"),
    parent=world
)

# --- camera + pipeline

# create camera (no pipeline kwarg)
camera = PinholeCamera(
    pixels=(800, 600),
    fov=45,
    parent=world,
    transform=translate(0, 0, -15)
)
camera.pixel_samples = 50

# create pipeline and assign as tuple
rgb = RGBPipeline2D()
camera.pipelines = (rgb,)   # <-- overwrite with a tuple

# render & save
camera.observe()
rgb.save("render.png")
print("Saved render.png")
