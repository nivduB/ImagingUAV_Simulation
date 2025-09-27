#Authors: UPENN Student Researchers in ModLab

import math 

#[m] is meters
#[px] is pixels
#[rad] radians
#[deg] degrees

PixelWidth = 7.5e-6     #[m] pixel width (across-track); This is what we will need for the IFOV calc
PixelHeight = 120e-6     #[m] pixel height 
NumberPixels = 1024     #[px] Number of Pixels as denoted 1024 x 1 
F = 0.01                #[m] pinhole-to-sensor distance 
H = 0.50                #[m] height above the ground
LambdaViolet = 400e-9   #[m] Violet wavelength for pinhole sizing
LambdaRed = 700e-9      #[m] Red wavelength for pinhole sizing 
HoleSize = 0.3e-3       #[m] Maximum Actual hole diameter (0.30 mm)

def sensor():
    #for ifov we are using a small angle approximation
    ifov = PixelWidth / F         #[rad] per pixel FOV or IFOV (we need to use pixel width)
    
    fovRad = NumberPixels * ifov          #[rad] getting actual FOV with total number of pixels
    fovDeg = math.degrees(fovRad) #[deg] of FOV 
    # Ground Sample Distance basically the amount of detail taken relative to the ground
    GSDx = H * (PixelWidth/F)      #[m] Ground Sample Distance. Real World Distance mapped by each pixel. This is width direction
    swath = NumberPixels * GSDx    #[m] total width of ground that is being covered by the scanner 
    
    # Edge GSD correction (flat ground): phi = arctan( (SENSOR_WIDTH/2) / f )
    sensorWidth = NumberPixels * PixelWidth               # Width of the sensor    
    phiEdge = math.atan((sensorWidth / 2) / F)            # Due to the nature of how the pixels are captured near the edge
    GSDCorrection = GSDx / math.cos(phiEdge)              #Corrects for how distances are stretched in one direction; This is linear GSD correction    
    return ifov, fovRad, fovDeg, GSDx, swath, sensorWidth, phiEdge, GSDCorrection
    
def pinhole(): 
    #https://www.idexot.com/media/wysiwyg/02_Gaussian_Beam_Optics.pdf
    # Theoretical optimum (from minimizing combined blur):
    #   d* = sqrt(2.44 * λ * f)  ≈ 1.56 * sqrt(λ f)
    # Conservative/practical rule (for broadband, real pinholes):
    #   d* ≈ 1.9 * sqrt(λ f)

    # --- Violet (400 nm) ---
    dTheoreticalViolet   = math.sqrt(2.44 * LambdaViolet * F)
    dConservativeViolet  = 1.9 * math.sqrt(LambdaViolet * F)

    nFnoTheoViolet  = F / dTheoreticalViolet
    nFnoConsViolet = F / dConservativeViolet
    nFnoActualViolet = F / HoleSize 

    print(f"Violet 400 nm:")
    print(f"Theoretical d = {dTheoreticalViolet*1e3:.3f} mm, f/{nFnoTheoViolet:.1f}")
    print(f"Conservative d = {dConservativeViolet*1e3:.3f} mm, f/{nFnoConsViolet:.1f}\n")
    print(f" Actual d= {HoleSize*1e3:.3f} mm,  f/{nFnoActualViolet:.1f}\n")

    # --- Red (700 nm) ---
    dTheoreticalRed  = math.sqrt(2.44 * LambdaRed * F)
    dConservativeRed  = 1.9 * math.sqrt(LambdaRed * F)

    nFnoTheoRed  = F / dTheoreticalRed
    nFnoConsRed  = F / dConservativeRed
    nFnoActualRed = F / HoleSize


    print(f"Red 700 nm:")
    print(f"Theoretical d = {dTheoreticalRed*1e3:.3f} mm, f/{nFnoTheoRed:.1f}")
    print(f"Conservative d = {dConservativeRed*1e3:.3f} mm, f/{nFnoConsRed:.1f}")
    print(f"  Actual d = {HoleSize*1e3:.3f} mm,  f/{nFnoActualRed:.1f}")



     
if __name__ == "__main__":
    sensor()
    
    ifov, fovRad, fovDeg, GSDx, swath, sensorWidth, phiEdge, GSDCorrection = sensor()
    print("=== Sensor Geometry ===")
    print(f"IFOV: {ifov:.6e} rad")
    print(f"FOV:  {fovRad:.6f} rad  ({fovDeg:.2f} deg)")
    print(f"GSDx: {GSDx*1e3:.3f} mm")
    print(f"Swath: {swath*1e2:.1f} cm")
    print(f"Edge angle: {math.degrees(phiEdge):.2f}°")
    print(f"GSD Correction (edge): {GSDCorrection*1e3:.3f} mm\n")

    # Call pinhole() to show pinhole sizing for violet and red
    pinhole()

     
    


    