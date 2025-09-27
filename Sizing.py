import math 


PixelWidth = 7.5e-6     #[m] pixel width (across-track); This is what we will need for the IFOV calc
PixelHeight = 120-6     #[m] pixel height 
NumberPixels = 1024     # Number of Pixels as denoted 1024 x 1 
F = 0.01                #[m] pinhole-to-sensor distance 
H = 0.50                #[m] heighgt above the ground
LambdaViolet = 400e-9   #[m] Violet wavelength for pinhole sizing
LambdaRed = 700e-9      #[m] Red wavelength for pinhole sizing 

