from sdf import *

# Define the font and text
FONT = 'ArialRegular.ttf'
TEXT = 'skoltech'

# Create the text geometry
text_geom = text(FONT, TEXT).extrude(0.1)

# Get the dimensions of the text geometry
w, h, d = text_geom.bounds()

# Create a cube with slightly larger dimensions than the text
cube_size = (w + 0.2, h + 0.2, d + 0.2)
cube_geom = box(cube_size)

# Combine the text and cube geometries
result = cube_geom & text_geom.translate((0, 0, 0.1))

# Save the result as an STL file
result.save('out.stl')