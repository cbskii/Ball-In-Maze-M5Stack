# M5Stack Ball In Maze Game
A little "ball in maze" style game for the M5Stack using the ESP IDF.

#### From this:
<real ball in maze example>

#### To this:
<M5Stack game GIF>

## Building

Notes:
- Follow esp IDF set up instructions, use get_idf command to export needed environment variables
- Build with "idf.py build", flash with "idf.py flash" (may have to rm -rf build dir if there are
  issues)
- Monitor serial output with "idf.py monitor", exit with "Ctrl-]"
- Combine all with "idf.py clean flash monitor"
