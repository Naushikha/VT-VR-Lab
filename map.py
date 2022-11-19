import csv
import cartopy.crs as ccrs
import cartopy.io.img_tiles as cimgt
from cartopy.mpl.gridliner import LONGITUDE_FORMATTER, LATITUDE_FORMATTER

# Fix for Tkinter #########
import matplotlib

matplotlib.use("agg")
###########################
import matplotlib.pyplot as plt


fileName = "2022_11_19-15_10.unreal"
startSample = 110
endSample = 150

AISMessages = []

with open(f"out/{fileName}.csv", "r") as csvFile:
    reader = csv.DictReader(csvFile)
    i = 0
    for line in reader:
        # Extract a sample from AIS recording
        if startSample <= i <= endSample:
            AISMessages.append(line)
        i += 1

### Drawing map
# Colombo Harbor coordinates
# 6.970111, 79.821057
# 6.937585, 79.855412
# [ lonMin, lonMax, latMin, latMax ]
harbor_extent = [
    79.821057,
    79.858412,
    6.937585,
    6.974111,
]

# Create a Stamen watercolor background instance or use Google Maps
# terrain_requestor = cimgt.Stamen("watercolor")
terrain_requestor = cimgt.GoogleTiles()
# terrain_requestor = cimgt.GoogleTiles(style="satellite") # Use satellite images

# Define map size and dpi
fig = plt.figure(figsize=(10, 9), dpi=150)

# Create a GeoAxes in the tile's projection
ax = plt.axes(projection=ccrs.PlateCarree())

# Limit the extent of the map to a small longitude/latitude range
ax.set_extent(harbor_extent, crs=ccrs.PlateCarree())

# Increase quality in Cartopy map
ax.add_image(terrain_requestor, 15, interpolation="spline36", regrid_shape=2000)

# Draw grid-lines on the map (Optional)
# gl = ax.gridlines(draw_labels=True, alpha=0.3)
# gl.xlabels_top = gl.ylabels_right = False
# gl.xformatter = LONGITUDE_FORMATTER
# gl.yformatter = LATITUDE_FORMATTER

i = 0
prevAISMessage = ""

# Plot origin on the map
originLat = 6.955879
originLon = 79.844690
plt.plot(
    originLon,
    originLat,
    marker="o",
    markersize=3,
    markeredgecolor="red",
    markerfacecolor="green",
)


for AISMessage in AISMessages:
    i += 1
    if prevAISMessage == "":
        prevAISMessage = AISMessage
        continue
    plt.plot(
        [float(prevAISMessage["Longitude"]), float(AISMessage["Longitude"])],
        [float(prevAISMessage["Latitude"]), float(AISMessage["Latitude"])],
        "ob--",
        markersize=1.5,
        linewidth=0.5,
    )
    plt.pause(0.01)  # A little tick to force an update on the map
    plt.savefig(f"img/{i}.png", bbox_inches="tight")
    print(f"Map Progress: {i}/{endSample - startSample + 1}")
    prevAISMessage = AISMessage

# RUN: C:\APPS\python-3.10.0\python.exe map.py

# References:
# https://stackoverflow.com/a/49988926/11436038
# https://stackoverflow.com/questions/49155110/why-do-my-google-tiles-look-poor-in-a-cartopy-map
# https://github.com/SciTools/cartopy/issues/1048
# https://github.com/SciTools/cartopy/issues/1563
