import csv
import cartopy.crs as ccrs
import cartopy.io.img_tiles as cimgt

# import matplotlib  # https://stackoverflow.com/a/49988926/11436038
# matplotlib.use("agg")
import matplotlib.pyplot as plt

processD = []

with open("out.csv", "r") as csvFile:
    reader = csv.DictReader(csvFile)
    i = 0
    for line in reader:
        if 110 <= i <= 150:
            processD.append(line)
        i += 1

# print(processD)

# newlist = sorted(processD, key=lambda d: d["TIMESTAMP"])
newlist = processD

# print(newlist)
# print(*newlist, sep='\n')

### Drawing map
# 6.970111, 79.821057
# 6.937585, 79.855412
# Colombo Harbor coordinates
harbor_extent = [
    79.821057,
    79.858412,
    6.937585,
    6.974111,
]  # ax.set_extent([lonmin, lonmax, latmin, latmax]
# Create a Stamen watercolor background instance / Google Maps
# terrain_requestor = cimgt.Stamen('watercolor') # cimgt.GoogleTiles()
terrain_requestor = cimgt.GoogleTiles()
# Define map size and dpi
fig = plt.figure(figsize=(10, 9), dpi=150)

# Create a GeoAxes in the tile's projection
ax = plt.axes(projection=ccrs.PlateCarree())
# Limit the extent of the map to a small longitude/latitude range
ax.set_extent(harbor_extent, crs=ccrs.PlateCarree())
# Add the Stamen data at zoom level 6
ax.add_image(terrain_requestor, 15, interpolation="spline36", regrid_shape=2000)

# https://stackoverflow.com/questions/49155110/why-do-my-google-tiles-look-poor-in-a-cartopy-map

# from cartopy.mpl.gridliner import LONGITUDE_FORMATTER, LATITUDE_FORMATTER
# request = cimgt.GoogleTiles()

# fig = plt.figure(figsize=(13, 9))
# ax = plt.axes(projection=request.crs)

# gl = ax.gridlines(draw_labels=True, alpha=0.3)
# gl.xlabels_top = gl.ylabels_right = False
# gl.xformatter = LONGITUDE_FORMATTER
# gl.yformatter = LATITUDE_FORMATTER

# ax.set_extent(harbor_extent)
# ax.add_image(request, 15)
# ax.plot([79.821057, 79.858412], [6.937585, 6.974111], 'ob-', markersize=1.5, linewidth=0.5, linestyle="dashed")
# plt.show(block=False)
# plt.pause(1)
# input("Press Enter to continue...")

c = 0
prev_ais_point = ""

midLat = 6.95967
midLon = 79.845175
plt.plot(
    midLon,
    midLat,
    marker="o",
    markersize=20,
    markeredgecolor="red",
    markerfacecolor="green",
)


for ais_point in newlist:
    if prev_ais_point == "":
        prev_ais_point = ais_point
        continue
    print(float(ais_point["Longitude"]), float(ais_point["Latitude"]))
    plt.plot(
        [float(prev_ais_point["Longitude"]), float(ais_point["Longitude"])],
        [float(prev_ais_point["Latitude"]), float(ais_point["Latitude"])],
        "ob-",
        markersize=1.5,
        linewidth=0.5,
        linestyle="dashed",
    )
    plt.pause(0.01)  # little trick to update the map
    plt.savefig(f"img/{c}.png", bbox_inches="tight")
    c += 1
    # time.sleep(0.1)
    prev_ais_point = ais_point  # Push this for last

plt.show()

# for ais_point in newlist:
# 	print(float(ais_point['LON']), float(ais_point['LAT']))
# 	plt.plot(float(ais_point['LON']), float(ais_point['LAT']), 'ro', markersize=2)       # plot the red dot on the map
# 	plt.pause(0.01)                          # little trick to update the map
# 	time.sleep(0.1)

# RUN: C:\APPS\python-3.10.0\python.exe map.py
