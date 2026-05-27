# Whats In Here
This is a small sample of some project documents spanning data processing and flightline planning tasks. We **DO NOT** include our full data pipeline - you probably do not want to download ~1TB of LiDAR data. If you have interest in the processing files email Hugh - hal20a@fsu.edu

We include the class definition as well as a sample script used to generate flight lines. If canvas allows - we will also include a sample DSM so you can run the workflow. If you are on MacOS, note that multiprocessing in Jupyter Notebooks can often have issues. We reccomend running this in a Linux environment. We used Ubuntu 22.xx.

If you would like to download a sample DSM (if you do not see a .tif file here), we host a sample DSM on an AWS S3 compatible service provided by Digital Ocean. You can download it using the following command:
```sh
curl https://fl-parcels.atl1.digitaloceanspaces.com/ST_MARKS_SUBSET_DEM/ST_MARKS_FULL_MOSAIC.tif -o yourDSM.tif
```
which downloads our sample DSM to a file named yourDSM.tif in the current working directory. The downloaded DSM is at 1M resolution and is about 700MB in size.

## What You Need
This workflow depends on a good number of libraries, the python versions are included in the *requirements.txt* file. There are also some required libraries that depend on C++ binaries (namely GDAL), these should also be present to run the workflow. For GDAL see -> https://gdal.org/en/stable/download.html 

## Class definition ~/flight_path.py
The class definition file defines a class you can use to compute flight lines from a numpy array at variable resolution, you will need to provide mappings back to real space from pixel space in the form of a rasterio.Affine instance.

## Sample generation ~/make_flight_path.ipynb
A sample ipynb script for generating flight paths from a given DSM input - if a DSM is not included it is because it was too big for canvas. Naturally, we cannot include LiDAR data due to size constraints.