import rasterio
import math
from rasterio.transform import Affine
import numpy as np
import pyastar2d
from copy import deepcopy
import geopandas as gp
import pandas as pd


class FlightPath:
    """A class that encapsulates logic for calculating low level flight paths based on estimated ground visibility"""

    def __init__(
        self,
        data: np.ndarray,
        crs: int,
        transform: rasterio.transform.Affine,
        viewshed_distance: int = 0,
        resolution_reduction_factor: int = 1,
        avoid_value: int | float | None = None,
    ) -> None:
        """
        Args:
            data (np.ndarray): A tif or other spatially enabled image represented as a numpy array. The format of the tif should be such that the cell value is representative of the amount of visible ground in that cell or another equivilant metric.
            crs (int): The coordinate reference system of the data.
            transform (rasterio.transform.Affine): A transform that maps from pixel space to geographic space for 'data'. It assumed to be a symetric transform over x,y
            viewshed_distance (int): The distance to weight each cell by in a sliding circle approach - this affects how far out we are expected to see and thus how we weight the path - in CRS units relative to original resolution, i.e. if resolution 3m then 10 is 30m.
            resolution_reduction_factor (int): By what factor to reduce resolution by before calculating. e.g. , if resultution in crs is originally 1 and we set to 3 then calculations are done at a resultion of 3 CRS units ( using average )
            avoid_value (int | float | None): cell value to HEAVILY incentivize the algorithm to avoid ( like an open water mask )
        """

        self.data = deepcopy(data)
        self.crs = crs
        self.transform = Affine(
            transform[0],
            transform[1] * max(resolution_reduction_factor, 1),
            transform[2],
            transform[3],
            transform[4],
            transform[5] * max(resolution_reduction_factor, 1),
        )
        self.viewshed_distance = viewshed_distance
        self.resolution_reduction_factor = resolution_reduction_factor
        self.avoid_value = avoid_value

    def calculateFlightPath(
        self,
        consider_as_ground_value: int | float = 1,
        included_buffer: int | None = 0,
        included_buffer_type: str = "x",
    ) -> gp.GeoDataFrame:
        """Calculate a flight path.

        Args:
            consider_as_ground_value (int | float, optional): The value below or equal to which a cell will be considered a ground cell. Defaults to 1.
            included_buffer (None | int , optional): To reduce edge effects you can specify a known buffer. The buffer will be used to calculate metrics but will NOT be available for A* to use as a path. e.g., specify a viewshed size of 200 and a buffer of 200 - you would pass a segment with width >= 800 and only the middle 400 would be used for pathfinding.
            included_buffer_type (None | Literal): either "x" or "y" and denotes if the buffer should be applied on shape[0] or shape[1] , default "x"

        Returns:
            gp.GeoDataFrame
        """

        # TODO: Support flight height as well as terrain features

        assert (
            included_buffer_type == "x" or included_buffer_type == "y"
        ), "included buffer must be x or y"

        WINDOW_SIZE: int | float = int(self.transform[1])

        if self.avoid_value:
            self.data[self.data == self.avoid_value] = 9999

        self.data = self.data.clip(
            min=0, max=None
        )  # we clip all values to min() = 0 , cant have negative height lol

        if self.resolution_reduction_factor > 1:
            pad_x = (WINDOW_SIZE - (self.data.shape[0] % WINDOW_SIZE)) % WINDOW_SIZE
            pad_y = (WINDOW_SIZE - (self.data.shape[1] % WINDOW_SIZE)) % WINDOW_SIZE

            padded_matrix = np.pad(
                array=self.data,
                pad_width=((0, (pad_x)), (0, (pad_y))),
                constant_values=np.nan,
            )

            windows = np.lib.stride_tricks.sliding_window_view(
                x=padded_matrix, window_shape=(WINDOW_SIZE, WINDOW_SIZE)
            )[::WINDOW_SIZE, ::WINDOW_SIZE]
        else:
            windows = self.data

        # initialize ground percentage
        ground_percent = np.zeros(
            shape=(windows.shape[0], windows.shape[1]), dtype=np.float32
        )

        # initialize
        for x in range(0, ground_percent.shape[0]):
            for y in range(0, ground_percent.shape[1]):

                window: np.ndarray = windows[x, y]

                ground_ish_proportion = (
                    window[window <= consider_as_ground_value].size / window.size
                )

                # adjust as this decides what the weight scaling is between nodes
                ground_percent[x, y] = 900 / np.exp(2 ** np.exp(ground_ish_proportion))

        STEP = math.floor(self.viewshed_distance / WINDOW_SIZE)

        if STEP > 0:
            blurred = np.ndarray(shape=ground_percent.shape, dtype=np.float32)

            for x in range(0, ground_percent.shape[0]):
                for y in range(0, ground_percent.shape[1]):
                    window_avg = ground_percent[
                        max(x - STEP, 0) : x + STEP, max(y - STEP, 0) : y + STEP
                    ].mean()

                    blurred[x, y] = window_avg

            ground_percent = blurred

        start = 0 + (included_buffer // self.resolution_reduction_factor)
        end = (
            ground_percent.shape[0]
            if included_buffer_type == "x"
            else ground_percent.shape[1]
        ) - (included_buffer // self.resolution_reduction_factor)

        goalVal = (end - start) // 2

        isX = included_buffer_type == "x"

        path = pyastar2d.astar_path(
            weights=ground_percent[
                start if isX else 0 : end if isX else ground_percent.shape[1],
                start if not isX else 0 : (end if not isX else ground_percent.shape[1]),
            ],
            start=(0, goalVal if isX else ground_percent.shape[0] - 1),
            goal=(0, goalVal if not isX else ground_percent.shape[1] - 1),
            allow_diagonal=True,
        )

        # map back to physical space from pixel space
        mapped = pd.DataFrame(
            [
                {
                    "x": ((x[1]) * WINDOW_SIZE) + self.transform[0],
                    "y": ((-x[0]) * WINDOW_SIZE) + self.transform[3],
                }
                for x in path
            ]
        )

        df = gp.GeoDataFrame(
            data=mapped,
            geometry=gp.points_from_xy(mapped["x"], mapped["y"]),
            crs=self.crs,
        )

        return df.to_crs(4326)
