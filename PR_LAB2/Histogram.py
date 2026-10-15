import numpy as np
import matplotlib.pyplot as plt

class Histogram2D:
    """
    Class for creating and manipulating a 2D histogram.
    """

    def __init__(self, num_bins_x, num_bins_y, x_range, y_range):
        """
        Initialize a new Histogram2D instance.

        :param:
            num_bins_x (int): Number of bins in the X-direction.
            num_bins_y (int): Number of bins in the Y-direction.
            x_range (numpy.ndarray): Range of values for the X-axis.
            y_range (numpy.ndarray): Range of values for the Y-axis.
        """

        self.num_bins_x = num_bins_x
        self.num_bins_y = num_bins_y

        self.nCells = num_bins_x * num_bins_y

        self.x_range = x_range
        self.size_x = x_range[-1]
        self.cell_size_x = 2 * self.size_x / (self.num_bins_x)
        self.x_center_range = np.linspace(self.x_range[0] + self.cell_size_x / 2, self.x_range[-1] - self.cell_size_x / 2, self.num_bins_x)

        self.y_range = y_range
        self.size_y = y_range[-1]
        self.cell_size_y = 2 * self.size_y / (self.num_bins_y)
        self.y_center_range = self.y_range + self.cell_size_y / 2 - 1

        self.histogram = np.zeros(num_bins_x * num_bins_y, dtype=float)

    @property
    def histogram_2d(self):
        """
        Get the 2D histogram data as a NumPy array.

        :returns:
            numpy.ndarray: The 2D histogram data.
        """
        return self.histogram.reshape((self.num_bins_x, self.num_bins_y))

    @histogram_2d.setter
    def histogram_2d(self, histogram_2d):
        """
        Set the 2D histogram data.

        :param:
            histogram_2d (numpy.ndarray): A 2D NumPy array containing the new histogram data.

        :raises:
            ValueError: If the provided histogram_2d has a different shape than the initialized histogram grid.

        """
        if histogram_2d.shape == (self.num_bins_x, self.num_bins_y):
            self.histogram = histogram_2d.flatten()
        else:
            raise ValueError("The provided histogram_2d must have the same shape as the initialized histogram grid.")

    @property
    def histogram_1d(self):
        """
        Get the histogram data as a 1D NumPy array.

        :returns:
            numpy.ndarray: The 1D histogram data.
        """
        return self.histogram

    @histogram_1d.setter
    def histogram_1d(self, histogram_1d):
        """
        Set the 1D histogram data.

        :param:
            histogram_1d (numpy.ndarray): A 1D NumPy array containing the new histogram data.

        :raises:
            ValueError: If the provided histogram_1d has a different size than the initialized histogram.
        """
        if histogram_1d.size == self.histogram.size:
            self.histogram = histogram_1d
        else:
            raise ValueError("The provided histogram_1d must have the same size as the initialized histogram.")

    def plot_histogram(self):
        """
        Plot the 2D histogram using Matplotlib.
        """
        plt.xlabel('X-Axis')
        plt.ylabel('Y-Axis')
        plt.title('2D Histogram')
        ax = plt.gca()
        ax.grid(color='w', linestyle='-', linewidth=0.5)
        ax.set_xlim([self.x_range[0], self.x_range[-1]])
        ax.set_ylim([self.y_range[0], self.y_range[-1]])
        ax.set_xticks(np.arange(self.x_range[0], self.x_range[-1], self.cell_size_x))
        ax.set_yticks(np.arange(self.y_range[0], self.y_range[-1], self.cell_size_y))

        plt.imshow(self.histogram_2d, extent=[self.x_range[0], self.x_range[-1], self.y_range[0], self.y_range[-1]], origin='lower', aspect='auto')

        plt.show()

    @property
    def element(self):
        """
        Property to access individual elements of the histogram using range values.

        :returns:
            ElementAccessor: An instance of ElementAccessor for getting and setting individual elements by range.
        """
        class ElementAccessor:
            def __init__(self, histogram, num_bins_x, num_bins_y, x_range, y_range):
                self.histogram = histogram
                self.num_bins_x = num_bins_x
                self.x_range = x_range
                self.y_range = y_range
                self.size_x = x_range[-1]
                self.cell_size_x = 2 * self.size_x / (self.num_bins_x - 1)
                self.num_bins_y = num_bins_y
                self.size_y = y_range[-1]
                self.cell_size_y = 2 * self.size_y / (self.num_bins_y - 1)

            def __getitem__(self, coordinates):
                x_coord, y_coord = coordinates
                x_bin = np.digitize(x_coord, self.x_range) - 1
                y_bin = np.digitize(y_coord, self.y_range) - 1
                if 0 <= x_bin < self.num_bins_x and 0 <= y_bin < len(self.histogram) // self.num_bins_x:
                    index = x_bin + y_bin * self.num_bins_x
                    return self.histogram[index]
                else:
                    raise ValueError("Bin coordinates are out of bounds")

            def __setitem__(self, coordinates, value):
                x_coord, y_coord = coordinates
                # x_bin = np.digitize(x_coord, self.x_range)
                # y_bin = np.digitize(y_coord, self.y_range)
                x_bin=round((x_coord + self.size_x) / self.cell_size_x)
                y_bin=round((y_coord + self.size_y) / self.cell_size_y)
                if 0 <= x_bin < self.num_bins_x and 0 <= y_bin < self.num_bins_x:
                    index = x_bin + y_bin * self.num_bins_x
                    self.histogram[index] = value
                else:
                    raise ValueError("Bin coordinates are out of bounds")

        return ElementAccessor(self.histogram, self.num_bins_x, self.num_bins_y, self.x_range, self.y_range)

    def __mul__(self, other_histogram):
        """
        Multiply two Histogram2D objects element-wise using the * operator.

        :param:
            other_histogram (Histogram2D): Another Histogram2D object to be multiplied with.

        :raises:
            ValueError: If the provided histogram has a different shape than the current histogram.
        """
        if (self.num_bins_x, self.num_bins_y) != (other_histogram.num_bins_x, other_histogram.num_bins_y):
            raise ValueError("Histograms must have the same shape for element-wise multiplication.")

        result = Histogram2D(self.num_bins_x, self.num_bins_y, self.x_range, self.y_range)
        result.histogram = self.histogram * other_histogram.histogram
        return result


if __name__ == "__main__":
    # Create a Histogram2D instance
    num_bins_x = 11
    num_bins_y = 11
    x_range = np.linspace(-5, 5, num_bins_x )
    y_range = np.linspace(-5, 5, num_bins_y )
    hist = Histogram2D(num_bins_x, num_bins_y, x_range, y_range)

    print("x_range: ", x_range)
    print("y_range: ", y_range)

    hist.histogram_2d[1,2]=1
    hist.element[1,2]=1
    hist.plot_histogram()
    plt.pause(5)

    # Modify individual elements using range values
    hist.element[-2.5, 5] = 100  # Set the value at (0.2, 0.3)
    value = hist.element[0,0]
    print(f"Updated value at (0.2, 0.3): {value}")

    # Get the entire 2D histogram and set a new one
    hist_data = hist.histogram_2d
    print("2D Histogram Data:")
    print(hist_data)

    print("1D Histogram Data:")
    print(hist.histogram_1d)

    new_histogram_data = np.random.randint(0, 10, (num_bins_x, num_bins_y))
    hist.histogram_2d = new_histogram_data

    # Plot the 2D histogram
    hist.plot_histogram()

    # Access individual elements using 1D index
    value_1d = hist.histogram_1d[42]
    print(f"Value at 1D index 42: {value_1d}")

    # Modify individual elements using 1D index
    hist.histogram_1d[42] = 99
    value_1d = hist.histogram_1d[42]
    print(f"Updated value at 1D index 42: {value_1d}")
