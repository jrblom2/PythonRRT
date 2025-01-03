# PythonRRT

Author: Joseph Blom

This simple python project is built to run a simple 3D RRT pathfinding algorythm. The main functions to do this are in RRT while the code to execute them is in the plotter notebook file.

# Usage
A random plot can be produced by running the single cell in the plotter notebook and by passing an empty value to the `doRRT()` function. This function can take a seed value. This project is wrapped in a Jupyter Notebook becuase the defualt plotly browser rendering was working poorly at the time of completion.

`jupyter nbconvert --to html --no-input plotter.ipynb` can be used to export a plot to a web freindly format.
