#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV data
data = pd.read_csv(pd.io.common.StringIO("""fn,float,double,Q8.24,Q1.15,Q16.16,Q6.10
add,22550,43848,6719,6633,6720,8146
sub,18581,42930,6718,6787,6721,8062
mul,16405,27093,6599,8716,5992,7341
div,10879,19816,40158,9008,27823,8915
abs,3191,29113,4622,7670,4623,7671
exp,287066,677510,43542,30439,101678,37154
log,493895,999890,134021,22340,251439,35247
"""))

# Set function names as index
data.set_index('fn', inplace=True)
data /= 1000

# Plot the results
plt.figure(figsize=(10, 6))
data.plot(kind='bar', figsize=(12, 6))
plt.xlabel("Function")
plt.ylabel("Time (ųs)")
plt.title("Execution speed on Pogobots per 1000 instance of functions for different data types")
plt.xticks(rotation=45)
plt.legend(title="Data Type")
plt.grid(axis='y', linestyle='--', alpha=0.7)
plt.savefig("bench.pdf")

# MODELINE "{{{1
# vim:noexpandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
# vim:foldmethod=marker
