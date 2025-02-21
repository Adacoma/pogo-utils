#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV data
data = pd.read_csv(pd.io.common.StringIO("""fn,float,double,Q8.24,Q1.15,Q16.16,Q6.10
add,23602,40325,6722,7989,6718,8146
sub,18565,41265,6718,8146,6721,8196
mul,10283,15023,6242,8719,5864,8816
div,10488,14342,7678,9006,41014,8911
abs,3179,27067,4622,7671,4624,7672
exp,288844,650799,43541,30439,105869,37152
log,495053,968037,126840,22343,263386,33819
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
