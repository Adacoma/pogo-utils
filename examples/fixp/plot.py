#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV data
data = pd.read_csv(pd.io.common.StringIO("""fn,float,double,Q8.24,Q1.15,Q16.16,Q6.10
add,23600,41992,7151,6633,7146,8146
sub,18565,46737,7149,6788,7148,8062
mul,16405,27046,6949,8718,6398,7341
div,10880,19815,40159,9105,171263,8915
abs,3205,28303,4623,7669,4624,7669
exp,287067,680127,43542,30580,99159,37153
log,493867,999890,132500,22341,248113,35246
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
