#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV data
data = pd.read_csv(pd.io.common.StringIO("""fn,float,double,Q8.24,Q1.15,Q16.16,Q6.10
add,23456,42324,7149,6633,7149,8148
sub,18563,41265,7146,6787,7150,8062
mul,16426,27045,6949,8719,6398,7342
div,10856,19814,40159,9103,95156,8914
abs,3192,29160,4622,7671,4623,7671
exp,288987,651656,43541,30583,102205,37152
log,498524,969561,131266,22342,253014,41291
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
