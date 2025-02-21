#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV data
data = pd.read_csv(pd.io.common.StringIO("""fn,float,double,Q8.24,Q1.15,Q16.16,Q6.10
add,20051,41802,7149,7990,7148,8148
sub,18579,46550,7149,8148,7150,8194
mul,13323,18535,6671,8718,6292,8814
div,10531,16246,7677,9100,46822,8863
abs,3226,27067,4623,7671,4626,7672
exp,284926,670607,43540,30439,66596,42008
log,495049,987892,123461,22340,207407,36627
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
