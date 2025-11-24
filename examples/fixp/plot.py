#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV data
data = pd.read_csv(pd.io.common.StringIO("""fn,float,double,Q8.24,Q1.15,Q16.16,Q6.10
add,20023,41891,7149,6633,3719,8147
sub,18575,44882,7149,6789,4101,8062
mul,16447,32849,6951,9386,6395,7341
div,14735,24051,40157,9104,21779,8915
abs,3201,27068,4623,7672,4957,7672
exp,321940,637792,45539,34533,28059,37439
log,466492,944323,136691,40539,81780,31776
tanh,819108,1558120,26349,25819,24444,24870
"""))

# Set function names as index
data.set_index('fn', inplace=True)
data /= 1000

# Plot the results
plt.figure(figsize=(10, 6))
data.plot(kind='bar', figsize=(12, 6))
plt.xlabel("Function")
plt.ylabel("Time (ųs)")
plt.title("Execution speed on Pogobots of one instance of functions for different data types")
plt.xticks(rotation=45)
plt.legend(title="Data Type")
plt.grid(axis='y', linestyle='--', alpha=0.7)
plt.savefig("bench.pdf")

# MODELINE "{{{1
# vim:noexpandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
# vim:foldmethod=marker
