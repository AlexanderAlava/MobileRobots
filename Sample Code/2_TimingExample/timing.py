# This program demonstrates usage of some timing functions.
# See https://docs.python.org/3.7/library/time.html for more details.

import time

# Monotonic is one of the several timing functions that gets the current time.
# The actual value returned by monotonic isn't useful by itself.
# Instead, you should only be finding the difference in time between two 
# calls to monotonic.
t1 = time.monotonic()

print("Current time:")
print("{0}s".format(t1))

# Wait 500 ms.
time.sleep(0.5)

# Get current time again.
t2 = time.monotonic()
msElapsed = (t2 - t1) * 10 ** 3 # Convert seconds to ms

print("500 ms later:")
print("{0}s, {1:.4}ms elapsed".format(t2, msElapsed))

# Wait 100 us. The Pi will most likely end up waiting significantly longer.
time.sleep(0.0001)

# Get current time again.
t3 = time.monotonic()
usElapsed = (t3 - t2) * 10 ** 6 # Convert seconds to us

print("100 us later:")
print("{0}s, {1:.4}us elapsed".format(t3, usElapsed))
