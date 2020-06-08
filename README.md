## Creating the FIR Filter

The FIR filter was created in Python using the `scipy.signal` module:

```python
import numpy
from scipy.signal import firwin

def create_filter(rate=62500, oversample=3, taps=24, bandwidth=500, window='hamming'):
    fs = rate//oversample
    nyquist = 0.5 * rate
    return firwin(taps, [fs-(bandwidth/2),fs+(bandwidth/2)], nyq=nyquist, pass_zero=False, window='hamming', scale=False)

# Disable scientific notation, so we can copy the values to C
numpy.set_printoptions(suppress=True, precision=15)

print(create_filter())

```

This results in a filter output such as:

```
array([ 0.000552267628092, -0.000000136206718, -0.001015894260281,
       -0.0015628464157  ,  0.000000329625115,  0.003089577068582,
        0.00395535506267 , -0.000000418414772, -0.005577484899362,
       -0.006212228801914,  0.000000193393475,  0.006898507563183,
        0.006898507563183,  0.000000193393475, -0.006212228801914,
       -0.005577484899362, -0.000000418414772,  0.00395535506267 ,
        0.003089577068582,  0.000000329625115, -0.0015628464157  ,
       -0.001015894260281, -0.000000136206718,  0.000552267628092])
```

