import numpy as np
import numpy.typing as npt
from typing import *

f64 = np.float64
f64array = npt.NDArray[f64]

type f64any = f64 | f64array | float