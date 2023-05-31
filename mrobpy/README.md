[![PyPi version](https://img.shields.io/pypi/v/mrob.svg)](https://pypi.org/project/mrob/)
[![PyPi downloads](https://img.shields.io/pypi/dm/mrob.svg)](https://pypi.org/project/mrob/)
[![Documentation Status](https://readthedocs.org/projects/mrob/badge/?version=latest)](https://mrob.readthedocs.io/en/latest/?badge=latest)

<p align="center">
  <img src="https://sites.skoltech.ru/app/data/uploads/sites/50/2018/02/mr_animate1.gif" width="450">
</p>

# MROB: Mobile Robotics library
The Mobile Robotics library (mrob) is our common framework for implementing our robotics research and projects. It includes a core set of functionalities such as geometric transformations (SE3), factor graphs for general state estimation, optimization, 3D point cloud registration and more to come.

MROB was designed in C++14 with the aim of using it in Python via bindings. 
## Installation
```
python -m pip install mrob
```
## Usage
```python
import mrob 
import numpy as np

graph = mrob.FGraph()
R = mrob.geometry.SO3(np.eye(3))
t = np.zeros(3)
pose = mrob.geometry.SE3(R, t)
```
## Examples
You can find more usage examples in [python_examples](https://github.com/prime-slam/mrob/tree/master/python_examples).

## Supported platforms
|         | macOS    | Linux         | Windows     |
|---------|----------|---------------|-------------|
| Version | 11+      | manylinux2014 | Windows2019 |
| Python  | 3.7-3.10 | 3.6-3.10      | 3.6-3.10    |

## License
Apache-2.0 License
