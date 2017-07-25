# Yet Another Python Fuzzy Logic Module #

This package was created in an effort to get fuzzy with Python,
even if only a base Python installation is available. That being
said, Numpy will be supported (and become an optional dependency)
as soon as possible. 

YAPFLM was written from the perspective of a <span style="font-variant: small-caps">
Matlab</span> Fuzzy Logic Toolbox&trade; user who is trying to move to open-source
solutions. It fairly closely follows ML syntax and stores its FIS objects in
a similar fashion. Also included in the package is a FIS parser for parsing
<span style="font-variant: small-caps">Matlab</span> FIS files. This parsing
is dependent on the Python module [`parse`](https://pypi.python.org/pypi/parse "Parse Link")

A heavy emphasis on reduced copmutational complexity has been the abiding motivation behind
this implementation of a fuzzy inference system. There are no arrays for universes of discourse,
only membership function inverses for defuzzification.
