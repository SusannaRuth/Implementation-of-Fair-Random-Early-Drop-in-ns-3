# Implementation-of-Flow-Random-Early-Drop-in-ns-3
## Overview
FRED is an Active Queue Management algorithm which maintains the buffer count for each flow that has packets in the buffer and uses this state information so that the loss rate for each flow depends on its usage of the buffer [1]. In addition, FRED tries to identify non-adaptive flows and prevent them from utilising more share of the buffer and bandwidth. FRED is fairer than RED in handling different types of flows and it is already implemented in ns-2 [2]. This repository contains the implementation of FRED in ns-3 [3].
## References
[1] Dong Lin, Robert Morris.(1997). Dynamics of Random Early Detection. Proceedings of the ACM SIGCOMM '97 conference on Applications, technologies, architectures, and protocols for computer communication.

[2] http://www.isi.edu/nsnam/ns/

[3] http://www.nsnam.org/
