#!/bin/bash
xacro kuroko.xacro > export/kuroko.urdf;gz sdf -p export/kuroko.urdf > export/kuroko.sdf;sdf2usd export/kuroko.sdf export/kuroko.usd

