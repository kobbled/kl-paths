# environement variables for test motion scripts
# --------------
pos         := PR[85]
pr1         := PR[80]
pr2         := PR[81]
Tool_Offset := PR[20]

speed    := R[200]
termn    := R[255]
poseType := R[253]
poseCode := R[259]

# constants
# --------

#terms
FINE := -1
CNT  := 100

# codes
PTH_CLOSE  := 0
PTH_MOVETO := 1
PTH_LINETO := 2
PTH_CURVE3 := 3
PTH_CURVE4 := 4
PTH_STOP   := 79

# types
PTH_TOOLING := 100
PTH_LINKING := 105
