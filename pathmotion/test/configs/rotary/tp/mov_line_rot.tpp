#***************** ;
#Linear Move ;
#--------------- ;
#for moving robot linearly to ;
#next pose in karel path ;
#***************** ;

TP_GROUPMASK = "1,1,*,*,*"
TP_COMMENT = "linear move"

if termn < 0
  linear_move.to(pos).at(speed, 'mm/s').term(FINE).coord.
              tool_offset(Tool_Offset)
else
  linear_move.to(pos).at(speed, 'mm/s').term(termn).coord.
              tool_offset(Tool_Offset)
end
