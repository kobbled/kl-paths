#***************** ;
#Linear Move ;
#--------------- ;
#for moving robot linearly to ;
#next pose in karel path ;
#***************** ;

TP_GROUPMASK = "1,*,*,*,*"
TP_COMMENT = "arc move"

case poseType
  when PTH_MOVETO
    linear_move.to(pos).at(speed, 'mm/s').term(FINE).
              tool_offset(Tool_Offset)
    circular_move.mid(pr1).to(pr2).at(speed, 'mm/s').term(termn).
                  tool_offset(Tool_Offset)
  when PTH_CLOSE
    circular_move.mid(pr1).to(pr2).at(speed, 'mm/s').term(termn).
                  tool_offset(Tool_Offset)
    linear_move.to(pos).at(speed, 'mm/s').term(FINE).
              tool_offset(Tool_Offset)
  else
    circular_move.mid(pr1).to(pr2).at(speed, 'mm/s').term(termn).
                  tool_offset(Tool_Offset)
end
