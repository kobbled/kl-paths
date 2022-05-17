#***************** ;
#Linear Move ;
#--------------- ;
#for moving robot linearly to ;
#next pose in karel path ;
#***************** ;

TP_GROUPMASK = "1,1,*,*,*"
TP_COMMENT = "scan move rot"

if termn < 0
  case poseType
    when PTH_TOOLING
      linear_move.to(pos).at(speed, 'mm/s').term(FINE).coord.
              time_after(0.0, turn_on(scan_trigger)).
              tool_offset(Tool_Offset)
    else
      linear_move.to(pos).at(speed, 'mm/s').term(FINE).coord.
              tool_offset(Tool_Offset)
  end
else
  case poseType
    when PTH_TOOLING
      linear_move.to(pos).at(speed, 'mm/s').term(termn).coord.
                  time_after(0.0, turn_on(scan_trigger)).
                  tool_offset(Tool_Offset)
    else
      linear_move.to(pos).at(speed, 'mm/s').term(termn).coord.
                  tool_offset(Tool_Offset)
  end
end
