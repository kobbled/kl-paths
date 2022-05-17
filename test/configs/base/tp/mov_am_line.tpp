#***************** ;
#Linear Move ;
#--------------- ;
#for moving robot linearly to ;
#next pose in karel path ;
#***************** ;

TP_GROUPMASK = "1,*,*,*,*"
TP_COMMENT = "AM linear"

case poseCode
  when PTH_MOVETO

    case poseType
      when PTH_TOOLING
        linear_move.to(pos).at(speed, 'mm/s').term(FINE).
              time_after(0.0, RUN_LASER_START()).
              tool_offset(Tool_Offset)
      else
        linear_move.to(pos).at(speed, 'mm/s').term(FINE).
              tool_offset(Tool_Offset)
    end
    
  when PTH_CLOSE

    case poseType
      when PTH_TOOLING
        linear_move.to(pos).at(speed, 'mm/s').term(FINE).
              time_after(0.0, RUN_LASER_STOP()).
              tool_offset(Tool_Offset)
      else
        linear_move.to(pos).at(speed, 'mm/s').term(FINE).
              tool_offset(Tool_Offset)
    end

  else
    linear_move.to(pos).at(speed, 'mm/s').term(termn).
              tool_offset(Tool_Offset)
end
